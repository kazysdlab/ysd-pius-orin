import rclpy
from rclpy.node import Node
from control_msgs.msg import NormalizedControls, PwmControls, ChannelPwm
from std_msgs.msg import String
import serial
import yaml
import os
import time
import struct

# PIUS brake PWM polarity (verified on the vehicle in 3ddf092): 1135 = full brake, 1948 = released.
BRAKE_PWM_FULL = 1135
BRAKE_PWM_RELEASED = 1948
PWM_STEERING_CENTER = 1500
PWM_THROTTLE_STOP = 1000
STATE_FORWARD, STATE_NEUTRAL, STATE_REVERSE = 0, 1, 2


def pack_pius_packet(steering, throttle, brake, state):
    """Header 0xFFFF + 4 x int16 + checksum (sum of payload bytes & 0x7F, never 0xFF). 11 bytes."""
    payload = struct.pack('<4h', steering, throttle, brake, state)
    checksum = sum(payload) & 0x7F
    return struct.pack('<H', 0xFFFF) + payload + bytes([checksum])


class ShiftSequencer:
    """Shift (D / N / R) only while the accelerator is off (PIUS 補足資料, 注意事項 2).

    The direction follows the sign of the throttle. A direction is changed only after the throttle has been
    off (1000 us) for hold_sec, and the throttle stays off for hold_sec after the change, so the motor
    controller never sees a shift with the accelerator on. D <-> R always goes through N. When the throttle is
    released the direction is kept for neutral_delay_sec, so that a speed controller hovering near zero
    throttle does not shift D <-> N on every step. force_neutral (command timeout) shifts to N at once.
    """

    def __init__(self, hold_sec=0.3, neutral_delay_sec=0.5):
        self.hold = float(hold_sec)
        self.neutral_delay = max(float(neutral_delay_sec), self.hold)
        self.state = STATE_NEUTRAL
        self.changed_at = None
        self.off_since = float('-inf')      # the throttle has been off since (None while it is on)

    def _set(self, state, now):
        if state != self.state:
            self.state = state
            self.changed_at = now

    def update(self, now, throttle, force_neutral=False):
        """Returns (state, throttle_allowed). When not allowed, send the throttle as 1000 us."""
        want = STATE_FORWARD if throttle > 0.01 else STATE_REVERSE if throttle < -0.01 else None
        settled = self.changed_at is None or now - self.changed_at >= self.hold
        if not force_neutral and want is not None and want == self.state and settled:
            self.off_since = None
            return self.state, True
        if self.off_since is None:
            self.off_since = now
        off_for = now - self.off_since
        if force_neutral:
            self._set(STATE_NEUTRAL, now)
        elif want is None:
            if self.state != STATE_NEUTRAL and settled and off_for >= self.neutral_delay:
                self._set(STATE_NEUTRAL, now)
        elif want != self.state and settled and off_for >= self.hold:
            self._set(want if self.state == STATE_NEUTRAL else STATE_NEUTRAL, now)
        return self.state, False


class SenderNode(Node):
    def __init__(self):
        super().__init__('sender_node')
        
        # Load configuration
        config_path = '/home/orin/dev/ws/colcon_ws/config.yaml'
        if not os.path.exists(config_path):
            self.get_logger().error(f"Configuration file not found at {config_path}!")
            self.protocol = "pius"
            self.port = "/dev/ttyTHS1"
            self.baudrate = 115200
            self.write_hz = 30.0
            self.msg_format = "S{steering_pwm}T{throttle_pwm}\n"
            self.left_pwm, self.right_pwm, self.center_pwm = 290, 490, 390
            self.forward_pwm, self.stopped_pwm, self.reverse_pwm = 500, 370, 220
            self.shift_hold_sec, self.neutral_delay_sec = 0.3, 0.5
            self.verbose = 1
        else:
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)
            self.protocol = cfg['sender'].get('protocol', 'pius')
            
            # Default port depends on protocol if not explicitly specified
            default_port = "/dev/ttyTHS1" if self.protocol == "pius" else "/dev/ttyUSB0"
            self.port = cfg['sender'].get('port', default_port)
            self.baudrate = int(cfg['sender'].get('baudrate', 115200))
            self.write_hz = float(cfg['sender'].get('write_hz', 30.0))
            self.msg_format = cfg['sender'].get('message_format', 'S{steering_pwm}T{throttle_pwm}\n')
            
            # Donkey Car Steering mapping constants
            st_cfg = cfg['sender']['steering']
            self.left_pwm = int(st_cfg.get('left_pwm', 290))
            self.right_pwm = int(st_cfg.get('right_pwm', 490))
            self.center_pwm = int(st_cfg.get('center_pwm', 390))
            
            # Donkey Car Throttle mapping constants
            th_cfg = cfg['sender']['throttle']
            self.forward_pwm = int(th_cfg.get('forward_pwm', 500))
            self.stopped_pwm = int(th_cfg.get('stopped_pwm', 370))
            self.reverse_pwm = int(th_cfg.get('reverse_pwm', 220))
            
            self.shift_hold_sec = float(cfg['sender'].get('shift_hold_sec', 0.3))
            self.neutral_delay_sec = float(cfg['sender'].get('neutral_delay_sec', 0.5))
            self.verbose = int(cfg['sender'].get('verbose', 1))

        # Initial normalized values (Safe state: stopped with full brake)
        self.latest_steering = 0.0
        self.latest_throttle = 0.0
        self.latest_brake = 1.0
        self.latest_msg_time = 0.0
        self.timed_out = True
        self.shift = ShiftSequencer(self.shift_hold_sec, self.neutral_delay_sec)

        # Launch argument dry_run:=true forces DRY RUN even when the port is available
        self.declare_parameter('dry_run', False)
        force_dry_run = self.get_parameter('dry_run').get_parameter_value().bool_value

        # Try to open the serial port
        self.ser = None
        self.dry_run = False
        self.dry_run_reason = ""
        if force_dry_run:
            self.dry_run = True
            self.dry_run_reason = "launch引数 dry_run:=true"
            self.get_logger().warning("【シリアル警告】dry_run:=true が指定されたため、シリアル送信なし（DRY RUNモード）で起動します。")
        else:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
                self.get_logger().info(f"【接続成功】車両制御ポート {self.port} のオープンに成功しました。(ボーレート: {self.baudrate}, プロトコル: {self.protocol.upper()})")
            except Exception as e:
                self.get_logger().warning(f"【シリアル警告】車両制御ポート {self.port} をオープンできませんでした: {e}。シリアル送信なし（DRY RUNモード）で起動します。")
                self.dry_run = True
                self.dry_run_reason = str(e)

        # --- ヘルスステータス配信用 ---
        self.health_pub = self.create_publisher(String, '/system/health', 10)
        self.health_timer = self.create_timer(1.0, self._publish_health)
        self._publish_health()  # 起動直後に即時パブリッシュ

        # Actual PWM values sent to the vehicle (for the dashboard)
        self.pwm_pub = self.create_publisher(PwmControls, '/vehicle/pwm', 10)

        # Subscription and Timer
        self.sub = self.create_subscription(NormalizedControls, '/cmd/normalized', self._controls_callback, 10)
        self.timer = self.create_timer(1.0 / self.write_hz, self._write_to_serial)

    def _publish_health(self):
        msg = String()
        if self.dry_run:
            msg.data = f"serial:DRY_RUN:{self.dry_run_reason}"
        else:
            msg.data = f"serial:OK:{self.port}"
        self.health_pub.publish(msg)

    def _publish_pwm(self, values):
        msg = PwmControls()
        msg.data = [ChannelPwm(channel=ch, pwm=int(v)) for ch, v in enumerate(values)]
        self.pwm_pub.publish(msg)

    def _map_range(self, val, src_min, src_max, dst_min, dst_max):
        # Linear range mapping formula
        return dst_min + (val - src_min) * (dst_max - dst_min) / (src_max - src_min)

    def _controls_callback(self, msg: NormalizedControls):
        self.latest_msg_time = time.time()
        
        for channel_data in msg.data:
            if channel_data.channel == 0:
                self.latest_steering = channel_data.normalized
            elif channel_data.channel == 1:
                self.latest_throttle = channel_data.normalized
            elif channel_data.channel == 2:
                self.latest_brake = channel_data.normalized

    def _write_to_serial(self):
        # Safety Timeout: If no message received for > 1.0 second, stop with full brake
        if time.time() - self.latest_msg_time > 1.0:
            if not self.timed_out:
                self.get_logger().warning("【安全停止】操作指令が1秒以上届いていません。スロットル停止・フルブレーキを出力します。")
                self.timed_out = True
            self.latest_steering = 0.0
            self.latest_throttle = 0.0
            self.latest_brake = 1.0
        elif self.timed_out:
            self.get_logger().info("【安全停止解除】操作指令の受信を再開しました。")
            self.timed_out = False

        if self.protocol == "pius":
            # ------------------ PIUS BINARY PROTOCOL ------------------
            st_val = self.latest_steering
            th_val = self.latest_throttle

            # 1. Steering calculation (input range -1.0 to 1.0)
            # Left: [-1.0, 0.0] -> [1.139, 1.5]
            # Right: [0.0, 1.0] -> [1.5, 1.835]
            if st_val > 0.0:
                st_pwm = st_val * (1.835 - 1.5) + 1.5
            elif st_val < 0.0:
                st_pwm = (1.0 + st_val) * (1.5 - 1.139) + 1.139
            else:
                st_pwm = 1.5
            st_out = int(clip(st_pwm * 1000.0, 1139.0, 1835.0))

            # 2. Throttle calculation (input range -1.0 to 1.0)
            # Both forward and reverse throttle values are sent as absolute values
            th_pwm = abs(th_val) * (2.0 - 1.0) + 1.0
            th_out = int(clip(th_pwm * 1000.0, 1000.0, 2000.0))

            # 3. Brake calculation (input range 0.0 to 1.0)
            # brake=0.0 -> released (1948), brake=1.0 -> full brake (1135)
            br_val = clip(self.latest_brake, 0.0, 1.0)
            br_pwm = BRAKE_PWM_RELEASED - (br_val * (BRAKE_PWM_RELEASED - BRAKE_PWM_FULL))
            br_out = int(clip(br_pwm, BRAKE_PWM_FULL, BRAKE_PWM_RELEASED))

            # 4. State/Shift: 0 = Forward, 1 = Neutral, 2 = Reverse, from the sign of the throttle.
            # Shifting only with the accelerator off (ShiftSequencer): the throttle is held at 1000 while shifting.
            state_out, throttle_ok = self.shift.update(time.time(), th_val, force_neutral=self.timed_out)
            if not throttle_ok:
                th_out = PWM_THROTTLE_STOP

            data_bytes = pack_pius_packet(st_out, th_out, br_out, state_out)
            self._publish_pwm([st_out, th_out, br_out, state_out])

            if self.dry_run:
                if self.verbose >= 2:
                    self.get_logger().info(f"[DRY RUN] PIUS Send -> steer={st_out}, throttle={th_out}, brake={br_out}, state={state_out} (Bytes: {data_bytes.hex()})")
            else:
                try:
                    self.ser.write(data_bytes)
                    if self.verbose >= 2:
                        self.get_logger().info(f"PIUS Send -> steer={st_out}, throttle={th_out}, brake={br_out}, state={state_out}")
                except Exception as e:
                    self.get_logger().error(f"Serial write error: {e}")

        else:
            # ------------------ DONKEY CAR TEXT PROTOCOL ------------------
            st_val = self.latest_steering
            th_val = self.latest_throttle

            # 1. Map steering (range -1.0 to 1.0)
            if st_val < 0.0:
                st_pwm = self._map_range(st_val, -1.0, 0.0, self.left_pwm, self.center_pwm)
            else:
                st_pwm = self._map_range(st_val, 0.0, 1.0, self.center_pwm, self.right_pwm)
            st_min = min(self.left_pwm, self.right_pwm)
            st_max = max(self.left_pwm, self.right_pwm)
            target_steering_pwm = int(clip(st_pwm, st_min, st_max))

            # 2. Map throttle (range -1.0 to 1.0)
            if th_val < 0.0:
                th_pwm = self._map_range(th_val, -1.0, 0.0, self.reverse_pwm, self.stopped_pwm)
            else:
                th_pwm = self._map_range(th_val, 0.0, 1.0, self.stopped_pwm, self.forward_pwm)
            th_min = min(self.reverse_pwm, self.forward_pwm)
            th_max = max(self.reverse_pwm, self.forward_pwm)
            target_throttle_pwm = int(clip(th_pwm, th_min, th_max))

            self._publish_pwm([target_steering_pwm, target_throttle_pwm])

            # Format text message
            cmd_str = self.msg_format.format(
                steering_pwm=target_steering_pwm,
                throttle_pwm=target_throttle_pwm
            )

            if self.dry_run:
                if self.verbose >= 2:
                    self.get_logger().info(f"[DRY RUN] Donkey Send -> {cmd_str.strip()}")
            else:
                try:
                    self.ser.write(cmd_str.encode('utf-8'))
                    if self.verbose >= 2:
                        self.get_logger().info(f"Donkey Send -> {cmd_str.strip()}")
                except Exception as e:
                    self.get_logger().error(f"Serial write error: {e}")

    def shutdown_vehicle(self):
        # Safety shutdown sequence before closing serial port
        if self.ser is not None and not self.dry_run:
            try:
                if self.protocol == "pius":
                    safe_bytes = pack_pius_packet(PWM_STEERING_CENTER, PWM_THROTTLE_STOP, BRAKE_PWM_FULL, STATE_NEUTRAL)
                    self.ser.write(safe_bytes)
                    self.ser.flush()
                else:
                    # Donkey safety state: neutral steering, stopped throttle
                    safe_str = self.msg_format.format(
                        steering_pwm=self.center_pwm,
                        throttle_pwm=self.stopped_pwm
                    )
                    self.ser.write(safe_str.encode('utf-8'))
                self.get_logger().info("【シリアル切断】車両へ安全停止パケットを送信しました。")
            except Exception as e:
                self.get_logger().error(f"【切断エラー】安全停止パケットの送信に失敗しました: {e}")

def clip(val, val_min, val_max):
    return max(val_min, min(val, val_max))

def main(args=None):
    rclpy.init(args=args)
    node = SenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_vehicle()
        if node.ser is not None:
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
