import json
import os
import threading
import time

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from pius_control.wheel_estimator import WheelSpeedEstimator


class WheelSpeedNode(Node):
    """
    Wheel speed sensor: a reed switch on the rear-left brake disc, buffered by a 74HC14 into a GPIO
    (High while the magnet is at the switch, 1 pulse per wheel revolution).

    The pin is polled (~1 kHz) and a pulse is counted only when the level has stayed High for stable_ms,
    which removes short electrical noise and contact bounce. Each pulse is then checked against physics and
    the IMU (wheel_estimator.py): valid pulses go to /wheel/pulse (JSON {"count", "stamp", "interval"}) for
    data recording, rejected ones to /wheel/pulse_rejected with the reason. The speed on /wheel/status
    (JSON, 10 Hz) integrates the IMU between pulses, so it is continuous at start and stop; without the IMU
    it falls back to the pulses only. The sensor is optional and never affects driving.
    """

    def __init__(self):
        super().__init__('wheel_speed_node')

        config_path = '/home/orin/dev/ws/colcon_ws/config.yaml'
        cfg = {}
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f).get('wheel', {})
        else:
            self.get_logger().error(f"Configuration file not found at {config_path}!")
        self.pin = int(cfg.get('pin', 7))
        self.circumference = float(cfg.get('circumference_m', 0.0))
        self.pulses_per_rev = max(1, int(cfg.get('pulses_per_rev', 1)))   # magnets on the wheel
        self.stable_ms = float(cfg.get('stable_ms', 10))
        self.verbose = int(cfg.get('verbose', 1))
        axis = cfg.get('imu_forward_axis', [-0.338, 0.941])
        self.est = WheelSpeedEstimator(
            self.circumference,
            max_speed=float(cfg.get('max_speed_kmh', 8.0)) / 3.6,
            forward_axis=(float(axis[0]), float(axis[1])),
            stop_timeout=float(cfg.get('stop_timeout_sec', 3.0)),
            pulses_per_rev=self.pulses_per_rev,
            min_distance_ratio=float(cfg.get('min_distance_ratio', 0.6)),
            jump_margin=float(cfg.get('jump_margin_mps', 0.4)),
        )

        self.pub_pulse = self.create_publisher(String, cfg.get('pulse_topic', '/wheel/pulse'), 50)
        self.pub_rejected = self.create_publisher(String, cfg.get('rejected_topic', '/wheel/pulse_rejected'), 50)
        self.pub_status = self.create_publisher(String, cfg.get('status_topic', '/wheel/status'), 10)
        self.health_pub = self.create_publisher(String, '/system/health', 10)
        self.create_subscription(Imu, cfg.get('imu_topic', '/imu/data'), self._on_imu, 100)

        self.lock = threading.Lock()
        self.status = "ERROR"
        self.status_detail = ""
        self.gpio = None
        self.running = True

        try:
            import Jetson.GPIO as GPIO
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.pin, GPIO.IN)
            self.gpio = GPIO
            self.status = "OK"
            threading.Thread(target=self._poll_loop, daemon=True).start()
            circ = (f"外周 {self.circumference:.3f} m・1 回転 {self.pulses_per_rev} パルス" if self.circumference > 0
                    else "外周未設定（パルスのみ記録）")
            self.get_logger().info(f"【車速センサー】BOARD {self.pin} 番ピンの監視を開始しました"
                                   f"（{circ}、{self.stable_ms:.0f} ms 安定でパルスと判定、IMU で補正）")
        except Exception as e:
            self.status_detail = f"GPIO を使えません: {e}"
            self.get_logger().error(f"【車速センサー】{self.status_detail}（走行・記録には影響しません）")

        if self.circumference <= 0 and self.gpio is not None:
            self.get_logger().warning("【車速センサー】config.yaml の wheel.circumference_m が未設定のため、速度は計算しません。")

        self.status_timer = self.create_timer(0.1, self._publish_status)
        self.health_timer = self.create_timer(1.0, self._publish_health)
        self._publish_health()

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _poll_loop(self):
        """Count a pulse when the level has been High for stable_ms after being Low for stable_ms."""
        stable = self.stable_ms / 1000.0
        level = self.gpio.input(self.pin)
        filtered = level                 # if the magnet is at the switch at startup, wait for it to leave
        changed = time.monotonic()
        while self.running:
            try:
                now = time.monotonic()
                raw = self.gpio.input(self.pin)
                if raw != level:
                    level, changed = raw, now
                elif level != filtered and now - changed >= stable:
                    filtered = level
                    if filtered == 1:
                        # Time stamp of the rising edge itself, not of the end of the stable period
                        self._on_pulse(self._now() - (now - changed))
                time.sleep(0.001)
            except Exception as e:
                if self.running:
                    self.status = "ERROR"
                    self.status_detail = f"GPIO の読み取りに失敗: {e}"
                    self.get_logger().error(f"【車速センサー】{self.status_detail}")
                return

    def _on_imu(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 or self._now()
        a, g = msg.linear_acceleration, msg.angular_velocity
        with self.lock:
            self.est.on_imu(stamp, (a.x, a.y, a.z), (g.x, g.y, g.z))

    def _on_pulse(self, stamp):
        with self.lock:
            ok, reason, interval = self.est.on_pulse(stamp)
            count, rejected = self.est.count, self.est.rejected
        msg = String()
        if ok:
            msg.data = json.dumps({"count": count, "stamp": stamp, "interval": interval})
            self.pub_pulse.publish(msg)
        else:
            msg.data = json.dumps({"stamp": stamp, "interval": interval, "reason": reason, "rejected": rejected})
            self.pub_rejected.publish(msg)
        if self.verbose >= 2:
            self.get_logger().info(f"Wheel pulse {'OK' if ok else 'rejected (' + reason + ')'} interval={interval}")

    def _publish_status(self):
        if self.gpio is None:
            return
        now = self._now()
        with self.lock:
            speed, source = self.est.speed(now)
            status = {
                "speed": speed,                         # best estimate [m/s] (IMU-assisted when available)
                "speed_pulse": self.est.pulse_speed(now),   # pulses only (previous method)
                "source": source,                       # "imu" or "pulse"
                "direction": self.est.direction(now),   # +1 forward / -1 reverse / 0 stopped or unknown
                "still": self.est.still,
                "count": self.est.count,
                "rejected": self.est.rejected,
                "last_pulse_age": None if self.est.last_pulse is None else now - self.est.last_pulse,
                "circumference": self.circumference,
                "pulses_per_rev": self.pulses_per_rev,
                "pulse_distance": self.est.pulse_distance,  # distance between two pulses [m]
                "imu_forward_axis": list(self.est.fwd),     # car forward direction in IMU x/y (for offline replays)
            }
        msg = String()
        msg.data = json.dumps(status)
        self.pub_status.publish(msg)

    def _publish_health(self):
        if self.gpio is not None and self.status == "OK":
            now = self._now()
            with self.lock:
                speed, source = self.est.speed(now)
                count, rejected = self.est.count, self.est.rejected
            speed_text = "速度不明" if speed is None else f"{speed * 3.6:.1f}km/h"
            src = "IMU併用" if source == "imu" else "パルスのみ"
            self.status_detail = f"{speed_text}（{src}）・累計{count}パルス・除外{rejected}" + \
                ("" if self.circumference > 0 else "・外周未設定")
        msg = String()
        msg.data = f"wheel:{self.status}:{self.status_detail}"
        self.health_pub.publish(msg)

    def close(self):
        self.running = False
        if self.gpio is not None:
            time.sleep(0.01)
            self.gpio.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = WheelSpeedNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
