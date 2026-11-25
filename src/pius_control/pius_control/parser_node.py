import rclpy
from rclpy.node import Node
from gamepad_msgs.msg import Gamepad
from control_msgs.msg import NormalizedControls, ChannelNormalized
from std_msgs.msg import String, Bool
import yaml
import os

class ParserNode(Node):
    def __init__(self):
        super().__init__('parser_node')
        
        # Load configuration
        config_path = '/home/orin/dev/ws/colcon_ws/config.yaml'
        if not os.path.exists(config_path):
            self.get_logger().error(f"Configuration file not found at {config_path}!")
            self.input_topic = "/gamepad/state"
            self.output_topic_manual = "/cmd/normalized/manual"
            self.output_topic_mode = "/cmd/mode"
            self.output_topic_record = "/cmd/record"
            self.output_topic_shift = "/cmd/shift"
            self.steering_channel = 0
            self.throttle_channel = 1
            self.verbose = 1
        else:
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)
            self.input_topic = cfg['parser'].get('input_topic', '/gamepad/state')
            self.output_topic_manual = cfg['parser'].get('output_topic_manual', '/cmd/normalized/manual')
            self.output_topic_mode = cfg['parser'].get('output_topic_mode', '/cmd/mode')
            self.output_topic_record = cfg['parser'].get('output_topic_record', '/cmd/record')
            self.output_topic_shift = cfg['parser'].get('output_topic_shift', '/cmd/shift')
            self.steering_channel = int(cfg['parser'].get('steering_channel', 0))
            self.throttle_channel = int(cfg['parser'].get('throttle_channel', 1))
            self.verbose = int(cfg['parser'].get('verbose', 1))

        self.sub = self.create_subscription(Gamepad, self.input_topic, self._gamepad_callback, 10)
        self.pub_manual = self.create_publisher(NormalizedControls, self.output_topic_manual, 10)
        self.pub_mode = self.create_publisher(String, self.output_topic_mode, 10)
        self.pub_record = self.create_publisher(Bool, self.output_topic_record, 10)
        self.pub_shift = self.create_publisher(String, self.output_topic_shift, 10)
        self.health_pub = self.create_publisher(String, '/system/health', 10)
        self.health_timer = self.create_timer(1.0, self._publish_health)

        # The driving mode itself is owned by mux_node; SHARE only sends a "toggle" request
        self.prev_share_pressed = False
        
        self.logging_enabled = True # Enabled by default, toggled by OPTIONS button
        self.prev_option_pressed = False

        self.is_reverse = False
        self.prev_cross_pressed = False
        
        self.get_logger().info(f"【操作解析】コントローラー入力解析ノードを初期化しました。(ステアリング: {self.steering_channel}, スロットル: {self.throttle_channel})")

    def _publish_health(self):
        msg = String()
        msg.data = "parser:OK"
        self.health_pub.publish(msg)

    def _gamepad_callback(self, msg: Gamepad):
        # 1. Request a mode toggle on the rising edge of SHARE (mux decides whether it is allowed)
        share_pressed = msg.buttons.share
        if share_pressed and not self.prev_share_pressed:
            self.get_logger().info("【運転モード】SHAREボタンが押されました。走行モードの切替を要求します。")
            mode_msg = String()
            mode_msg.data = "toggle"
            self.pub_mode.publish(mode_msg)
        self.prev_share_pressed = share_pressed

        # 2. Parse and toggle logging/recording (OPTION button)
        option_pressed = msg.buttons.option
        if option_pressed and not self.prev_option_pressed:
            self.logging_enabled = not self.logging_enabled
            self.get_logger().info(f"【データ収集】記録状態を変更しました: {'[記録ON]' if self.logging_enabled else '[記録OFF]'}")
            
            record_msg = Bool()
            record_msg.data = self.logging_enabled
            self.pub_record.publish(record_msg)
            
        self.prev_option_pressed = option_pressed

        # Publish the logging status
        record_msg = Bool()
        record_msg.data = self.logging_enabled
        self.pub_record.publish(record_msg)

        # 3. Shift (D/R) toggle calculation (Cross button)
        cross_pressed = msg.buttons.cross
        if cross_pressed and not self.prev_cross_pressed:
            self.is_reverse = not self.is_reverse
            self.get_logger().info(f"【シフト切替】{'[リバース (REVERSE)]' if self.is_reverse else '[ドライブ (DRIVE)]'} に切り替わりました。")
        self.prev_cross_pressed = cross_pressed

        shift_msg = String()
        shift_msg.data = "R" if self.is_reverse else "D"
        self.pub_shift.publish(shift_msg)

        # 4. Parse and publish normalized manual controls
        controls_msg = NormalizedControls()
        
        steering_data = ChannelNormalized()
        steering_data.channel = self.steering_channel
        steering_data.normalized = msg.axis.lx
        
        # Acceleration using R2 trigger (range: [0.0, 1.0])
        # If in reverse state, throttle is negative (range: [-1.0, 0.0])
        r2_val = msg.axis.r2
        throttle_val = -r2_val if self.is_reverse else r2_val

        throttle_data = ChannelNormalized()
        throttle_data.channel = self.throttle_channel
        throttle_data.normalized = throttle_val
        
        # DualSense L2 trigger for independent brake (range: [0.0, 1.0])
        brake_data = ChannelNormalized()
        brake_data.channel = 2  # Brake channel
        brake_data.normalized = msg.axis.l2
        
        controls_msg.data = [steering_data, throttle_data, brake_data]
        self.pub_manual.publish(controls_msg)

        if self.verbose >= 2:
            shift_str = "R" if self.is_reverse else "D"
            self.get_logger().info(f"Controls -> Record: {self.logging_enabled}, Shift: {shift_str}, Steer: {msg.axis.lx:.2f}, Throttle: {throttle_val:.2f}, Brake: {msg.axis.l2:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = ParserNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
