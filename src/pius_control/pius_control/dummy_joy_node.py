import rclpy
from rclpy.node import Node
from gamepad_msgs.msg import Gamepad, GamepadAxis, GamepadButtons
import yaml
import math
import time
import os

class DummyJoyNode(Node):
    def __init__(self):
        super().__init__('dummy_joy_node')
        
        # Load centralized configuration
        config_path = '/home/orin/dev/ws/colcon_ws/config.yaml'
        if not os.path.exists(config_path):
            self.get_logger().error(f"Configuration file not found at {config_path}!")
            # Fallback defaults
            self.pattern = "zigzag"
            self.pub_hz = 10.0
            self.output_topic = "/gamepad/state"
            self.verbose = 1
        else:
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)
            self.pattern = cfg['dummy_joy'].get('pattern', 'zigzag')
            self.pub_hz = float(cfg['dummy_joy'].get('pub_hz', 10.0))
            self.output_topic = cfg['dummy_joy'].get('output_topic', '/gamepad/state')
            self.verbose = int(cfg['dummy_joy'].get('verbose', 1))

        self.pub = self.create_publisher(Gamepad, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / self.pub_hz, self._publish_dummy_state)
        
        self.start_time = time.time()
        
        # --- ヘルスステータス配信用 ---
        from std_msgs.msg import String
        self.health_pub = self.create_publisher(String, '/system/health', 10)
        self.health_timer = self.create_timer(1.0, self._publish_health)

        self.get_logger().info(f"DummyJoyNode initialized with pattern: {self.pattern}, pub_hz: {self.pub_hz}")

    def _publish_health(self):
        from std_msgs.msg import String
        msg = String()
        msg.data = "gamepad:DUMMY"
        self.health_pub.publish(msg)

    def _publish_dummy_state(self):
        t = time.time() - self.start_time
        msg = Gamepad()
        msg.axis = GamepadAxis()
        msg.buttons = GamepadButtons()

        # Default values
        lx = 0.0  # steering
        ly = 0.0  # throttle
        
        # Apply patterns
        if self.pattern == "straight":
            lx = 0.0
            ly = 0.5
        elif self.pattern == "turn":
            lx = 0.6
            ly = 0.3
        elif self.pattern == "stop":
            lx = 0.0
            ly = 0.0
        elif self.pattern == "zigzag":
            # Sinusoidal steering wave with 5 second period, constant throttle
            lx = 0.6 * math.sin(2.0 * math.pi * 0.2 * t)
            ly = 0.4
        else:
            self.get_logger().warning(f"Unknown pattern: {self.pattern}")

        # Set axes
        msg.axis.lx = lx
        msg.axis.ly = ly
        msg.axis.rx = 0.0
        msg.axis.ry = 0.0
        msg.axis.l2 = 0.0
        msg.axis.r2 = ly  # parser_node reads throttle from the R2 trigger
        msg.axis.dpx = 0
        msg.axis.dpy = 0

        # Simulate share button toggle every 15 seconds to switch mode (manual/auto)
        # We will hold the button pressed for 0.5 seconds
        cycle_time = t % 15.0
        if cycle_time < 0.5:
            msg.buttons.share = True
        else:
            msg.buttons.share = False

        self.pub.publish(msg)
        if self.verbose >= 2:
            self.get_logger().info(f"Dummy Gamepad State Sent -> lx: {lx:.2f}, ly: {ly:.2f}, share_btn: {msg.buttons.share}")

def main(args=None):
    rclpy.init(args=args)
    node = DummyJoyNode()
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
