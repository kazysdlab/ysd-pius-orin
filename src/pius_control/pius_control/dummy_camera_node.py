import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import yaml
import os
import time

class DummyCameraNode(Node):
    def __init__(self):
        super().__init__('dummy_camera_node')
        
        # Load configuration
        config_path = '/home/orin/dev/ws/colcon_ws/config.yaml'
        if not os.path.exists(config_path):
            self.get_logger().error(f"Configuration file not found at {config_path}!")
            self.output_topic = "/camera/image_raw"
            self.resize = [84, 84]
            self.pub_hz = 15.0
            self.verbose = 1
        else:
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f)
            self.output_topic = cfg['camera'].get('output_topic', '/camera/image_raw')
            self.resize = cfg['camera'].get('resize', [84, 84])
            self.pub_hz = float(cfg['camera'].get('pub_hz', 15.0))
            self.verbose = int(cfg['camera'].get('verbose', 1))

        self.pub = self.create_publisher(Image, self.output_topic, rclpy.qos.qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / self.pub_hz, self._publish_frame)
        self.start_time = time.time()
        
        # --- ヘルスステータス配信用 ---
        from std_msgs.msg import String
        self.health_pub = self.create_publisher(String, '/system/health', 10)
        self.health_timer = self.create_timer(1.0, self._publish_health)

        self.get_logger().info(f"DummyCameraNode initialized. Output shape: {self.resize}, Pub Hz: {self.pub_hz}")

    def _publish_health(self):
        from std_msgs.msg import String
        msg = String()
        msg.data = "camera:DUMMY"
        self.health_pub.publish(msg)

    def _publish_frame(self):
        t = time.time() - self.start_time
        w, h = self.resize
        
        # Generate dummy image: black background with a shifting vertical white lane (simulating a road)
        frame = np.zeros((h, w, 3), dtype=np.uint8)
        
        # Lane center shifts sinusoidally
        lane_center = int((w / 2) + (w / 3) * np.sin(2 * np.pi * 0.1 * t))
        lane_width = int(w / 10) + 1
        
        # Draw lane
        left_bound = max(0, lane_center - lane_width)
        right_bound = min(w, lane_center + lane_width)
        frame[:, left_bound:right_bound, :] = 255  # White lane

        # Sensor-like noise, like a real camera (save_node treats bit-identical frames as a frozen feed)
        frame = np.clip(frame.astype(np.int16) + np.random.randint(-4, 5, frame.shape), 0, 255).astype(np.uint8)

        # Convert to Image message
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "dummy_camera"
        msg.height = h
        msg.width = w
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = w * 3
        msg.data = frame.tobytes()

        self.pub.publish(msg)
        if self.verbose >= 2:
            self.get_logger().info("Dummy frame sent.")

def main(args=None):
    rclpy.init(args=args)
    node = DummyCameraNode()
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
