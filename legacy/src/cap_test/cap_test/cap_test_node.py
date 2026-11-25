# システム・前提
import rclpy
from rclpy.node import Node

# メッセージ
from sensor_msgs.msg import Image

# QoS
from rclpy.qos import qos_profile_sensor_data

# 外部
import cv2
import numpy as np


class CapTestNode(Node):
    def __init__(self):
        super().__init__("cap_test_node")

        self.declare_parameter('input_topic', '/camera/image_raw')
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(Image,input_topic,self._get_action,qos_profile_sensor_data)
    

    def _convert_msg_to_numpy(self, msg: Image) -> np.ndarray:
        """

        バイト列を画像に変換する。

        Args:
            msg (Image): メッセージ

        Returns:
            frame (np.ndarray): 画像

        """
        # 情報の取得
        h, w, c = msg.height, msg.width, 3
        enc = msg.encoding.lower()
        step = int(msg.step)

        # 変換
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        bytes_per_pixel = np.dtype(np.uint8).itemsize * c
        if step == w * bytes_per_pixel:
            frame = buf.reshape((h, w, c))
        else:
            row_view = buf.reshapre((h, step // np.dtype(np.uint8).itemsize))
            useful = row_view[:, : (w * c)]
            frame = useful.reshape((h, w, c))

        return frame

    def _get_action(self, msg: Image):
        """

        モデルを使用して正規化された操作量を取得する。

        Args:
            msg (Image): カメラから取得した画像

        """
        # 画像の取得
        frame = self._convert_msg_to_numpy(msg)

        self.get_logger().info(f"shape:{frame.shape}")


    

def main(args=None):
    rclpy.init(args=args)
    node = CapTestNode()
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