# システム・前提
import rclpy
from rclpy.node import Node

# メッセージ
from gamepad_msgs.msg import Gamepad, GamepadAxis, GamepadButtons
from std_msgs.msg import String

# 自作
from .dualsense import DualSense

class DualSenseNode(Node):
    def __init__(self):
        """

        ゲームパッドの状態をトピックとして送信するノード。
        切断されてもノードは終了せず、再接続を試み続ける（切断中は何も送信しない）。

        Parameters:
            output_topic (str): ゲームパッドの状態を送信するトピック名
            update_hz (float): ゲームパッドの状態を1秒間に何回更新するか
            pub_hz (float): ゲームパッドの状態を1秒間に何回送信するか
            reconnect_interval (float): 再接続を試みる間隔 [秒]
            verbose (int): ログ出力レベル

        """
        super().__init__('dualsense_node')

        # --- パラメータ定義 ---
        self.declare_parameter('output_topic', '/gamepad/state')
        self.declare_parameter('update_hz', 100.0)
        self.declare_parameter('pub_hz', 10.0)
        self.declare_parameter('reconnect_interval', 1.0)
        self.declare_parameter('verbose', 1)

        # --- パラメータ呼び出し ---
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        update_hz = self.get_parameter('update_hz').get_parameter_value().double_value
        pub_hz = self.get_parameter('pub_hz').get_parameter_value().double_value
        reconnect_interval = self.get_parameter('reconnect_interval').get_parameter_value().double_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value

        # --- パブリッシャー ---
        self.pub = self.create_publisher(Gamepad, self.output_topic, 10)
        self.health_pub = self.create_publisher(String, '/system/health', 10)
        self.notice_pub = self.create_publisher(String, '/system/notice', 10)

        self.gamepad = None
        self.state = None
        self.status = "RECONNECTING"
        self.status_detail = "起動中"
        self.failed_attempts = 0

        self._connect()

        self.pub_timer = self.create_timer(1/pub_hz, self._pub)
        self.gamepad_state_update_timer = self.create_timer(1/update_hz, self._update)
        self.reconnect_timer = self.create_timer(reconnect_interval, self._reconnect)
        self.health_timer = self.create_timer(1.0, self._publish_health)
        self._publish_health()  # 起動直後に即時パブリッシュ

    def _notice(self, level, text):
        msg = String()
        msg.data = f"{level}|gamepad|{text}"
        self.notice_pub.publish(msg)

    def _connect(self):
        try:
            self.gamepad = DualSense()
            self.state = self.gamepad.read()  # 初期状態の取得
        except Exception as e:
            self.gamepad = None
            self.failed_attempts += 1
            self.status_detail = str(e)
            # 毎秒の再試行でログが埋まらないよう、初回と以降10回ごとに出力する
            if self.failed_attempts == 1 or self.failed_attempts % 10 == 0:
                self.get_logger().error(f'【接続失敗】DualSenseコントローラーが見つかりません。USB接続かBluetoothペアリングの状態を確認してください。再接続を試行中です... (試行{self.failed_attempts}回目: {e})')
            return False

        was_reconnect = self.status == "RECONNECTING" and self.status_detail != "起動中"
        self.status = "OK"
        self.status_detail = self.gamepad.path
        self.failed_attempts = 0
        if was_reconnect:
            self.get_logger().info(f'【再接続成功】DualSenseコントローラーに再接続しました。(トピック: {self.output_topic})')
            self._notice("INFO", "コントローラーに再接続しました")
        elif self.verbose >= 1:
            self.get_logger().info(f'【接続成功】DualSenseコントローラーを接続しました。(トピック: {self.output_topic})')
        self._publish_health()
        return True

    def _handle_disconnect(self, error):
        if self.gamepad is not None:
            self.gamepad.close()
        self.gamepad = None
        self.state = None
        self.status = "RECONNECTING"
        self.status_detail = str(error)
        self.get_logger().error(f'【接続断】DualSenseコントローラーとの接続が切れました: {error}。操作の送信を停止し、再接続を試みます。')
        self._notice("ERROR", "コントローラーとの接続が切れました。再接続を試みています（車両は安全停止します）")
        self._publish_health()

    def _reconnect(self):
        if self.gamepad is None:
            self._connect()

    def _update(self):
        """

        ゲームパッドを読み取って状態を更新する。

        """
        if self.gamepad is None:
            return
        try:
            self.state = self.gamepad.read()
        except OSError as e:
            self._handle_disconnect(e)

    def _pub(self):
        """

        現在の状態を送信する。切断中は送信しない（下流のタイムアウトで安全停止させるため）。

        """
        if self.gamepad is None or self.state is None:
            return

        msg = Gamepad()
        axis = self.state["axis"]
        buttons = self.state["buttons"]

        # 連続値対応
        msg.axis = GamepadAxis()
        msg.axis.lx = axis["lx"]
        msg.axis.ly = axis["ly"]
        msg.axis.rx = axis["rx"]
        msg.axis.ry = axis["ry"]
        msg.axis.l2 = axis["l2"]
        msg.axis.r2 = axis["r2"]
        msg.axis.dpx = axis["dpx"]
        msg.axis.dpy = axis["dpy"]

        # 離散値対応
        msg.buttons = GamepadButtons()
        msg.buttons.cross = buttons["cross"]
        msg.buttons.circle = buttons["circle"]
        msg.buttons.square = buttons["square"]
        msg.buttons.triangle = buttons["triangle"]
        msg.buttons.l1 = buttons["l1"]
        msg.buttons.l2 = buttons["l2"]
        msg.buttons.l3 = buttons["l3"]
        msg.buttons.r1 = buttons["r1"]
        msg.buttons.r2 = buttons["r2"]
        msg.buttons.r3 = buttons["r3"]
        msg.buttons.touch = buttons["touch"]
        msg.buttons.share = buttons["share"]
        msg.buttons.option = buttons["option"]
        msg.buttons.ps = buttons["ps"]

        # パブリッシュ
        self.pub.publish(msg)

        # ログ出力
        if self.verbose >= 2:
            self.get_logger().info(f'Message Sent (Gamepad): {msg}')

    def _publish_health(self):
        msg = String()
        msg.data = f"gamepad:{self.status}:{self.status_detail}"
        self.health_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DualSenseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.gamepad is not None:
            node.gamepad.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
