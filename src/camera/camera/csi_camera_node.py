# システム・前提
import os
import threading
import time

import rclpy
from rclpy.node import Node

# メッセージ
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

# QoS
from rclpy.qos import qos_profile_sensor_data

# 外部
import cv2
import numpy as np

# 自作
from .csi_camera import CSICamera

class CSICameraNode(Node):
    def __init__(self):
        """

        CSICameraの画像を定期的にパブリッシュするノード。
        接続が切れてもノードは終了せず、通知を出したうえで再接続を試み続ける。

        Parameters:
            output_topic (str): ガメラ画像の送信先
            sensor_id (id): カメラ番号 (/dev/video?の値)
            capture_size (list[int]): カメラから取得する画像のサイズ
            capture_fps (int): カメラ画像の更新頻度
            resize (list[int]): モデル用画像のサイズ [幅, 高さ]
            crop (list[int]): モデル用画像に使う範囲 [x, y, 幅, 高さ]（capture_size の座標。空を除くため）。空なら全体
            flip_method (int): 画像の回転方向
            nvargs (str): パイプラインの追加オプション
            frame_id (str): カメラ画像送信時のメタデータ
            pub_hz (float): カメラ画像を1秒間に何回送信するか
            stall_timeout (float): この秒数フレームが届かなければ取得処理が固まったとみなす
            max_reconnect_interval (float): 再接続の試行間隔の上限 [秒]
            restart_after_failures (int): この回数続けて接続に失敗したらノードを再起動する（0で無効）
            preview_topic (str): ダッシュボード表示用の画像（JPEG, sensor_msgs/CompressedImage）の送信先
            preview_hz (float): 表示用画像を1秒間に何回送信するか（0で無効）
            preview_size (list[int]): 表示用画像のサイズ。カメラからはこのサイズで取り込み、モデル用画像はそこから切り出して縮小する
            preview_jpeg_quality (int): 表示用画像の JPEG 品質
            preview_white_balance (list[float]): 表示用画像だけにかける色の補正 [赤, 緑, 青] の倍率（モデル用画像には使わない）
            verbose (int): ログ出力レベル

        """
        super().__init__('csi_camera_node')

        # --- パラメータ定義 ---
        self.declare_parameter('output_topic', '/camera/image_raw')
        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('capture_size', [1280, 720])
        self.declare_parameter('capture_fps', 60)
        self.declare_parameter('resize', [84, 84])
        self.declare_parameter('crop', [0])
        self.declare_parameter('flip_method', 0)
        self.declare_parameter('nvargs', '')
        self.declare_parameter('frame_id', 'frame')
        self.declare_parameter('pub_hz', 10.0)
        self.declare_parameter('stall_timeout', 3.0)
        self.declare_parameter('max_reconnect_interval', 5.0)
        self.declare_parameter('restart_after_failures', 5)
        self.declare_parameter('preview_topic', '/camera/preview/compressed')
        self.declare_parameter('preview_hz', 15.0)
        self.declare_parameter('preview_size', [640, 480])
        self.declare_parameter('preview_jpeg_quality', 80)
        self.declare_parameter('preview_white_balance', [1.0, 1.0, 1.0])
        self.declare_parameter('verbose', 1)

        # --- パラメータ呼び出し ---
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.sensor_id = self.get_parameter('sensor_id').get_parameter_value().integer_value
        self.capture_size = list(self.get_parameter('capture_size').get_parameter_value().integer_array_value)
        self.capture_fps = self.get_parameter('capture_fps').get_parameter_value().integer_value
        self.resize = list(self.get_parameter('resize').get_parameter_value().integer_array_value)
        crop = list(self.get_parameter('crop').get_parameter_value().integer_array_value)
        self.flip_method = self.get_parameter('flip_method').get_parameter_value().integer_value
        self.nvargs = self.get_parameter('nvargs').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        pub_hz = self.get_parameter('pub_hz').get_parameter_value().double_value
        self.stall_timeout = self.get_parameter('stall_timeout').get_parameter_value().double_value
        self.max_reconnect_interval = self.get_parameter('max_reconnect_interval').get_parameter_value().double_value
        self.restart_after_failures = self.get_parameter('restart_after_failures').get_parameter_value().integer_value
        preview_topic = self.get_parameter('preview_topic').get_parameter_value().string_value
        preview_hz = self.get_parameter('preview_hz').get_parameter_value().double_value
        preview_size = list(self.get_parameter('preview_size').get_parameter_value().integer_array_value)
        self.preview_jpeg_quality = self.get_parameter('preview_jpeg_quality').get_parameter_value().integer_value
        wb = list(self.get_parameter('preview_white_balance').get_parameter_value().double_array_value)
        self.preview_lut = self._white_balance_lut(wb) if len(wb) == 3 and wb != [1.0, 1.0, 1.0] else None
        self.verbose = self.get_parameter('verbose').get_parameter_value().integer_value

        # --- パブリッシャー ---
        self.pub = self.create_publisher(Image, output_topic, qos_profile_sensor_data)
        # Dashboard-only preview (higher resolution and rate, JPEG); inference and data collection use output_topic
        # at `resize` / pub_hz. The camera is captured at preview_size (keep the 16:9 aspect of capture_size); the
        # model image is the `crop` region (sky removed) of that frame, downscaled to `resize`.
        self.preview_pub = self.create_publisher(CompressedImage, preview_topic, qos_profile_sensor_data) if preview_hz > 0 else None
        self.capture_output = preview_size if len(preview_size) == 2 else self.capture_size
        sx = self.capture_output[0] / self.capture_size[0]
        sy = self.capture_output[1] / self.capture_size[1]
        if len(crop) == 4:
            x, y, w, h = crop
            self.crop = (round(x * sx), round(y * sy), round(w * sx), round(h * sy))
        else:
            self.crop = None
        self.latest_preview = None
        self.health_pub = self.create_publisher(String, '/system/health', 10)
        self.notice_pub = self.create_publisher(String, '/system/notice', 10)

        # --- 取得スレッドとの共有状態 ---
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_seq = 0
        self.published_seq = 0
        self.preview_seq = 0
        self.last_frame_time = 0.0
        self.status = "RECONNECTING"
        self.status_detail = "起動中"
        self.connecting_since = time.monotonic()
        self.ever_connected = False
        self.long_reconnect_notified = False
        self.fps = 0.0
        self.running = True

        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

        self.timer = self.create_timer(1/pub_hz, self._pub)
        if self.preview_pub is not None:
            self.preview_timer = self.create_timer(1/preview_hz, self._pub_preview)
        self.watchdog_timer = self.create_timer(0.5, self._watchdog)
        self.health_timer = self.create_timer(1.0, self._publish_health)
        self._publish_health()
        crop_text = f", 切り出し: {list(crop)}" if self.crop else ""
        self.get_logger().info(f'【カメラ】CSIカメラへの接続を開始します。 (解像度: {tuple(self.resize)}{crop_text}, 送信頻度: {pub_hz}Hz, トピック名: {output_topic}'
                               + (f', 表示用: {tuple(self.capture_output)} {preview_hz}Hz {preview_topic})' if self.preview_pub else ')'))

    # --- 通知 ---
    def _notice(self, level, text):
        msg = String()
        msg.data = f"{level}|camera|{text}"
        self.notice_pub.publish(msg)

    def _set_status(self, status, detail):
        with self.lock:
            self.status = status
            self.status_detail = detail
        self._publish_health()

    # --- 取得スレッド ---
    def _restart_if_stuck(self, failures):
        # The Argus client inside this process can be left unusable (e.g. after nvargus-daemon restarts),
        # so after repeated failures start over with a fresh process (launch respawns this node).
        if self.restart_after_failures <= 0 or failures < self.restart_after_failures:
            return
        text = f"カメラに{failures}回続けて接続できませんでした。カメラノードを再起動して接続し直します"
        self.get_logger().error(f'【再起動】{text}')
        self._notice("ERROR", text)
        self._set_status("RESTARTING", text)
        time.sleep(0.3)  # let the notice and health messages go out
        os._exit(1)

    def _capture_loop(self):
        interval = 1.0
        failures = 0
        cap = None
        fps_count, fps_start = 0, time.monotonic()
        while self.running:
            if cap is None:
                try:
                    cap = CSICamera(self.sensor_id, self.capture_size, self.capture_fps, self.capture_output, self.flip_method, self.nvargs)
                    fps_count, fps_start = 0, time.monotonic()
                except Exception as e:
                    failures += 1
                    self._set_status("RECONNECTING", f"カメラを開けません: {e}")
                    if self.verbose >= 1:
                        self.get_logger().error(f'【接続失敗】CSIカメラを開けません。物理的な接続を確認してください。{interval:.0f}秒後に再試行します。(試行{failures}回目: {e})')
                    self._restart_if_stuck(failures)
                    time.sleep(interval)
                    interval = min(interval * 2, self.max_reconnect_interval)
                    continue

            try:
                frame = cap.read()
            except Exception as e:
                cap.release()
                cap = None
                failures += 1
                with self.lock:
                    was_ok = self.status == "OK"
                    if was_ok:
                        self.connecting_since = time.monotonic()
                self._set_status("RECONNECTING", f"画像の取得に失敗: {e}")
                if was_ok:
                    self.get_logger().error(f'【接続断】CSIカメラから画像を取得できませんでした。再接続を試みます。({e})')
                    self._notice("ERROR", "カメラとの接続が切れました。再接続を試みています（AUTO走行は停止します）")
                elif self.verbose >= 1:
                    self.get_logger().error(f'【再接続失敗】CSIカメラから画像を取得できませんでした。{interval:.0f}秒後に再試行します。(試行{failures}回目: {e})')
                self._restart_if_stuck(failures)
                time.sleep(interval)
                interval = min(interval * 2, self.max_reconnect_interval)
                continue

            failures = 0

            now = time.monotonic()
            fps_count += 1
            if now - fps_start >= 1.0:
                self.fps = fps_count / (now - fps_start)
                fps_count, fps_start = 0, now

            with self.lock:
                self.latest_preview = frame
                self.latest_frame = self._model_image(frame)
                self.latest_seq += 1
                self.last_frame_time = now
                became_ok = self.status != "OK"
                if became_ok:
                    self.status = "OK"
                    self.status_detail = ""
            if became_ok:
                interval = 1.0
                self.long_reconnect_notified = False
                if self.ever_connected:
                    self.get_logger().info('【再接続成功】CSIカメラに再接続しました。')
                    self._notice("INFO", "カメラに再接続しました")
                else:
                    self.get_logger().info('【接続成功】CSIカメラを起動しました。')
                    self._notice("INFO", "カメラに接続しました")
                self.ever_connected = True
                self._publish_health()

        if cap is not None:
            cap.release()

    def _model_image(self, frame):
        """Model input: the crop region (sky removed) downscaled to `resize`."""
        if self.crop is not None:
            x, y, w, h = self.crop
            frame = frame[y:y + h, x:x + w]
        if frame.shape[1] == self.resize[0] and frame.shape[0] == self.resize[1]:
            return np.ascontiguousarray(frame)
        return cv2.resize(frame, tuple(self.resize), interpolation=cv2.INTER_AREA)

    # --- 監視 ---
    def _watchdog(self):
        with self.lock:
            status = self.status
            last_frame_time = self.last_frame_time
            connecting_since = self.connecting_since
        now = time.monotonic()

        if status == "OK" and now - last_frame_time > self.stall_timeout:
            # cap.read() is blocked inside GStreamer; it cannot be interrupted safely from another thread.
            # Exit so that launch (respawn=True) restarts this node with a fresh pipeline.
            text = f"カメラ画像が{self.stall_timeout:.0f}秒以上届いていません。カメラノードを再起動します"
            self.get_logger().error(f'【取得停止】{text}')
            self._notice("ERROR", text)
            self._set_status("RESTARTING", text)
            time.sleep(0.3)  # let the notice and health messages go out
            os._exit(1)

        if status == "RECONNECTING" and now - connecting_since > 30.0 and not self.long_reconnect_notified:
            self.long_reconnect_notified = True
            text = "30秒以上再接続できません。ケーブル接続を確認し、改善しなければホストで sudo systemctl restart nvargus-daemon を実行してください"
            self.get_logger().error(f'【再接続失敗】{text}')
            self._notice("ERROR", text)

    # --- 配信 ---
    def _convert_numpy_to_msg(self, frame: np.ndarray) -> Image:
        """

        カメラ画像のNumPy配列をImageメッセージに変換する。

        Args:
            frame (ndarray): 画像

        Returns:
            msg (Image): 画像のメッセージ

        """
        # メッセージ作成
        msg = Image()

        # データ挿入
        h, w, c = frame.shape
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = h
        msg.width = w
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = int(frame.strides[0])
        msg.data = frame.tobytes()

        return msg

    def _pub(self):
        """

        カメラから取得した最新の画像を送信する（同じ画像は二度送らない）。

        """
        with self.lock:
            if self.latest_frame is None or self.latest_seq == self.published_seq:
                return
            frame = self.latest_frame
            self.published_seq = self.latest_seq

        msg = self._convert_numpy_to_msg(frame)
        self.pub.publish(msg)

        # ログ出力
        if self.verbose >= 2:
            self.get_logger().info(f'Frame sent.')

    def _pub_preview(self):
        """

        ダッシュボード表示用に最新の画像を JPEG で送信する（推論・記録には使わない）。

        """
        with self.lock:
            if self.latest_preview is None or self.latest_seq == self.preview_seq:
                return
            frame = self.latest_preview
            self.preview_seq = self.latest_seq

        if self.preview_lut is not None:
            frame = cv2.LUT(frame, self.preview_lut)     # display only: the model image keeps the camera's colours
        ok, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.preview_jpeg_quality])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = jpeg.tobytes()
        self.preview_pub.publish(msg)

    @staticmethod
    def _white_balance_lut(gains_rgb, knee=235):
        """Per-channel lookup table (BGR order) for the preview colour correction. Values up to `knee` are
        scaled by the gain; above it they blend linearly to 255, so blown-out highlights stay white instead of
        taking a colour cast."""
        v = np.arange(256, dtype=np.float64)
        chans = []
        for g in (gains_rgb[2], gains_rgb[1], gains_rgb[0]):          # B, G, R
            low = v * g
            top = min(knee * g, 255.0)
            high = top + (v - knee) * (255.0 - top) / (255.0 - knee)
            chans.append(np.clip(np.where(v <= knee, low, high), 0, 255))
        return np.stack(chans, axis=1).reshape(1, 256, 3).astype(np.uint8)

    def _publish_health(self):
        with self.lock:
            status, detail = self.status, self.status_detail
        msg = String()
        if status == "OK":
            msg.data = f"camera:OK:{self.fps:.1f}fps"
        else:
            msg.data = f"camera:{status}:{detail}"
        self.health_pub.publish(msg)

    def stop(self):
        self.running = False
        self.capture_thread.join(timeout=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = CSICameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
