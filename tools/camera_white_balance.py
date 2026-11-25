"""ダッシュボード表示用の色の補正（config.yaml の camera.preview_white_balance）を、白い紙から測る

色補正ファイル（Jetson Nano 用）のため、カメラの画像は全体が紫寄りになります。学習に使うモデル用の画像は
そのまま使い、人が見るダッシュボードの映像だけをこの倍率で補正します。

使い方（走行システムの起動中に、コンテナ内で実行）:
  1. 白い紙（コピー用紙で可）を、モデルに渡す範囲（ダッシュボードの白い破線の枠）の中央に大きく写す
  2. python3 tools/camera_white_balance.py
  3. 表示された preview_white_balance を config.yaml の camera に貼り付ける（反映にはシステムの再起動が必要）
屋外でも測って値が近ければ、camera.unverified から preview_white_balance を消してください。
"""
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    rclpy.init()
    node = rclpy.create_node('camera_white_balance')
    frames = []

    def on_image(m):
        frames.append(np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).astype(float))

    node.create_subscription(Image, '/camera/image_raw', on_image, qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    rclpy.shutdown()
    if not frames:
        raise SystemExit("/camera/image_raw が届きません（走行システムを起動してから実行してください）")
    img = np.mean(frames[-10:], axis=0)
    h, w, _ = img.shape
    centre = img[h // 4: 3 * h // 4, w // 4: 3 * w // 4]           # BGR
    b, g, r = (float(centre[..., i].mean()) for i in range(3))
    if max(r, g, b) > 245 or min(r, g, b) < 40:
        print("⚠ 白い紙が明るすぎる（白飛び）か暗すぎます。紙の向きや位置を変えて測り直してください。")
    spread = float(np.std(centre, axis=(0, 1)).mean())
    if spread > 25:
        print("⚠ 画像の中央が一様ではありません。白い紙を中央に大きく写してください。")
    gains = [g / r, 1.0, g / b]
    print(f"中央の平均（補正なし）: 赤 {r:.0f} / 緑 {g:.0f} / 青 {b:.0f}（R/G {r / g:.2f}、B/G {b / g:.2f}）")
    print("config.yaml の camera に貼り付けてください（表示用のみ。モデル用画像には影響しません）:")
    print(f"  preview_white_balance: [{gains[0]:.3f}, 1.0, {gains[2]:.3f}]")


if __name__ == "__main__":
    main()
