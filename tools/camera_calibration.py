"""カメラのキャリブレーション（予測した走行経路をカメラ画像に重ねる表示用）

config.yaml の camera_geometry（仮の値）を、実測から求めた値に置き換えるためのツールです。
求めた値は YAML として表示されるので、config.yaml に貼り付け、camera_geometry.unverified から名前を消してください。

  1. capture     : ダッシュボード用の画像（/camera/preview/compressed）を保存（コンテナ内、走行システムの起動中）
  2. intrinsics  : チェッカーボードを写した画像から、レンズの焦点距離と歪み（K, D）を求める
  3. extrinsics  : 地面の印の位置（後軸中央から測った [前, 左] m）と画像上の位置から、カメラの位置と向きを求める
  4. check       : 求めた値で地面の格子（1 m ごと）を画像に重ねて、合っているかを確かめる

使い方:
    python3 tools/camera_calibration.py capture --out data/calib/board --count 25 --interval 2
    python3 tools/camera_calibration.py intrinsics "data/calib/board_*.jpg" --pattern 9x6 --square 0.025
    python3 tools/camera_calibration.py extrinsics data/calib/markers.json
    python3 tools/camera_calibration.py check data/calib/markers.jpg --png data/calib/check.png

markers.json の例（pixel は保存した画像上の位置。画像ビューアーでマウスを合わせると分かります）:
    {"image": "data/calib/markers.jpg",
     "points": [{"ground": [2.0, 0.0], "pixel": [320, 300]}, {"ground": [3.0, 1.0], "pixel": [180, 255]}, ...]}
印は 6 個以上、前後・左右に散らばるように置いてください（例: 後軸中央から 2・3・5 m 先の中心線上と左右 1 m）。
"""
import argparse
import glob
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'pius_control', 'pius_control'))
from calibration import load_config  # noqa: E402
from camera_geometry import BODY_TO_OPTICAL, IMAGE_SIZE, project_ground  # noqa: E402

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.yaml')


def geometry():
    return dict(load_config(CONFIG).get('camera_geometry', {}))


def yaml_list(m):
    return json.dumps(np.round(np.asarray(m, dtype=float), 4).tolist())


def cmd_capture(args):
    import rclpy
    from sensor_msgs.msg import CompressedImage
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('calib_capture')
    latest = {}
    node.create_subscription(CompressedImage, '/camera/preview/compressed', lambda m: latest.update(m=m), 10)
    saved, next_t = 0, time.time() + 1.0
    print(f"{args.interval} 秒ごとに {args.count} 枚保存します。チェッカーボードの位置・向き・距離を毎回変えてください。")
    while saved < args.count:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() >= next_t and 'm' in latest:
            path = f"{args.out}_{saved:02d}.jpg"
            with open(path, 'wb') as f:
                f.write(bytes(latest['m'].data))
            saved += 1
            next_t = time.time() + args.interval
            print(f"  保存 {path}", flush=True)
    rclpy.shutdown()


def cmd_intrinsics(args):
    import cv2
    cols, rows = (int(v) for v in args.pattern.lower().split('x'))
    obj = np.zeros((1, cols * rows, 3), np.float64)
    obj[0, :, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * args.square
    objpoints, imgpoints = [], []
    for path in sorted(glob.glob(args.images)):
        img = cv2.imread(path)
        if img is None:
            continue
        scale = IMAGE_SIZE[0] / img.shape[1]          # work in pixels of the 1280x720 capture
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCorners(gray, (cols, rows), cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not ok:
            print(f"  {os.path.basename(path)}: 格子が見つかりません（除外）")
            continue
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
        objpoints.append(obj)
        imgpoints.append((corners * scale).reshape(1, -1, 2).astype(np.float64))
    if len(objpoints) < 8:
        sys.exit(f"格子が見つかった画像が {len(objpoints)} 枚しかありません（8 枚以上必要。20 枚以上を推奨）")
    K, D = np.zeros((3, 3)), np.zeros((4, 1))
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW
    rms, K, D, _, _ = cv2.fisheye.calibrate(objpoints, imgpoints, IMAGE_SIZE, K, D, flags=flags,
                                            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
    print(f"使った画像 {len(objpoints)} 枚、再投影誤差 {rms:.2f} 画素（1 前後なら良好）")
    print("\nconfig.yaml の camera_geometry に貼り付け、unverified から K と D を消してください:")
    print(f"  K: {yaml_list(K)}\n  D: {yaml_list(D.ravel())}")


def solve_pose(ground, pixels, K, D):
    """Ground points [N, 2] (rear-axle frame) and pixels [N, 2] (1280x720) -> (position, rpy_deg, rms px)."""
    import cv2
    obj = np.array([[x, y, 0.0] for x, y in ground], dtype=np.float64)
    pix = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    und = cv2.fisheye.undistortPoints(pix, np.asarray(K, float), np.asarray(D, float).reshape(4, 1))
    ok, rvec, tvec = cv2.solvePnP(obj, und, np.eye(3), None, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise RuntimeError("カメラの位置を求められませんでした")
    R, _ = cv2.Rodrigues(rvec)                 # optical = R p + t
    R_vc = R.T @ BODY_TO_OPTICAL               # camera body axes in the vehicle frame
    position = (-R.T @ tvec).ravel()
    pitch = math.degrees(math.asin(-R_vc[2, 0]))
    roll = math.degrees(math.atan2(R_vc[2, 1], R_vc[2, 2]))
    yaw = math.degrees(math.atan2(R_vc[1, 0], R_vc[0, 0]))
    geom = {"K": K, "D": D, "position_m": position.tolist(), "rpy_deg": [roll, pitch, yaw]}
    proj = np.array(project_ground([tuple(g) for g in ground], geom, normalized=False), dtype=float)
    rms = float(np.sqrt(np.mean(np.sum((proj - np.asarray(pixels)) ** 2, axis=1))))
    return position, (roll, pitch, yaw), rms


def cmd_extrinsics(args):
    import cv2
    with open(args.markers) as f:
        spec = json.load(f)
    pts = spec["points"]
    if len(pts) < 4:
        sys.exit("印は 4 個以上必要です（6 個以上を推奨）")
    width = spec.get("image_width")
    if width is None and spec.get("image"):
        img = cv2.imread(spec["image"])
        width = img.shape[1] if img is not None else IMAGE_SIZE[0]
    scale = IMAGE_SIZE[0] / float(width or IMAGE_SIZE[0])
    g = geometry()
    pos, rpy, rms = solve_pose([p["ground"] for p in pts], [[p["pixel"][0] * scale, p["pixel"][1] * scale] for p in pts],
                               g["K"], g.get("D", [0, 0, 0, 0]))
    print(f"印 {len(pts)} 個、再投影誤差 {rms:.1f} 画素（1280×720 換算。数画素以内なら良好）")
    print("\nconfig.yaml の camera_geometry に貼り付け、unverified から position_m と rpy_deg を消してください:")
    print(f"  position_m: {yaml_list(pos)}\n  rpy_deg: {yaml_list(rpy)}")
    if "camera_geometry.K" in [f"camera_geometry.{k}" for k in g.get("unverified", [])]:
        print("※ K・D がまだ仮の値です。先に intrinsics を行うと、位置と向きも正確になります。")


def cmd_check(args):
    import cv2
    img = cv2.imread(args.image)
    if img is None:
        sys.exit(f"画像を読めません: {args.image}")
    g = geometry()
    sx, sy = img.shape[1], img.shape[0]
    def draw(points, color):
        uv = project_ground(points, g)
        prev = None
        for q in uv:
            if q is not None and prev is not None:
                cv2.line(img, (int(prev[0] * sx), int(prev[1] * sy)), (int(q[0] * sx), int(q[1] * sy)), color, 1, cv2.LINE_AA)
            prev = q
    for x in np.arange(1.0, 9.0, 1.0):
        draw([(x, y) for y in np.linspace(-2, 2, 41)], (0, 255, 255))
    for y in np.arange(-2.0, 2.5, 0.5):
        draw([(x, y) for x in np.linspace(0.5, 8, 76)], (0, 200, 0) if y else (0, 0, 255))
    out = args.png or os.path.splitext(args.image)[0] + "_check.png"
    cv2.imwrite(out, img)
    print(f"地面の格子（黄: 前方 1 m ごと、緑: 左右 0.5 m ごと、赤: 中心線）を重ねた画像: {out}")


def main():
    ap = argparse.ArgumentParser(description="カメラのキャリブレーション（走行経路の重ね表示用）")
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture"); c.add_argument("--out", default="data/calib/board"); c.add_argument("--count", type=int, default=25)
    c.add_argument("--interval", type=float, default=2.0)
    i = sub.add_parser("intrinsics"); i.add_argument("images"); i.add_argument("--pattern", default="9x6", help="内側の角の数 列x行")
    i.add_argument("--square", type=float, default=0.025, help="マス 1 辺の長さ [m]")
    e = sub.add_parser("extrinsics"); e.add_argument("markers")
    k = sub.add_parser("check"); k.add_argument("image"); k.add_argument("--png", default=None)
    args = ap.parse_args()
    {"capture": cmd_capture, "intrinsics": cmd_intrinsics, "extrinsics": cmd_extrinsics, "check": cmd_check}[args.cmd](args)


if __name__ == "__main__":
    main()
