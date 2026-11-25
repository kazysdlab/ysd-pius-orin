"""走行ログから後軸中心の走行軌跡を復元して確認する（タイヤ外周・片輪補正・ヨーレートの符号の確認用）

waypoint の正解データと同じ処理（src/pius_control/pius_control/waypoint_labels.py）で、車輪パルスと IMU から
後軸中心の軌跡を復元し、走行距離・向きの変化・始点と終点のずれを表示して、上から見た軌跡を PNG に保存します。

確認用の走行の例:
  - 巻き尺で測った直線: 表示される距離と実測を比べる（外周の確認。--circumference で値を変えて再計算できる）
  - 同じ大きさの円を左回り・右回り: 区間ごとの距離が左右で揃えば片輪補正と符号が正しい（--from / --to で区間指定）
  - コースを一周して同じ位置・向きで止まる: 始点と終点のずれが復元誤差

使い方（ホストで実行。h5py がない場合はコンテナで実行）:
    python3 tools/trajectory_check.py data/speed_logs/speed_YYYYmmdd_HHMMSS.jsonl
    python3 tools/trajectory_check.py data/pius_drive_YYYYmmdd_HHMMSS.h5 --circumference 1.21
    python3 tools/trajectory_check.py <ログ> --from 30 --to 75        # 記録開始からの秒数で区間を指定
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'pius_control', 'pius_control'))
from calibration import load_config  # noqa: E402
from waypoint_labels import REAR_TRACK_M, imu_mounting, load_h5, load_speed_log, reconstruct, summary  # noqa: E402

CONFIG = load_config(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.yaml'))
VEHICLE = CONFIG.get('vehicle', {})


def main():
    ap = argparse.ArgumentParser(description="走行ログから後軸中心の軌跡を復元して確認する")
    ap.add_argument("log", help="speed_live のログ（.jsonl）または記録ファイル（.h5）")
    ap.add_argument("--circumference", type=float, default=None, help="タイヤ外周 [m]（既定: ログの値）")
    ap.add_argument("--pulses-per-rev", type=int, default=None, help="1 回転あたりのパルス数（既定: ログの値）")
    ap.add_argument("--track", type=float, default=float(VEHICLE.get("rear_track_m", REAR_TRACK_M)),
                    help="後輪の左右の間隔 [m]（既定: config.yaml の vehicle.rear_track_m）")
    ap.add_argument("--yaw-sign", type=float, default=None,
                    help="ヨーレートの符号（左回りが正なら 1。既定: 記録時に実測済みならその値、それ以外は vehicle.yaw_sign）")
    ap.add_argument("--forward-axis", type=float, nargs=2, default=None, metavar=("X", "Y"),
                    help="IMU の前方向（既定: 記録時に実測済みならその値、それ以外は wheel.imu_forward_axis）")
    ap.add_argument("--from", dest="t_from", type=float, default=None, help="区間の開始 [記録開始からの秒]")
    ap.add_argument("--to", dest="t_to", type=float, default=None, help="区間の終了 [記録開始からの秒]")
    ap.add_argument("--png", default=None, help="軌跡の図の保存先（既定: ログと同じ名前の _trajectory.png）")
    args = ap.parse_args()

    data = load_h5(args.log) if args.log.endswith(".h5") else load_speed_log(args.log)
    C = args.circumference or data["circumference"] or 1.277
    if len(data["imu_t"]) == 0:
        sys.exit("IMU のデータがありません（IMU が接続されていない状態の記録です）")
    t0 = data["imu_t"][0]
    ppr = args.pulses_per_rev or data["pulses_per_rev"]
    axis, sign, src = imu_mounting(data, CONFIG, args.forward_axis, args.yaw_sign)
    traj = reconstruct(data["imu_t"], data["acc"], data["gyro"], data["pulse_t"], C,
                       track=args.track, forward_axis=axis, yaw_sign=sign, pulses_per_rev=ppr)

    if args.t_from is not None or args.t_to is not None:
        m = np.ones(len(traj["t"]), dtype=bool)
        if args.t_from is not None:
            m &= traj["t"] - t0 >= args.t_from
        if args.t_to is not None:
            m &= traj["t"] - t0 <= args.t_to
        part = {k: (v[m] if isinstance(v, np.ndarray) and len(v) == len(m) else v) for k, v in traj.items()}
    else:
        part = traj
    s = summary(part)
    if s is None:
        sys.exit("軌跡を復元できる区間がありません（有効な車輪パルスが 2 回以上必要です）")
    print(f"外周 {C} m（1 回転 {ppr} パルス）、後輪の間隔 {args.track} m、ヨーレートの符号 {sign:+.0f}（{src['yaw_sign']}）、"
          f"IMU の前方向 ({axis[0]:+.3f}, {axis[1]:+.3f})（{src['axis']}）")
    print(f"パルス {s['pulses']} 回（採用 {s['accepted']} 回）")
    print(f"区間 {s['duration']:.1f} 秒: 走行距離 {s['distance']:.2f} m（後軸中心）、向きの変化 {s['heading_change_deg']:+.1f}°（左回りが +）")
    print(f"始点と終点のずれ {s['start_end_gap']:.2f} m")

    png = args.png or os.path.splitext(args.log)[0] + "_trajectory.png"
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    except ImportError:
        print("matplotlib がないため図は省略します")
        return
    v = part["valid"]
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(part["x"], part["y"], color="#cbd5e1", lw=1, label="復元できない区間を含む")
    xs, ys = np.where(v, part["x"], np.nan), np.where(v, part["y"], np.nan)
    ax.plot(xs, ys, color="#0284c7", lw=2, label="後軸中心の軌跡")
    i0, i1 = np.argmax(v), len(v) - 1 - np.argmax(v[::-1])
    ax.plot(part["x"][i0], part["y"][i0], "o", color="#16a34a", ms=9, label="始点")
    ax.plot(part["x"][i1], part["y"][i1], "s", color="#dc2626", ms=9, label="終点")
    ax.set_aspect("equal")
    ax.grid(alpha=.3)
    ax.set_xlabel("x [m]（記録開始時の前方）")
    ax.set_ylabel("y [m]（記録開始時の左）")
    ax.set_title(f"走行距離 {s['distance']:.1f} m・向きの変化 {s['heading_change_deg']:+.0f}°・始点と終点のずれ {s['start_end_gap']:.2f} m",
                 fontsize=10)
    ax.legend(fontsize=9)
    fig.tight_layout()
    fig.savefig(png, dpi=100)
    print(f"図: {png}")


if __name__ == "__main__":
    main()
