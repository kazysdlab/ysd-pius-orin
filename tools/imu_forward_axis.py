"""IMU の前方向（config.yaml の wheel.imu_forward_axis）を走行ログから推定する

まっすぐ走っているときの IMU の水平加速度は、ほぼ前後方向にだけ変化します。そこで
1. 向き: 走っている間の水平加速度 (ax, ay) が最もよく変化する方向（主成分）
2. 符号: 車輪のパルスから求めた加速度（1 回転分ずつの速度の差。磁石の間隔の不ぞろいを打ち消す）と
   同じ向きになる側
として求めます（以前の「パルスの加速度に合う向きの最小二乗」は、前後に直角な成分が決まらず不安定でした）。

使うログ: IMU を固定した後に、**まっすぐ・前進だけ**で、加速と減速をはっきり何度か入れて走ったもの
（測定ガイド手順 5 の A など）。カーブや後退が混ざると不正確になります。

使い方（ホストで実行。複数のログをまとめて使えます。ログごとの結果も表示します）:
    python3 tools/imu_forward_axis.py data/speed_logs/speed_YYYYmmdd_HHMMSS_A_straight1.jsonl ...
    python3 tools/imu_forward_axis.py data/pius_drive_YYYYmmdd_HHMMSS.h5
表示された imu_forward_axis を config.yaml の wheel に貼り付け、wheel.unverified から imu_forward_axis を消してください。
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'pius_control', 'pius_control'))
from calibration import load_config  # noqa: E402
from waypoint_labels import load_h5, load_speed_log, reconstruct  # noqa: E402


def moving_data(data, circumference, max_interval, guess_axis):
    """IMU horizontal acceleration while the car moves, and (pulse acceleration, mean ax, mean ay) per window.

    Only the pulses the speed estimator accepts are used (electrical noise makes spurious pulses).
    """
    ppr = max(1, int(data.get("pulses_per_rev") or 1))
    traj = reconstruct(data["imu_t"], data["acc"], data["gyro"], data["pulse_t"], circumference,
                       forward_axis=guess_axis, pulses_per_rev=ppr)
    p = np.sort(np.asarray(traj["accepted_pulses"], dtype=float))
    imu_t, acc = np.asarray(data["imu_t"], dtype=float), np.asarray(data["acc"], dtype=float)
    moving = np.zeros(len(imu_t), dtype=bool)
    for a, b in zip(p[:-1], p[1:]):
        if b - a <= max_interval:
            moving |= (imu_t >= a) & (imu_t < b)
    windows = []
    for k in range(len(p) - 2 * ppr):
        a, b, c = p[k], p[k + ppr], p[k + 2 * ppr]
        if np.max(np.diff(p[k:k + 2 * ppr + 1])) > max_interval:
            continue                      # standing still in between: the speed is unknown there
        v1, v2 = circumference / (b - a), circumference / (c - b)
        t1, t2 = (a + b) / 2, (b + c) / 2
        m = (imu_t >= t1) & (imu_t < t2)
        if m.sum() < 2:
            continue
        a_p = (v2 - v1) / (t2 - t1)
        if abs(a_p) > 3.0:
            continue                      # beyond what this car can do: a missed or spurious pulse
        windows.append((a_p, acc[m, 0].mean(), acc[m, 1].mean()))
    return acc[moving, :2], np.array(windows).reshape(-1, 3)


def estimate(horiz, windows):
    """(axis, share of the variance along it, correlation with the pulse acceleration, n windows) or None."""
    if len(horiz) < 50 or len(windows) < 5:
        return None
    cov = np.cov((horiz - horiz.mean(axis=0)).T)
    vals, vecs = np.linalg.eigh(cov)
    axis = vecs[:, np.argmax(vals)]
    share = float(vals.max() / max(vals.sum(), 1e-12))
    proj = windows[:, 1:] @ axis
    r = float(np.corrcoef(proj, windows[:, 0])[0, 1]) if np.std(proj) > 0 and np.std(windows[:, 0]) > 0 else 0.0
    if r < 0:
        axis, r = -axis, -r
    return (float(axis[0]), float(axis[1])), share, r, len(windows)


def angle(axis):
    return math.degrees(math.atan2(axis[1], axis[0]))


def main():
    ap = argparse.ArgumentParser(description="IMU の前方向を走行ログから推定する")
    ap.add_argument("logs", nargs="+", help="speed_live のログ（.jsonl）または記録ファイル（.h5）")
    ap.add_argument("--circumference", type=float, default=None, help="タイヤ外周 [m]（既定: ログの値）")
    ap.add_argument("--max-interval", type=float, default=1.5, help="これより長いパルス間隔は使わない [s]（停止を含むため）")
    args = ap.parse_args()

    cfg = load_config(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.yaml'))
    guess = tuple((cfg.get("wheel") or {}).get("imu_forward_axis", (-0.338, 0.941)))
    all_h, all_w = [], []
    for path in args.logs:
        data = load_h5(path) if path.endswith(".h5") else load_speed_log(path)
        C = args.circumference or data["circumference"]
        if not C:
            sys.exit(f"{path}: タイヤ外周が分かりません（--circumference で指定してください）")
        h, w = moving_data(data, C, args.max_interval, guess)
        e = estimate(h, w)
        text = "データが少なすぎます" if e is None else \
            f"前方向 {angle(e[0]):+6.1f}°・主成分の割合 {e[1]:.2f}・パルスとの相関 {e[2]:.2f}"
        print(f"{os.path.basename(path)}: 走行中の IMU {len(h)} 点・区間 {len(w)} … {text}")
        all_h.append(h)
        all_w.append(w)
    e = estimate(np.concatenate(all_h), np.concatenate(all_w))
    if e is None:
        sys.exit("データが少なすぎます。まっすぐ・前進で、加速と減速を何度か入れて走ったログを使ってください。")
    axis, share, r, n = e
    print(f"\nまとめ: 前方向 ({axis[0]:+.3f}, {axis[1]:+.3f})（IMU の x 軸から {angle(axis):+.1f}°）")
    print(f"主成分の割合 {share:.2f}（前後方向の変化が全体に占める割合。0.8 以上が目安）、パルスの加速度との相関 {r:.2f}（0.5 以上が目安）、区間 {n}")
    if share < 0.8 or r < 0.5:
        print("⚠ 確からしさが足りないので、この値は使わないでください（カーブ・後退が混ざっている、加減速が弱い、IMU が途中で動いた、など）。")
    else:
        print("\nconfig.yaml の wheel に貼り付け、wheel.unverified から imu_forward_axis を消してください:")
        print(f"  imu_forward_axis: [{axis[0]:.3f}, {axis[1]:.3f}]")


if __name__ == "__main__":
    main()
