"""ステア指令と前輪の操舵角の対応表（config.yaml の vehicle.steer_table）を走行ログから作る

ステアを一定に保って円を走ると、曲率 = ヨーレート ÷ 車速 で、前輪の操舵角 = atan(軸距 × 曲率) が分かります。
tools/speed_live.sh の記録（ステア指令・IMU・車速が入っている）から、ステア指令ごとの操舵角を求めて
対応表を表示します。表示された steer_table を config.yaml に貼り付け、vehicle.unverified から消してください。

走り方の例（MANUAL、ゆっくり 2〜4 km/h）:
  ステアを左いっぱい・左半分・右半分・右いっぱいなど、いくつかの位置で一定に保って、それぞれ円を 1 周以上走る。
  コントローラーのスティックを一定に保つのが難しければ、少しずつ違う位置で何回か走れば、まとめて集計されます。
  先に vehicle.yaw_sign を確認しておいてください（左回りの円で向きの変化が + になること。マニュアル 9.3）。

使い方（ホストで実行）:
    python3 tools/steering_calibration.py data/speed_logs/speed_20261010_*.jsonl
"""
import argparse
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'pius_control', 'pius_control'))
from calibration import load_config  # noqa: E402

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.yaml')


def load(path):
    imu_t, gz, st_t, speed, cmd_t, steer = [], [], [], [], [], []
    with open(path) as f:
        for line in f:
            r = json.loads(line)
            if r["k"] == "imu":
                imu_t.append(r["t"]); gz.append(r["gyro"][2])
            elif r["k"] == "status" and r.get("speed") is not None:
                st_t.append(r["t"]); speed.append(r["speed"])
            elif r["k"] == "cmd" and "0" in r["ch"]:
                cmd_t.append(r["t"]); steer.append(r["ch"]["0"])
    return [np.array(a) for a in (imu_t, gz, st_t, speed, cmd_t, steer)]


def main():
    ap = argparse.ArgumentParser(description="ステア指令と操舵角の対応表を走行ログから作る")
    ap.add_argument("logs", nargs="+", help="speed_live のログ（.jsonl、複数・ワイルドカード可）")
    ap.add_argument("--min-speed", type=float, default=0.3, help="この速度 [m/s] 以上の区間だけ使う")
    ap.add_argument("--bin", type=float, default=0.1, help="ステア指令の刻み")
    ap.add_argument("--min-samples", type=int, default=10)
    args = ap.parse_args()
    cfg = load_config(CONFIG)
    veh = cfg.get("vehicle", {})
    L, yaw_sign = float(veh.get("wheelbase_m", 1.5)), float(veh.get("yaw_sign", 1))
    if "yaw_sign" in (veh.get("unverified") or []):
        print("⚠ vehicle.yaw_sign がまだ仮の値です。符号が逆だと、対応表の左右も逆になります。")

    cmds, angles = [], []
    paths = sorted({p for pat in args.logs for p in glob.glob(pat)})
    for path in paths:
        imu_t, gz, st_t, speed, cmd_t, steer = load(path)
        if len(imu_t) < 10 or len(st_t) < 10 or len(cmd_t) < 10:
            print(f"  {os.path.basename(path)}: IMU・車速・ステア指令のいずれかがないため除外")
            continue
        used = 0
        for t, v in zip(st_t, speed):
            if v < args.min_speed:
                continue
            win = (cmd_t > t - 1.0) & (cmd_t <= t)          # steering held for the last second
            if win.sum() < 3:
                continue
            s = steer[win]
            if s.max() - s.min() > 0.06:
                continue
            g = (imu_t > t - 0.15) & (imu_t <= t + 0.15)
            if not g.any():
                continue
            kappa = yaw_sign * float(gz[g].mean()) / v
            cmds.append(float(s.mean()))
            angles.append(math.degrees(math.atan(L * kappa)))
            used += 1
        print(f"  {os.path.basename(path)}: {used} サンプル")
    if not cmds:
        sys.exit("使えるサンプルがありません（ステアを一定に保って、0.3 m/s 以上で走った区間が必要です）")

    cmds, angles = np.array(cmds), np.array(angles)
    print(f"\nステア指令ごとの操舵角（軸距 {L} m、左が +）:")
    table = []
    for c in np.arange(-1.0, 1.0 + 1e-9, args.bin):
        m = np.abs(cmds - c) <= args.bin / 2
        if m.sum() < args.min_samples:
            continue
        a = float(np.median(angles[m]))
        radius = L / math.tan(math.radians(abs(a))) if abs(a) > 0.3 else float("inf")
        print(f"  指令 {c:+.1f}: {a:+6.1f}°（{m.sum():4d} サンプル、ばらつき {np.std(angles[m]):.1f}°、回転半径 {radius:.1f} m）")
        table.append([round(float(c), 2), round(a, 1)])
    if not any(abs(c) < 1e-6 for c, _ in table):
        table.append([0.0, 0.0])
        table.sort()
    angs = [a for _, a in table]
    if len(table) < 3:
        sys.exit("指令の種類が少なすぎます（左右それぞれ 1 か所以上で一定のステアの走行が必要です）")
    if not (all(x > y for x, y in zip(angs, angs[1:])) or all(x < y for x, y in zip(angs, angs[1:]))):
        print("⚠ 角度が指令に対して単調になっていません。サンプルの少ない指令を除くか、走り直してください。")
    elif angs[0] < angs[-1]:
        print("⚠ 右（+）の指令で左に曲がっています。vehicle.yaw_sign かステアの配線・設定の向きを確認してください。")
    print("\nconfig.yaml の vehicle に貼り付け、unverified から steer_table を消してください"
          "（指令 ±1 のサンプルがなければ、端は直線で外挿されません。左右いっぱいでも走ってください）:")
    print(f"  steer_table: {json.dumps(table)}")


if __name__ == "__main__":
    main()
