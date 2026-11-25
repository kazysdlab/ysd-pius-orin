"""制御（waypoint_controller.py）の閉ループ・シミュレーション（実車を使わない確認用）

楕円形のコース（直線 + 半円）を、運動学モデルの車両に走らせます。モデルの代わりに「正しい waypoint」を
10 Hz・遅れ付きで与え、制御の符号・注視点・速度制御・再投影が正しく働くかを確かめます。
実車の操舵の効き（--true-max-steer）や車速の遅れを変えて、仮のパラメータでどこまで走れるかも見られます。
車両は補足資料のとおり、スロットルに応じた速度（controller.throttle_table）に向かって加速します。実車がその速度に
届かない場合（坂・重さ）は --speed-scale で、ステアが小さな変化に反応しない幅は --steer-deadband で変えられます。

使い方（ホストで実行）:
    python3 tools/sim_controller.py
    python3 tools/sim_controller.py --true-max-steer 18 --radius 4 --png sim.png
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'pius_control', 'pius_control'))
from calibration import load_config  # noqa: E402
from waypoint_controller import THROTTLE_TABLE, SteerMap, WaypointController  # noqa: E402


def make_track(straight, radius, step=0.02):
    """Closed racetrack (counter-clockwise): points every `step` m."""
    pts = []
    for s in np.arange(0, straight, step):
        pts.append((s, 0.0))
    for a in np.arange(-math.pi / 2, math.pi / 2, step / radius):
        pts.append((straight + radius * math.cos(a), radius + radius * math.sin(a)))
    for s in np.arange(straight, 0, -step):
        pts.append((s, 2 * radius))
    for a in np.arange(math.pi / 2, 3 * math.pi / 2, step / radius):
        pts.append((radius * math.cos(a), radius + radius * math.sin(a)))
    return np.array(pts)


def main():
    ap = argparse.ArgumentParser(description="制御の閉ループ・シミュレーション")
    ap.add_argument("--straight", type=float, default=12.0)
    ap.add_argument("--radius", type=float, default=5.0)
    ap.add_argument("--time", type=float, default=80.0)
    ap.add_argument("--true-max-steer", type=float, default=None, help="実車の最大操舵角 [deg]（既定: 設定の対応表と同じ）")
    ap.add_argument("--latency", type=float, default=0.1, help="画像から waypoint が届くまでの遅れ [s]")
    ap.add_argument("--speed-scale", type=float, default=1.0, help="実車の速度 / 表の速度（例 0.8: 表より 2 割遅い）")
    ap.add_argument("--steer-deadband", type=float, default=0.06,
                    help="ステアが反応しない指令の幅（補足資料: ±0.020 ms ≒ 指令 0.06）")
    ap.add_argument("--png", default=None)
    args = ap.parse_args()

    cfg = load_config(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'config.yaml'))
    p = dict(cfg.get('controller', {}))
    veh = cfg.get('vehicle', {})
    p['wheelbase_m'] = veh.get('wheelbase_m', 1.5)
    p['steer_table'] = veh.get('steer_table')
    ctl = WaypointController(p)
    if args.true_max_steer:
        m = args.true_max_steer
        true_map = SteerMap([[-1.0, m], [0.0, 0.0], [1.0, -m]])
    else:
        true_map = SteerMap(p['steer_table'])

    track = make_track(args.straight, args.radius)
    seg = np.diff(track, axis=0)
    s_track = np.concatenate([[0], np.cumsum(np.hypot(seg[:, 0], seg[:, 1]))])
    L = p['wheelbase_m']
    table = np.array(sorted(p.get('throttle_table', THROTTLE_TABLE)))
    x, y, psi, v, delta = 0.0, 0.3, 0.0, 0.0, 0.0     # start 0.3 m off the line
    steer_pos = 0.0                                    # where the steering actuator settled (command units)
    dt, t, next_wp = 0.02, 0.0, 0.0
    pending = []
    log = []
    while t < args.time:
        # "perfect model": the path 1..5 m ahead of the closest track point, sent with a delay
        if t >= next_wp:
            i = int(np.argmin(np.hypot(track[:, 0] - x, track[:, 1] - y)))
            pts = []
            for k in range(1, 6):
                s = (s_track[i] + k) % s_track[-1]
                j = int(np.searchsorted(s_track, s))
                gx, gy = track[min(j, len(track) - 1)]
                c, sn = math.cos(psi), math.sin(psi)
                pts.append((c * (gx - x) + sn * (gy - y), -sn * (gx - x) + c * (gy - y)))
            pending.append((t + args.latency, t, pts))
            next_wp = t + 0.1
        while pending and pending[0][0] <= t:
            _, t_img, pts = pending.pop(0)
            ctl.set_waypoints(t_img, pts)
        yaw_rate = v * math.tan(delta) / L
        ctl.update_odometry(t, v, yaw_rate)
        out = ctl.step(t, v)
        # Vehicle: the steering moves only for changes beyond the dead band, then lags
        if abs(out["steer"] - steer_pos) > args.steer_deadband:
            steer_pos = out["steer"]
        delta += (math.radians(true_map.to_angle(steer_pos)) - delta) * min(dt / 0.2, 1.0)
        # The motor controller drives towards the speed of the throttle (throttle_table), coasts down otherwise
        v_motor = args.speed_scale * float(np.interp(out["throttle"], table[:, 0], table[:, 1], left=0.0))
        acc = min((v_motor - v) / 0.8, 1.0) if v_motor > v else -0.15 - 0.2 * v
        acc -= 4.0 * out["brake"]
        v = max(0.0, v + acc * dt)
        x += v * math.cos(psi) * dt
        y += v * math.sin(psi) * dt
        psi += v * math.tan(delta) / L * dt
        err = float(np.min(np.hypot(track[:, 0] - x, track[:, 1] - y)))
        log.append((t, x, y, v, out.get("v_target", 0.0), err, out["steer"]))
        t += dt

    a = np.array(log)
    steady = a[a[:, 0] > 10]
    print(f"コース: 直線 {args.straight} m + 半径 {args.radius} m の半円、{args.time:.0f} 秒")
    print(f"横ずれ（10 秒以降）: 平均 {steady[:, 5].mean() * 100:.1f} cm、最大 {steady[:, 5].max() * 100:.1f} cm")
    print(f"速度（10 秒以降）: 平均 {steady[:, 3].mean():.2f} m/s（目標の平均 {steady[:, 4].mean():.2f}）、最高 {steady[:, 3].max():.2f} m/s")
    print(f"ステア指令: {a[:, 6].min():+.2f} 〜 {a[:, 6].max():+.2f}")
    if args.png:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))
        ax1.plot(track[:, 0], track[:, 1], color="#94a3b8", lw=6, alpha=.4, label="コース")
        ax1.plot(a[:, 1], a[:, 2], color="#0284c7", lw=1.5, label="車両（後軸中心）")
        ax1.set_aspect("equal"); ax1.legend(); ax1.grid(alpha=.3)
        ax2.plot(a[:, 0], a[:, 3], label="速度 [m/s]"); ax2.plot(a[:, 0], a[:, 4], label="目標速度 [m/s]")
        ax2.plot(a[:, 0], a[:, 5], label="横ずれ [m]"); ax2.plot(a[:, 0], a[:, 6], label="ステア指令", alpha=.6)
        ax2.legend(); ax2.grid(alpha=.3); ax2.set_xlabel("秒")
        fig.tight_layout(); fig.savefig(args.png, dpi=90)
        print(f"図: {args.png}")


if __name__ == "__main__":
    main()
