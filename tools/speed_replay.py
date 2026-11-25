"""車速ログ（tools/speed_live.py の記録）を、車速の推定処理（wheel_estimator.py）で再生して検証する

各パルスを採用したか・捨てたか（理由つき）を表示し、表示値の比較グラフを PNG で保存します。
推定処理の設定を変えたときに、実際に走ったログで効果を確かめるために使います。

使い方（ホストで実行）:
    python3 tools/speed_replay.py data/speed_logs/speed_YYYYmmdd_HHMMSS.jsonl
    python3 tools/speed_replay.py <ログ> --png out.png --circumference 1.277
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'pius_control', 'pius_control'))
from wheel_estimator import WheelSpeedEstimator  # noqa: E402

REASONS = {"too_fast": "速すぎる間隔", "not_moving": "IMU では停止中", "too_short": "発進直後で、IMU の移動距離が 1 回転に足りない",
           "too_sudden": "IMU の加速度では説明できない急な速度変化"}


def replay(path, circumference=None, pulses_per_rev=None, **kw):
    rows = []
    for line in open(path):
        try:
            rows.append(json.loads(line))
        except ValueError:              # a line cut off when the logger was stopped
            pass
    status = next((r for r in rows if r["k"] == "status" and r.get("circumference")), {})
    if circumference is None:
        circumference = status.get("circumference", 1.277)
    if pulses_per_rev is None:
        pulses_per_rev = int(status.get("pulses_per_rev") or 1)     # logs before 2026-10: 1 pulse per revolution
    events = []
    for r in rows:
        if r["k"] == "imu":
            events.append((r["t"], 0, r))
        elif r["k"] in ("pulse", "pulse_rejected"):   # replay every pulse the node saw
            events.append((r.get("stamp", r["t"]), 1, r))
        elif r["k"] == "status":
            events.append((r["t"], 2, r))
    events.sort(key=lambda e: (e[0], e[1]))
    if "forward_axis" not in kw and status.get("imu_forward_axis"):
        kw["forward_axis"] = tuple(status["imu_forward_axis"])      # the node's value when the log was recorded
    est = WheelSpeedEstimator(circumference, pulses_per_rev=pulses_per_rev, **kw)
    t0 = rows[0]["t"]
    out = {"t0": t0, "C": est.pulse_distance, "circumference": circumference, "pulses_per_rev": pulses_per_rev,
           "pulses": [], "series": []}
    for t, kind, r in events:
        if kind == 0:
            est.on_imu(t, r["acc"], r.get("gyro"))
        elif kind == 1:
            ok, reason, interval = est.on_pulse(t)
            out["pulses"].append((t - t0, ok, reason, interval))
        else:
            new, src = est.speed(t)
            old = r.get("speed_pulse", r.get("speed"))
            out["series"].append((t - t0, None if old is None else old * 3.6, None if new is None else new * 3.6, src))
    return out


def main():
    ap = argparse.ArgumentParser(description="車速ログを推定処理で再生して検証する")
    ap.add_argument("log")
    ap.add_argument("--circumference", type=float, default=None, help="タイヤ外周 [m]（既定: ログに記録された値）")
    ap.add_argument("--pulses-per-rev", type=int, default=None, help="1 回転あたりのパルス数（既定: ログの値）")
    ap.add_argument("--png", default=None, help="比較グラフの保存先（既定: ログと同じ名前の .png）")
    ap.add_argument("--quiet", action="store_true", help="パルスごとの表示を省略")
    args = ap.parse_args()

    out = replay(args.log, args.circumference, args.pulses_per_rev)
    C = out["C"]          # distance between two pulses
    if not args.quiet:
        for t, ok, reason, interval in out["pulses"]:
            iv = "   ---  " if interval is None else f"{interval:7.3f}s"
            kmh = "" if not interval else f"（{C / interval * 3.6:5.1f} km/h 相当）"
            mark = "採用" if ok else f"除外: {REASONS.get(reason, reason)}"
            print(f"{t:8.2f} s  間隔 {iv}{kmh}  {mark}")
    n_ok = sum(1 for p in out["pulses"] if p[1])
    print(f"\nパルス {len(out['pulses'])} 回 → 採用 {n_ok} 回、除外 {len(out['pulses']) - n_ok} 回（外周 {out['circumference']} m・1 回転 {out['pulses_per_rev']} パルス、距離 {n_ok * C:.1f} m）")
    old_max = max((s[1] for s in out["series"] if s[1] is not None), default=0)
    new_max = max((s[2] for s in out["series"] if s[2] is not None), default=0)
    print(f"最高速度の表示: これまで {old_max:.1f} km/h → 新しい推定 {new_max:.1f} km/h")

    png = args.png or os.path.splitext(args.log)[0] + "_replay.png"
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams["font.family"] = ["Noto Sans CJK JP", "DejaVu Sans"]
    except ImportError:
        print("matplotlib がないためグラフは省略します")
        return
    s = out["series"]
    fig, ax = plt.subplots(figsize=(14, 4.5))
    ax.plot([x[0] for x in s], [x[1] if x[1] is not None else float("nan") for x in s],
            color="#94a3b8", lw=1.2, label="これまでの表示値（パルスのみ）", drawstyle="steps-post")
    ax.plot([x[0] for x in s], [x[2] if x[2] is not None else float("nan") for x in s],
            color="#0284c7", lw=1.8, label="新しい推定値（パルス + IMU）")
    for t, ok, reason, interval in out["pulses"]:
        y = min(C / interval * 3.6, 9.5) if interval else 0.2
        ax.plot(t, y, "o" if ok else "v", color="#f59e0b" if ok else "#dc2626", ms=6)
    ax.plot([], [], "o", color="#f59e0b", label="採用したパルス")
    ax.plot([], [], "v", color="#dc2626", label="除外したパルス（上端 = 9.5 km/h 以上）")
    ax.set_ylim(0, 10)
    ax.set_xlabel("秒")
    ax.set_ylabel("km/h")
    ax.grid(alpha=.3)
    ax.legend(loc="upper left", fontsize=9, ncol=2)
    fig.tight_layout()
    fig.savefig(png, dpi=100)
    print(f"グラフ: {png}")


if __name__ == "__main__":
    main()
