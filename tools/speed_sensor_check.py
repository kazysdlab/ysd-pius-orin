"""車軸の速度センサー（リードスイッチ + 74HC14 バッファ基板）の動作確認スクリプト

配線: 74HC14 の出力 → 40ピンヘッダ 7番（BOARD 番号, GPIO09）、電源 1番（3.3V）、GND 9番。
ブレーキディスクのマグネットが通過してスイッチが閉じると、74HC14（反転）の出力が High になる
（立ち上がりエッジ）。タイヤ 1 回転につき、磁石の数（--ppr）だけパルスが出る。

使い方（Jetson 本体の端末で実行。gpio グループに入っていれば sudo 不要）:
    python3 tools/speed_sensor_check.py --raw                      # 配線確認: 入力レベルの変化を表示
    python3 tools/speed_sensor_check.py                            # パルス間隔・回転数を表示
    python3 tools/speed_sensor_check.py --circumference 1.277      # 速度 [km/h] も表示（タイヤ外周 [m]）
    python3 tools/speed_sensor_check.py --slow --circumference 1.277  # 手でゆっくり回す確認（通過ごとに表示、終了時にまとめ）
    python3 tools/speed_sensor_check.py --slow --ppr 4 --circumference 1.249  # 磁石 4 個のとき（磁石ごとの反応も比べる）
Ctrl + C で終了します。
"""
import argparse
import threading
import time

import Jetson.GPIO as GPIO


def raw_loop(pin):
    print("入力レベルの変化を表示します。マグネットを近づける（スイッチが閉じる）と 1、離すと 0 になれば配線は正常です。")
    print("※ 1 回の接近で 0/1 が細かく何度も切り替わるのはスイッチのチャタリングです（速度表示モードでは除去します）。")
    print("※ 変化がなくても 2 秒ごとに現在のレベルを表示します（動作中の確認用）。")
    last = None
    t0 = time.monotonic()
    last_print = t0
    while True:
        level = GPIO.input(pin)
        now = time.monotonic()
        if level != last:
            state = "スイッチ閉（マグネット検知）" if level else "スイッチ開"
            print(f"{now - t0:9.3f} s  レベル {level}  {state}", flush=True)
            last, last_print = level, now
        elif now - last_print >= 2.0:
            print(f"{now - t0:9.3f} s  （変化なし: レベル {level}）", flush=True)
            last_print = now
        time.sleep(0.0005)


def pulse_loop(pin, circumference, debounce_ms, timeout, ppr=1):
    """circumference here is the distance between two pulses (wheel circumference / ppr)."""
    lock = threading.Lock()
    state = {"count": 0, "last": None, "interval": None}

    def on_pulse(_channel):
        now = time.monotonic()
        with lock:
            if state["last"] is not None:
                state["interval"] = now - state["last"]
            state["last"] = now
            state["count"] += 1
            count, interval = state["count"], state["interval"]
        text = f"パルス #{count:4d}"
        if interval is None:
            text += "  （最初のパルス。速度は次のパルスとの間隔から計算します）"
        else:
            text += f"  間隔 {interval:6.3f} s  回転数 {60.0 / interval / ppr:6.1f} rpm"
            if circumference:
                text += f"  速度 {circumference / interval * 3.6:6.2f} km/h"
        print(text, flush=True)

    # Rising edge = switch closes. bouncetime ignores contact bounce and the bounce when the switch opens again.
    GPIO.add_event_detect(pin, GPIO.RISING, callback=on_pulse, bouncetime=debounce_ms)

    max_kmh = f"、約 {circumference / (debounce_ms / 1000) * 3.6:.0f} km/h まで" if circumference else ""
    min_kmh = f"（約 {circumference / timeout * 3.6:.1f} km/h 未満は停止と表示）" if circumference else ""
    print(f"BOARD {pin} 番ピンの立ち上がりを待っています。チャタリング除去 {debounce_ms} ms{max_kmh}。"
          f"{timeout:.0f} 秒パルスがなければ停止とみなします{min_kmh}。")

    was_moving = None
    while True:
        time.sleep(1.0)
        now = time.monotonic()
        with lock:
            last, interval, count = state["last"], state["interval"], state["count"]
        moving = last is not None and interval is not None and now - last <= timeout
        if moving and circumference:
            # Between pulses the true speed can only be lower than circumference / (time since last pulse)
            speed = circumference / max(interval, now - last) * 3.6
            print(f"  現在の推定速度 {speed:6.2f} km/h（累計 {count} パルス）", flush=True)
        elif not moving and was_moving is not False:
            print("  停止中（または低速すぎて検出できません）" if count else "  まだパルスがありません", flush=True)
        was_moving = moving


def slow_loop(pin, circumference, stable_ms, node_debounce_ms, passes, ppr=1):
    """Hand-turning test: report every magnet pass in detail and check how the ROS node would count it.

    A pass starts at the first raw rise and ends when the level has been stable Low for stable_ms.
    The filtered level only changes after the raw level has been stable for stable_ms, so it does not
    depend on how long the switch stays closed (which is long when the wheel is turned slowly).
    """
    stable = stable_ms / 1000.0
    node_debounce = node_debounce_ms / 1000.0
    t0 = time.monotonic()
    raw = GPIO.input(pin)
    raw_since = t0
    filtered = raw
    cur = None                  # pass in progress
    last_accepted = None        # last rising edge the ROS node would accept
    last_pass_start = None

    print(f"BOARD {pin} 番ピンを 0.5 ms ごとに読んでいます（{stable_ms} ms 安定したらレベル確定）。"
          "タイヤをゆっくり回してください。Ctrl + C で終了して結果をまとめます。")
    if raw:
        print("※ 今マグネットがスイッチの位置にあります（レベル 1）。少し回して離してから数え始めます。")
    print("", flush=True)

    while True:
        now = time.monotonic()
        level = GPIO.input(pin)
        if level != raw:
            raw, raw_since = level, now
            if cur is None and level == 1 and filtered == 0:
                cur = {"start": now, "edges": [], "closed_at": None}
            if cur is not None:
                cur["edges"].append((now, level))
                if level == 1:
                    # Same rule as the ROS node: a rising edge counts if debounce has passed since the last counted one
                    if last_accepted is None or now - last_accepted >= node_debounce:
                        last_accepted = now
                        cur["node_count"] = cur.get("node_count", 0) + 1
        elif level != filtered and now - raw_since >= stable:
            filtered = level
            if cur is not None and filtered == 1:
                cur["closed_at"] = raw_since
            elif cur is not None and filtered == 0:
                n = sum(1 for p in passes if not p["noise"]) + 1
                passes.append(_report_pass(cur, raw_since, n, last_pass_start, circumference, ppr))
                last_pass_start = cur["start"]
                cur = None
        if cur is not None and cur["closed_at"] is None and filtered == 0 and raw == 0 and now - raw_since >= stable:
            # Rose but never stayed High for stable_ms: noise, not a pass
            print(f"  （{now - t0:7.2f} s  {stable_ms} ms 未満の短いパルスを無視しました。"
                  f"ROS ノードは {cur.get('node_count', 0)} 回数えます）", flush=True)
            passes.append({"noise": True, "node_count": cur.get("node_count", 0)})
            cur = None
        time.sleep(0.0005)


def _report_pass(cur, opened_at, n, last_pass_start, circumference, ppr=1):
    """circumference here is the distance between two pulses (wheel circumference / ppr)."""
    edges = cur["edges"]
    closed_at = cur["closed_at"]
    closed_ms = (opened_at - closed_at) * 1000
    # Bounce span on closing: first raw rise .. level settled High. On opening: first fall after that .. settled Low.
    close_span = (closed_at - cur["start"]) * 1000
    open_edges = [t for t, lv in edges if t > closed_at]
    open_span = (opened_at - open_edges[0]) * 1000 if open_edges else 0.0
    node_count = cur.get("node_count", 0)
    magnet = (n - 1) % ppr + 1
    text = (f"通過 #{n:3d}{f'（磁石 {magnet}/{ppr}）' if ppr > 1 else ''}  閉じていた時間 {closed_ms:7.0f} ms  レベル変化 {len(edges):3d} 回"
            f"（跳ね返り 閉: {close_span:5.1f} ms / 開: {open_span:5.1f} ms）")
    if last_pass_start is not None:
        interval = cur["start"] - last_pass_start
        text += f"  前回から {interval:6.2f} s（{60.0 / interval / ppr:5.1f} rpm"
        if circumference:
            text += f"、{circumference / interval * 3.6:5.2f} km/h"
        text += "）"
    if circumference:
        text += f"  累計 {n * circumference:6.2f} m"
    if node_count != 1:
        text += f"  ⚠ ROS ノードの数え方だと {node_count} 回"
    print(text, flush=True)
    return {"noise": False, "closed_ms": closed_ms, "edges": len(edges), "close_span": close_span,
            "open_span": open_span, "node_count": node_count, "magnet": magnet}


def print_slow_summary(passes, circumference, node_debounce_ms, ppr=1):
    """circumference here is the distance between two pulses (wheel circumference / ppr)."""
    real = [p for p in passes if not p["noise"]]
    noise = [p for p in passes if p["noise"]]
    node_total = sum(p["node_count"] for p in passes)
    print("\n===== 結果 =====")
    if ppr > 1:
        print(f"検出した通過: {len(real)} 回 = タイヤ {len(real) / ppr:.2f} 回転（磁石 {ppr} 個として。"
              "実際に回した回転数 × {ppr} と同じなら正常）".replace("{ppr}", str(ppr)))
    else:
        print(f"検出した通過: {len(real)} 回（= タイヤの回転数。実際に回した回数と同じなら正常）")
    if circumference:
        print(f"距離: {len(real) * circumference:.2f} m（1 パルス {circumference:.3f} m として）")
    if ppr > 1 and len(real) >= ppr:
        print("磁石ごとの閉じていた時間（数え始めた磁石を 1 とする。大きく違う磁石は、距離・強さ・向きを見直す）:")
        for k in range(1, ppr + 1):
            ms = [p["closed_ms"] for p in real if p.get("magnet") == k]
            if ms:
                print(f"  磁石 {k}: 平均 {sum(ms) / len(ms):6.0f} ms（{len(ms)} 回）")
    if real:
        print(f"閉じていた時間: {min(p['closed_ms'] for p in real):.0f}〜{max(p['closed_ms'] for p in real):.0f} ms")
        print(f"跳ね返り（チャタリング）の最大: 閉じるとき {max(p['close_span'] for p in real):.1f} ms、"
              f"開くとき {max(p['open_span'] for p in real):.1f} ms")
    if noise:
        print(f"短いノイズ: {len(noise)} 回（無視しました）")
    verdict = "一致" if node_total == len(real) else "不一致 ⚠"
    print(f"ROS ノード（チャタリング除去 {node_debounce_ms} ms）の数え方: {node_total} 回 → 通過回数と{verdict}")


def main():
    ap = argparse.ArgumentParser(description="車軸の速度センサー（リードスイッチ）の動作確認")
    ap.add_argument("--pin", type=int, default=7, help="BOARD 番号のピン（既定: 7 = GPIO09）")
    ap.add_argument("--circumference", type=float, default=None, help="タイヤの外周 [m]。指定すると速度 [km/h] を表示")
    ap.add_argument("--ppr", type=int, default=1, help="1 回転あたりのパルス数（車輪の磁石の数。既定 1）")
    ap.add_argument("--debounce-ms", type=int, default=100,
                    help="この時間内の再検出を無視する [ms]（既定 100。チャタリングとスイッチが開くときの跳ね返り対策）")
    ap.add_argument("--timeout", type=float, default=3.0, help="この秒数パルスがなければ停止とみなす（既定 3.0）")
    ap.add_argument("--raw", action="store_true", help="配線確認: 入力レベルの変化をそのまま表示する")
    ap.add_argument("--slow", action="store_true",
                    help="手で回す低速テスト: マグネットの通過ごとに詳しく表示し、終了時に結果をまとめる")
    ap.add_argument("--stable-ms", type=int, default=20,
                    help="--slow: この時間レベルが変わらなければ確定とみなす [ms]（既定 20）")
    args = ap.parse_args()

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(args.pin, GPIO.IN)
    passes = []
    ppr = max(1, args.ppr)
    pulse_dist = args.circumference / ppr if args.circumference else None   # distance between two pulses
    try:
        if args.raw:
            raw_loop(args.pin)
        elif args.slow:
            slow_loop(args.pin, pulse_dist, args.stable_ms, args.debounce_ms, passes, ppr)
        else:
            pulse_loop(args.pin, pulse_dist, args.debounce_ms, args.timeout, ppr)
    except KeyboardInterrupt:
        if args.slow:
            print_slow_summary(passes, pulse_dist, args.debounce_ms, ppr)
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
