#!/usr/bin/env python3
"""ステアを決まった値で保つ（操舵角の測定用。docker・ROS なし、キーボードで操作）

走行システム（ROS）を止めた状態で、Jetson のシリアル（/dev/ttyTHS1）から Raspberry Pi の受信プログラムへ、
sender_node と同じ形式のパケットを 30 Hz で送り続けます。送るのはステアだけで、
アクセルは 1000 µs（オフ）・ブレーキはフル（1135 µs）・シフトは N に固定します（車は動きません）。

画面には、送っているパルス幅と、補足資料（村上商会）の設計値から換算した前輪の角度の目安を表示します。

使い方（ホストで実行。車両の MANUAL/PC スイッチは PC 側。Raspberry Pi の受信プログラムが動いていること）:
    sudo python3 tools/steer_hold.py              # dialout グループに入っていれば sudo は不要
    python3 tools/steer_hold.py --dry-run         # シリアルに送らず、表示だけ確かめる

キー:
    ← / →（または a / d）  0.05 ずつ左 / 右      , / .   0.01 ずつ左 / 右
    1〜9                     -1.00, -0.75, -0.50, -0.25, 0, +0.25, +0.50, +0.75, +1.00
    0 / c                    中央（0）
    q / Ctrl + C             中央に戻して終了（その後は受信側の安全停止: 中央・フルブレーキ）

ステアの変化は 1 秒あたり 1.0 までにしています（急に振らないため）。ステアは約 0.02 ms（指令 0.06）以内の
変化には反応しないので、まっすぐ（0）は、一度左右に切ってから戻した位置で測ってください。
"""
import argparse
import os
import select
import shutil
import struct
import subprocess
import sys
import termios
import time
import tty

# Same values and conversions as src/pius_control/pius_control/sender_node.py (keep them in sync)
STEER_MIN, STEER_CENTER, STEER_MAX = 1139, 1500, 1835
THROTTLE_STOP = 1000
BRAKE_FULL = 1135
STATE_NEUTRAL = 1
SEND_HZ = 30.0
SLEW_PER_SEC = 1.0
PRESETS = {'1': -1.0, '2': -0.75, '3': -0.5, '4': -0.25, '5': 0.0, '6': 0.25, '7': 0.5, '8': 0.75, '9': 1.0}

# PIUS 補足資料「ステアリング操作」(design values): pulse [ms] -> (right tyre, left tyre) [deg]
DESIGN = [(1.178, 24.58, 30.0), (1.234, 21.16, 25.0), (1.282, 17.5, 20.0), (1.333, 13.56, 15.0),
          (1.384, 9.35, 10.0), (1.453, 4.84, 5.0), (1.500, 0.0, 0.0), (1.545, 5.0, 4.84), (1.629, 10.0, 9.35),
          (1.694, 15.0, 13.56), (1.722, 20.0, 17.5), (1.771, 25.0, 21.16), (1.822, 30.0, 24.58)]


def pack_pius_packet(steering, throttle, brake, state):
    """Header 0xFFFF + 4 x int16 + checksum (sum of payload bytes & 0x7F). 11 bytes (sender_node.py)."""
    payload = struct.pack('<4h', steering, throttle, brake, state)
    return struct.pack('<H', 0xFFFF) + payload + bytes([sum(payload) & 0x7F])


def steer_pwm(s):
    """Normalised steering (-1 left .. +1 right) -> pulse width [us], as sender_node.py."""
    if s > 0.0:
        ms = s * (1.835 - 1.5) + 1.5
    elif s < 0.0:
        ms = (1.0 + s) * (1.5 - 1.139) + 1.139
    else:
        ms = 1.5
    return int(min(max(ms * 1000.0, STEER_MIN), STEER_MAX))


def design_angles(pwm_us):
    """(inner, outer, extrapolated) front wheel angles [deg] expected from the design table, or None at 0."""
    ms = pwm_us / 1000.0
    if abs(ms - 1.5) < 1e-6:
        return None
    rows = [(p, r, l) for p, r, l in DESIGN]
    if ms < rows[0][0] or ms > rows[-1][0]:
        a, b = (rows[0], rows[1]) if ms < rows[0][0] else (rows[-2], rows[-1])
        extra = True
    else:
        a, b = next((x, y) for x, y in zip(rows, rows[1:]) if x[0] <= ms <= y[0])
        extra = False
    f = (ms - a[0]) / (b[0] - a[0])
    right, left = a[1] + f * (b[1] - a[1]), a[2] + f * (b[2] - a[2])
    # turning right (> 1.5 ms): the right wheel is inside
    return (right, left, extra) if ms > 1.5 else (left, right, extra)


def open_serial(port, baud=115200):
    """Raw 8N1 serial without pyserial."""
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                                                     # iflag
    attrs[1] = 0                                                     # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL          # cflag
    attrs[3] = 0                                                     # lflag
    speed = getattr(termios, f'B{baud}')
    attrs[4] = attrs[5] = speed
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def system_running():
    """Name of what looks like the running driving system (it would also write to the port), or None."""
    if shutil.which('docker'):
        try:
            out = subprocess.run(['docker', 'ps', '--format', '{{.Names}}'], capture_output=True, text=True,
                                 timeout=5).stdout.split()
            if 'roscar2' in out:
                return 'コンテナ roscar2'
        except Exception:
            pass
    try:
        if subprocess.run(['pgrep', '-f', 'sender_node'], capture_output=True).returncode == 0:
            return 'sender_node'
    except Exception:
        pass
    return None


def read_key(timeout):
    if not select.select([sys.stdin], [], [], timeout)[0]:
        return None
    ch = os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
    if ch == '\x1b':                                                 # arrow keys: ESC [ C / D
        seq = ''
        while select.select([sys.stdin], [], [], 0.01)[0] and len(seq) < 2:
            seq += os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
        return {'[C': 'right', '[D': 'left'}.get(seq)
    return ch


def status_line(target, out, pwm):
    side = '右' if out > 0 else '左' if out < 0 else 'まっすぐ'
    d = design_angles(pwm)
    if d is None:
        guide = '設計値: 0°'
    else:
        inner, outer, extra = d
        guide = f"設計値の目安: 内側 {inner:4.1f}° / 外側 {outer:4.1f}°{'（表の外・推定）' if extra else ''}"
    moving = '' if abs(target - out) < 1e-6 else f'（{target:+.2f} へ移動中）'
    return f"ステア {out:+.2f}（{side}）{moving} → {pwm} µs | {guide} | ブレーキ: フル"


def main():
    ap = argparse.ArgumentParser(description="ステアを決まった値で保つ（操舵角の測定用）")
    ap.add_argument('--port', default='/dev/ttyTHS1')
    ap.add_argument('--dry-run', action='store_true', help='シリアルに送らない（表示の確認用）')
    ap.add_argument('--force', action='store_true', help='走行システムが動いていても送る（使わないでください）')
    args = ap.parse_args()

    running = system_running()
    if running and not args.dry_run and not args.force:
        sys.exit(f"走行システム（{running}）が動いています。同じポートに 2 か所から送ることになるため、"
                 "先に走行システムを Ctrl + C で止めてください。")
    fd = None
    if not args.dry_run:
        try:
            fd = open_serial(args.port)
        except PermissionError:
            sys.exit(f"{args.port} を開く権限がありません。sudo を付けて実行するか、"
                     "sudo usermod -aG dialout $USER のあと、ログインし直してください。")
        except OSError as e:
            sys.exit(f"{args.port} を開けません: {e}")
    if not sys.stdin.isatty():
        sys.exit("キーボードから操作する端末で実行してください。")

    print(__doc__.split('キー:')[1].split('ステアの変化')[0].rstrip())
    print("車両の MANUAL/PC スイッチが PC 側で、車が止まっていることを確かめてから操作してください。\n")
    target = out = 0.0
    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    period = 1.0 / SEND_HZ
    last = time.monotonic()
    quitting_until = None
    try:
        while True:
            key = read_key(period)
            if key in ('q', '\x03') and quitting_until is None:
                target = 0.0
                quitting_until = time.monotonic() + 1.0 / SLEW_PER_SEC + 0.5   # let it reach the centre
            elif quitting_until is None and key:
                if key in ('right', 'd'):
                    target += 0.05
                elif key in ('left', 'a'):
                    target -= 0.05
                elif key == '.':
                    target += 0.01
                elif key == ',':
                    target -= 0.01
                elif key in ('0', 'c'):
                    target = 0.0
                elif key in PRESETS:
                    target = PRESETS[key]
                target = round(min(max(target, -1.0), 1.0), 2)
            now = time.monotonic()
            dt, last = now - last, now
            step = SLEW_PER_SEC * dt
            out = out + min(max(target - out, -step), step)
            if abs(target - out) < 1e-3:
                out = target
            pwm = steer_pwm(out)
            packet = pack_pius_packet(pwm, THROTTLE_STOP, BRAKE_FULL, STATE_NEUTRAL)
            if fd is not None:
                try:
                    os.write(fd, packet)
                except BlockingIOError:
                    pass
            sys.stdout.write('\r\x1b[K' + status_line(target, out, pwm) + (' [DRY RUN]' if fd is None else ''))
            sys.stdout.flush()
            if quitting_until is not None and now >= quitting_until:
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        if fd is not None:
            os.close(fd)
        print("\n終了しました（受信側は 0.5 秒後に安全停止: ステア中央・フルブレーキ）。")


if __name__ == '__main__':
    main()
