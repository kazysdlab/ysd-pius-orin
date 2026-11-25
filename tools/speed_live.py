"""車速のリアルタイム表示とログ記録（走行システムと同時に、コンテナ内で実行するテスト用プログラム）

走行システム（ros2 launch）が動いている状態で、車速センサー・IMU・操作量のトピックを購読し、
- ブラウザ（http://<JetsonのIP>:8888）に車速のグラフをリアルタイム表示
- 端末にパルスごとの間隔と速度を表示
- すべての受信データを data/speed_logs/speed_YYYYmmdd_HHMMSS.jsonl に記録（後で検証用）
します。システムの動作には一切影響しません（購読するだけ）。

使い方（ホストの端末で）:
    ./tools/speed_live.sh                 # ログ: speed_YYYYmmdd_HHMMSS.jsonl
    ./tools/speed_live.sh --name B_left   # ログ: speed_YYYYmmdd_HHMMSS_B_left.jsonl（走り方ごとに分けるとき）
Ctrl + C で終了します。もう一度起動すると、新しいファイルに記録します。
"""
import json
import os
import re
import threading
import time
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from control_msgs.msg import NormalizedControls

LOG_DIR = '/home/orin/dev/ws/colcon_ws/data/speed_logs'
HISTORY_SEC = 60.0
REASONS = {"too_fast": "速すぎる間隔", "not_moving": "IMU では停止中", "too_short": "発進直後で移動距離が足りない",
           "too_sudden": "IMU では説明できない急な速度変化"}


class SpeedLive(Node):
    def __init__(self, log_path):
        super().__init__('speed_live')
        self.lock = threading.Lock()
        self.t0 = time.time()
        self.log = open(log_path, 'w')
        self.log_path = log_path
        self.samples = deque()      # [t, node_speed_kmh, throttle, brake, acc_x] at 10 Hz (on each /wheel/status)
        self.pulses = deque()       # [t, speed_kmh_from_interval, interval]
        self.rejected = deque()     # [t, speed_kmh_from_interval or None, reason]
        self.status = {}
        self.first_count = None
        self.throttle = self.brake = None
        self.acc = None
        self.mode = None
        self.n_imu = 0
        self.create_subscription(String, '/wheel/pulse', self._on_pulse, 50)
        self.create_subscription(String, '/wheel/pulse_rejected', self._on_rejected, 50)
        self.create_subscription(String, '/wheel/status', self._on_status, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 100)
        self.create_subscription(NormalizedControls, '/cmd/normalized', self._on_cmd, 50)
        self.create_subscription(String, '/mux/mode', self._on_mode, 10)
        self.create_subscription(String, '/control/status', lambda m: self._write("control", **json.loads(m.data)), 10)
        self.create_timer(1.0, self.log.flush)

    def _t(self):
        return time.time() - self.t0

    def _pulse_distance(self, st=None):
        """Distance between two pulses [m] (circumference / magnets on the wheel)."""
        st = self.status if st is None else st
        if st.get("pulse_distance"):
            return st["pulse_distance"]
        return (st.get("circumference") or 0.0) / max(1, int(st.get("pulses_per_rev") or 1))

    def _write(self, kind, **data):
        self.log.write(json.dumps({"t": round(time.time(), 6), "k": kind, **data}) + "\n")

    def _on_pulse(self, msg):
        d = json.loads(msg.data)
        self._write("pulse", **d)
        circ = self._pulse_distance()
        interval = d.get("interval")
        kmh = circ / interval * 3.6 if circ and interval else None
        with self.lock:
            self.pulses.append([self._t(), kmh, interval])
        if interval is None:
            print(f"パルス #{d['count']}（最初のパルス）", flush=True)
        else:
            text = f"パルス #{d['count']:5d}  間隔 {interval:6.3f} s"
            if kmh is not None:
                text += f"  → {kmh:5.2f} km/h"
            print(text, flush=True)

    def _on_rejected(self, msg):
        d = json.loads(msg.data)
        self._write("pulse_rejected", **d)
        circ = self._pulse_distance()
        interval = d.get("interval")
        kmh = circ / interval * 3.6 if circ and interval else None
        with self.lock:
            self.rejected.append([self._t(), kmh, d.get("reason")])
        print(f"  （除外: {REASONS.get(d.get('reason'), d.get('reason'))}"
              + (f"、間隔 {interval:.3f} s" if interval else "") + "）", flush=True)

    def _on_status(self, msg):
        d = json.loads(msg.data)
        self._write("status", **d)
        speed = d.get("speed")
        with self.lock:
            self.status = d
            if self.first_count is None:
                self.first_count = d.get("count", 0)
            sp = d.get("speed_pulse")
            self.samples.append([self._t(), None if speed is None else speed * 3.6,
                                 self.throttle, self.brake, self.acc, None if sp is None else sp * 3.6])
            cutoff = self._t() - HISTORY_SEC
            while self.samples and self.samples[0][0] < cutoff:
                self.samples.popleft()
            while self.pulses and self.pulses[0][0] < cutoff:
                self.pulses.popleft()
            while self.rejected and self.rejected[0][0] < cutoff:
                self.rejected.popleft()

    def _on_imu(self, msg):
        a, g, q = msg.linear_acceleration, msg.angular_velocity, msg.orientation
        self._write("imu", acc=[a.x, a.y, a.z], gyro=[g.x, g.y, g.z], q=[q.x, q.y, q.z, q.w])
        self.acc = a.x
        self.n_imu += 1

    def _on_cmd(self, msg):
        values = {c.channel: c.normalized for c in msg.data}
        self._write("cmd", ch=values)
        self.throttle, self.brake = values.get(1), values.get(2)

    def _on_mode(self, msg):
        if msg.data != self.mode:
            self._write("mode", mode=msg.data)
            self.mode = msg.data

    def snapshot(self):
        with self.lock:
            st = dict(self.status)
            count = st.get("count", 0) - (self.first_count or 0)
            return {
                "t": self._t(),
                "status": st,
                "mode": self.mode,
                "distance": count * self._pulse_distance(st),
                "pulses_since_start": count,
                "samples": list(self.samples),
                "pulses": list(self.pulses),
                "rejected": list(self.rejected),
                "log": os.path.basename(self.log_path),
            }

    def close(self):
        self.log.flush()
        self.log.close()


PAGE = r"""<!doctype html><html lang="ja"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>車速テスト</title>
<style>
:root{--bg:#0f172a;--fg:#e2e8f0;--sub:#94a3b8;--card:#1e293b;--line:#38bdf8;--dot:#f59e0b;--thr:#475569}
body{margin:0;background:var(--bg);color:var(--fg);font-family:system-ui,sans-serif;padding:12px 16px}
.big{font-size:56px;font-weight:700;line-height:1}.unit{font-size:20px;color:var(--sub)}
.row{display:flex;flex-wrap:wrap;gap:8px 20px;margin:8px 0;color:var(--sub);font-size:15px}
.row b{color:var(--fg)}canvas{width:100%;height:320px;background:var(--card);border-radius:8px}
.legend{font-size:13px;color:var(--sub);margin-top:6px}.sw{display:inline-block;width:12px;height:12px;border-radius:2px;margin:0 4px 0 12px;vertical-align:-1px}
.warn{color:#f87171}
</style></head><body>
<div><span class="big" id="spd">--</span> <span class="unit">km/h（システムの推定値・<span id="src">--</span>）</span></div>
<div class="row">
<span>最後のパルスから <b id="age">--</b> 秒</span>
<span>直前の間隔 <b id="intv">--</b> 秒</span>
<span>パルス <b id="cnt">--</b> 回（除外 <b id="rej">--</b>）</span>
<span>距離 <b id="dist">--</b> m</span>
<span>モード <b id="mode">--</b></span>
</div>
<canvas id="c"></canvas>
<div class="legend"><span class="sw" style="background:var(--line)"></span>システムの推定値（パルス + IMU）
<span class="sw" style="background:#94a3b8"></span>パルスのみの値（以前の方式）
<span class="sw" style="background:var(--dot)"></span>採用したパルス（外周 ÷ 間隔）
<span class="sw" style="background:#ef4444"></span>除外したパルス
<span class="sw" style="background:var(--thr)"></span>アクセル（0〜1、目盛は右）</div>
<div class="legend">直近 <select id="win"><option>15</option><option selected>30</option><option>60</option></select> 秒 ・ ログ: <span id="log"></span> <span id="err" class="warn"></span></div>
<script>
const $=id=>document.getElementById(id);
function draw(d){
  const c=$("c"),dpr=window.devicePixelRatio||1,W=c.clientWidth,H=c.clientHeight;
  c.width=W*dpr;c.height=H*dpr;const g=c.getContext("2d");g.scale(dpr,dpr);
  const win=+$("win").value,t1=d.t,t0=t1-win,L=36,R=30,T=10,B=24,pw=W-L-R,ph=H-T-B;
  let ymax=6;for(const s of d.samples)if(s[1]!=null)ymax=Math.max(ymax,s[1]*1.2);
  for(const p of d.pulses)if(p[1]!=null)ymax=Math.max(ymax,p[1]*1.2);ymax=Math.min(10,Math.ceil(ymax));
  const X=t=>L+(t-t0)/win*pw,Y=v=>T+ph-v/ymax*ph;
  g.strokeStyle="#334155";g.fillStyle="#94a3b8";g.font="11px system-ui";g.lineWidth=1;
  for(let v=0;v<=ymax;v+=ymax>12?2:1){g.beginPath();g.moveTo(L,Y(v));g.lineTo(L+pw,Y(v));g.stroke();g.fillText(v,6,Y(v)+4);}
  for(let s=0;s<=win;s+=5){const x=X(t1-s);g.fillText(s?`-${s}s`:"今",x-10,H-6);}
  g.fillText("1",L+pw+6,Y(ymax)+4);g.fillText("0",L+pw+6,Y(0)+4);
  // throttle as area (0..1 mapped to full height)
  g.fillStyle="rgba(71,85,105,.45)";g.beginPath();let started=false;
  for(const s of d.samples){if(s[0]<t0||s[2]==null)continue;const x=X(s[0]),y=T+ph-Math.max(0,s[2])*ph;
    if(!started){g.moveTo(x,T+ph);started=true;}g.lineTo(x,y);}
  if(started){g.lineTo(X(d.samples[d.samples.length-1][0]),T+ph);g.fill();}
  // pulse-only speed (previous method)
  g.strokeStyle="#94a3b8";g.lineWidth=1;g.beginPath();let on2=false;
  for(const s of d.samples){if(s[0]<t0)continue;if(s[5]==null){on2=false;continue;}
    const x=X(s[0]),y=Y(Math.min(s[5],ymax));on2?g.lineTo(x,y):g.moveTo(x,y);on2=true;}g.stroke();
  // system estimate
  g.strokeStyle="#38bdf8";g.lineWidth=2;g.beginPath();let on=false;
  for(const s of d.samples){if(s[0]<t0)continue;if(s[1]==null){on=false;continue;}
    const x=X(s[0]),y=Y(s[1]);on?g.lineTo(x,y):g.moveTo(x,y);on=true;}g.stroke();
  // per-pulse speed
  g.fillStyle="#f59e0b";for(const p of d.pulses){if(p[0]<t0)continue;
    if(p[1]!=null){g.beginPath();g.arc(X(p[0]),Y(p[1]),4,0,7);g.fill();}
    g.fillRect(X(p[0])-0.5,T+ph-6,1,6);}
  g.fillStyle="#ef4444";for(const p of d.rejected){if(p[0]<t0)continue;const x=X(p[0]),y=Y(Math.min(p[1]??0.2,ymax));
    g.beginPath();g.moveTo(x-4,y-4);g.lineTo(x+4,y-4);g.lineTo(x,y+3);g.fill();}
}
async function tick(){
  try{const d=await (await fetch("/api",{cache:"no-store"})).json();$("err").textContent="";
    const st=d.status||{};$("spd").textContent=st.speed==null?"--":(st.speed*3.6).toFixed(2);
    $("src").textContent=st.source==="imu"?"IMU併用":st.source==="pulse"?"パルスのみ":"--";
    $("rej").textContent=d.rejected.length+"（直近）";
    $("age").textContent=st.last_pulse_age==null?"--":st.last_pulse_age.toFixed(1);
    const lp=d.pulses[d.pulses.length-1];$("intv").textContent=lp&&lp[2]!=null?lp[2].toFixed(2):"--";
    $("cnt").textContent=d.pulses_since_start;$("dist").textContent=d.distance.toFixed(1);
    $("mode").textContent=d.mode||"--";$("log").textContent=d.log;draw(d);
  }catch(e){$("err").textContent="接続が切れました（プログラムを再起動してください）";}
  setTimeout(tick,200);
}
tick();
</script></body></html>"""


def make_handler(node):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path.startswith("/api"):
                body = json.dumps(node.snapshot()).encode()
                ctype = "application/json"
            else:
                body = PAGE.encode()
                ctype = "text/html; charset=utf-8"
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass
    return Handler


def log_name(name, now=None):
    """speed_YYYYmmdd_HHMMSS[_name].jsonl. Characters other than letters, digits, - and _ become _."""
    stem = (now or datetime.now()).strftime("speed_%Y%m%d_%H%M%S")
    tag = re.sub(r"[^\w-]+", "_", (name or "").strip()).strip("_")
    return f"{stem}_{tag}.jsonl" if tag else f"{stem}.jsonl"


def main():
    import argparse
    ap = argparse.ArgumentParser(description="車速のリアルタイム表示とログ記録")
    ap.add_argument("--port", type=int, default=8888)
    ap.add_argument("--name", default="", help="ログのファイル名に付ける名前（例: A_straight → speed_<日時>_A_straight.jsonl）")
    args = ap.parse_args()

    os.makedirs(LOG_DIR, exist_ok=True)
    log_path = os.path.join(LOG_DIR, log_name(args.name))
    rclpy.init()
    node = SpeedLive(log_path)
    node._write("meta", name=args.name.strip())
    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(node))
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"ブラウザで http://<JetsonのIP>:{args.port} を開いてください。ログ: data/speed_logs/{os.path.basename(log_path)}")
    print("Ctrl + C で終了します。", flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        snap = node.snapshot()
        dur = snap["t"]
        node.close()
        server.shutdown()
        print(f"\n終了: {dur:.0f} 秒、パルス {snap['pulses_since_start']} 回、距離 {snap['distance']:.1f} m、"
              f"IMU {node.n_imu} サンプル。ログ: data/speed_logs/{os.path.basename(log_path)}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
