import ipaddress
import json
import os
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool, String
from control_msgs.msg import NormalizedControls, PwmControls
from gamepad_msgs.msg import Gamepad

INDEX_HTML = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dashboard', 'index.html')

# Nodes shown on the connection panel, in display order (key in /system/health -> label)
HEALTH_KEYS = [
    ('camera', 'カメラ'),
    ('gamepad', 'コントローラー'),
    ('parser', '操作解析'),
    ('mux', 'モード管理'),
    ('inference', 'AI推論'),
    ('serial', '車両シリアル'),
    ('save', 'データ保存'),
    ('imu', 'IMU/GPS'),
    ('wheel', '車速センサー'),
    ('control', '自動運転の制御'),
]


class DashboardNode(Node):
    """
    Read-only web dashboard. Aggregates the system topics and serves:
      GET /             -> dashboard page
      GET /api/state    -> JSON snapshot of the current system state
      GET /stream.mjpg  -> camera image as MJPEG

    Future path prediction: publish std_msgs/String JSON on /inference/path as
    {"points": [[x, y], ...]} with x, y normalized to the image (0..1, origin top-left).
    """

    def __init__(self):
        super().__init__('dashboard_node')

        config_path = '/home/orin/dev/ws/colcon_ws/config.yaml'
        cfg = {}
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                cfg = yaml.safe_load(f) or {}
        else:
            self.get_logger().error(f"Configuration file not found at {config_path}!")
        dash = cfg.get('dashboard', {})
        self.host = dash.get('host', '0.0.0.0')
        self.port = int(dash.get('port', 8080))
        # Who may open the dashboard: localhost and Tailscale only (the local network is refused)
        self.allowed_networks = [ipaddress.ip_network(n, strict=False) for n in dash.get(
            'allowed_networks', ['127.0.0.0/8', '::1/128', '100.64.0.0/10', 'fd7a:115c:a1e0::/48'])]
        self.docker_gateway = self._docker_gateway() if dash.get('allow_docker_gateway', True) else None
        self.refused = {}
        self.jpeg_quality = int(dash.get('jpeg_quality', 80))
        self.health_timeout = float(dash.get('health_timeout_sec', 3.0))

        topic_image = cfg.get('camera', {}).get('output_topic', '/camera/image_raw')
        topic_preview = cfg.get('camera', {}).get('preview_topic', '/camera/preview/compressed')
        self.preview_size = list(cfg.get('camera', {}).get('preview_size', [640, 360]))
        # Region of the camera image that the model sees (sky removed), as fractions of the image
        cam_cfg = cfg.get('camera', {})
        crop, cap = list(cam_cfg.get('crop', [])), list(cam_cfg.get('capture_size', [1280, 720]))
        self.crop = [crop[0] / cap[0], crop[1] / cap[1], crop[2] / cap[0], crop[3] / cap[1]] \
            if len(crop) == 4 and len(cap) == 2 else None
        # Ground -> image projection for the predicted path, and the provisional (not yet measured) parameters
        from pius_control.calibration import unverified
        self.geometry = cfg.get('camera_geometry') or None
        self.geometry_provisional = bool(self.geometry and self.geometry.get('unverified'))
        self.unverified = [{'name': n, 'critical': c} for n, _, _, c in unverified(cfg)]
        self.last_control_path = 0.0
        topic_gamepad = cfg.get('parser', {}).get('input_topic', '/gamepad/state')
        mux_cfg = cfg.get('mux', {})
        topic_manual = mux_cfg.get('input_topic_manual', '/cmd/normalized/manual')
        topic_auto = mux_cfg.get('input_topic_auto', '/cmd/normalized/auto')
        topic_final = mux_cfg.get('output_topic', '/cmd/normalized')
        topic_mode = mux_cfg.get('output_topic_status', '/mux/mode')
        parser_cfg = cfg.get('parser', {})
        topic_record = parser_cfg.get('output_topic_record', '/cmd/record')
        topic_shift = parser_cfg.get('output_topic_shift', '/cmd/shift')

        self.lock = threading.Lock()
        self.frame_cond = threading.Condition(self.lock)
        self.jpeg = None
        self.frame_seq = 0
        self.frame_info = {}
        self.frame_times = deque(maxlen=30)
        self.last_preview_time = 0.0

        self.health = {}
        self.notices = deque(maxlen=50)
        self.values = {}

        self.create_subscription(String, '/system/health', self._health_cb, 50)
        self.create_subscription(String, '/system/notice', self._notice_cb, 50)
        # The camera's preview topic is faster (smoother, less lag); the model input topic is the fallback
        self.create_subscription(CompressedImage, topic_preview, self._preview_cb, qos_profile_sensor_data)
        self.create_subscription(Image, topic_image, lambda m: self._image_cb(m, preview=False), qos_profile_sensor_data)
        self.create_subscription(Gamepad, topic_gamepad, self._gamepad_cb, 10)
        self.create_subscription(NormalizedControls, topic_manual, lambda m: self._controls_cb('manual', m), 10)
        self.create_subscription(NormalizedControls, topic_auto, lambda m: self._controls_cb('auto', m), 10)
        self.create_subscription(NormalizedControls, topic_final, lambda m: self._controls_cb('final', m), 10)
        self.create_subscription(String, topic_mode, lambda m: self._set('mode', m.data), 10)
        self.create_subscription(Bool, topic_record, lambda m: self._set('record', m.data), 10)
        self.create_subscription(String, topic_shift, lambda m: self._set('shift', m.data), 10)
        self.create_subscription(String, '/save/status', self._json_cb('save'), 10)
        self.create_subscription(String, cfg.get('imu', {}).get('status_topic', '/imu/status'), self._json_cb('imu'), 10)
        self.create_subscription(String, cfg.get('wheel', {}).get('status_topic', '/wheel/status'), self._json_cb('wheel'), 10)
        self.create_subscription(PwmControls, '/vehicle/pwm', self._pwm_cb, 10)
        self.create_subscription(String, '/inference/path', self._json_cb('path'), 10)
        self.create_subscription(String, cfg.get('inference', {}).get('output_topic_waypoints', '/inference/waypoints'),
                                 self._waypoints_cb, 10)
        self.create_subscription(String, cfg.get('controller', {}).get('status_topic', '/control/status'),
                                 self._control_cb, 10)

        self.server = ThreadingHTTPServer((self.host, self.port), self._make_handler())
        self.server.daemon_threads = True
        self.server_thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.server_thread.start()

        self.get_logger().info(
            f"【ダッシュボード】Webダッシュボードを起動しました。ブラウザで開いてください: http://localhost:{self.port}"
            f"（Jetson 本体）、http://<Tailscale の IP>:{self.port}（Tailscale 経由）。"
            f"ローカルネットワークからのアクセスは拒否します（許可: {', '.join(map(str, self.allowed_networks))}"
            + (f"、Docker 経由の localhost {self.docker_gateway}）" if self.docker_gateway else "）"))

    @staticmethod
    def _docker_gateway():
        """Inside the container, connections to localhost:<port> on the host arrive from the container's
        default gateway (Docker's port forwarding), so that address stands for localhost."""
        try:
            with open('/proc/net/route') as f:
                for line in f.readlines()[1:]:
                    fields = line.split()
                    if fields[1] == '00000000' and int(fields[3], 16) & 2:
                        return ipaddress.ip_address(socket.inet_ntoa(bytes.fromhex(fields[2])[::-1]))
        except (OSError, ValueError, IndexError):
            pass
        return None

    def is_allowed(self, client):
        try:
            ip = ipaddress.ip_address(client.split('%', 1)[0])
        except ValueError:
            return False
        if ip.version == 6 and ip.ipv4_mapped:
            ip = ip.ipv4_mapped
        if self.docker_gateway is not None and ip == self.docker_gateway:
            return True
        return any(ip in net for net in self.allowed_networks if net.version == ip.version)

    def note_refused(self, client):
        now = time.time()
        if now - self.refused.get(client, 0.0) > 60.0:      # at most once a minute per address
            self.refused[client] = now
            self.get_logger().warning(f"【ダッシュボード】許可していないアドレスからのアクセスを拒否しました: {client}"
                                      "（localhost と Tailscale 経由のみ許可）")

    # --- ROS callbacks ---
    def _set(self, key, value):
        with self.lock:
            self.values[key] = {'value': value, 't': time.time()}

    def _project(self, points):
        if not self.geometry or not points:
            return None
        from pius_control.camera_geometry import project_ground
        try:
            return [q for q in project_ground([tuple(p) for p in points], self.geometry) if q is not None]
        except Exception:
            return None

    def _waypoints_cb(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self._set('waypoints', d)
        # The controller's re-projected path is more current; use the raw prediction only without it
        if time.time() - self.last_control_path > 0.5:
            pts = self._project(d.get('points'))
            if pts:
                self._set('path', {'points': pts, 'source': 'inference', 'provisional': self.geometry_provisional})

    def _control_cb(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        self._set('control', d)
        if d.get('path'):
            pts = self._project(d['path'])
            la = self._project([d['lookahead']]) if d.get('lookahead') else None
            if pts:
                self.last_control_path = time.time()
                self._set('path', {'points': pts, 'lookahead': la[0] if la else None, 'source': 'control',
                                   'provisional': self.geometry_provisional})

    def _json_cb(self, key):
        def cb(msg):
            try:
                self._set(key, json.loads(msg.data))
            except ValueError:
                self.get_logger().warning(f"Invalid JSON on {key}: {msg.data[:80]}", throttle_duration_sec=10.0)
        return cb

    def _health_cb(self, msg):
        parts = msg.data.split(':', 2)
        if len(parts) < 2:
            return
        with self.lock:
            self.health[parts[0]] = {
                'state': parts[1],
                'detail': parts[2] if len(parts) > 2 else '',
                't': time.time(),
            }

    def _notice_cb(self, msg):
        level, source, text = (msg.data.split('|', 2) + ['', ''])[:3]
        with self.lock:
            self.notices.appendleft({'t': time.time(), 'level': level, 'source': source, 'text': text})

    def _controls_cb(self, key, msg):
        channels = {int(c.channel): float(c.normalized) for c in msg.data}
        self._set(key, {
            'steering': channels.get(0),
            'throttle': channels.get(1),
            'brake': channels.get(2),
        })

    def _pwm_cb(self, msg):
        self._set('pwm', [int(c.pwm) for c in msg.data])

    def _gamepad_cb(self, msg):
        a, b = msg.axis, msg.buttons
        self._set('gamepad', {
            'axis': {'lx': a.lx, 'ly': a.ly, 'rx': a.rx, 'ry': a.ry, 'l2': a.l2, 'r2': a.r2},
            'buttons': {name: bool(getattr(b, name)) for name in
                        ('cross', 'circle', 'square', 'triangle', 'l1', 'r1', 'share', 'option', 'ps')},
        })

    def _store_jpeg(self, jpeg, width, height, source):
        now = time.time()
        with self.frame_cond:
            self.jpeg = jpeg
            self.frame_seq += 1
            self.frame_times.append(now)
            self.frame_info = {'width': width, 'height': height, 't': now, 'source': source}
            self.frame_cond.notify_all()

    def _preview_cb(self, msg):
        # Already JPEG-encoded by the camera node: forward as is
        self.last_preview_time = time.time()
        self._store_jpeg(bytes(msg.data), self.preview_size[0], self.preview_size[1], 'preview')

    def _image_cb(self, msg, preview=False):
        if time.time() - self.last_preview_time < 1.0:
            return
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            frame = buf.reshape((msg.height, int(msg.step)))[:, :msg.width * 3].reshape((msg.height, msg.width, 3))
            if msg.encoding == 'rgb8':
                frame = frame[:, :, ::-1]
            ok, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        except Exception as e:
            self.get_logger().warning(f"Failed to encode camera image: {e}", throttle_duration_sec=10.0)
            return
        if not ok:
            return
        self._store_jpeg(jpeg.tobytes(), msg.width, msg.height, 'model_input')

    # --- snapshot ---
    def snapshot(self):
        now = time.time()
        with self.lock:
            health = []
            for key, label in HEALTH_KEYS:
                h = self.health.get(key)
                if h is None:
                    state, detail, age = 'WAITING', '起動待ち', None
                else:
                    age = now - h['t']
                    state, detail = h['state'], h['detail']
                    if age > self.health_timeout:
                        state, detail = 'LOST', f'{age:.0f}秒間応答がありません（ノード停止の可能性）'
                health.append({'key': key, 'label': label, 'state': state, 'detail': detail, 'age': age})

            values = {k: {'value': v['value'], 'age': now - v['t']} for k, v in self.values.items()}
            times = list(self.frame_times)
            fps = (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else 0.0
            camera = dict(self.frame_info)
            if camera:
                camera['age'] = now - camera.pop('t')
                camera['fps'] = fps
            notices = [dict(n, age=now - n['t']) for n in list(self.notices)[:20]]
        return {'time': now, 'health': health, 'values': values, 'camera': camera, 'notices': notices,
                'crop': self.crop, 'unverified': self.unverified}

    # --- HTTP ---
    def _make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, fmt, *args):
                pass

            def _send(self, code, body, content_type):
                self.send_response(code)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split('?', 1)[0]
                if not node.is_allowed(self.client_address[0]):
                    node.note_refused(self.client_address[0])
                    try:
                        self._send(403, 'このダッシュボードは Jetson 本体（localhost）と Tailscale 経由でのみ利用できます。\n'
                                   .encode('utf-8'), 'text/plain; charset=utf-8')
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                    return
                try:
                    if path in ('/', '/index.html'):
                        with open(INDEX_HTML, 'rb') as f:
                            self._send(200, f.read(), 'text/html; charset=utf-8')
                    elif path == '/api/state':
                        body = json.dumps(node.snapshot(), ensure_ascii=False).encode('utf-8')
                        self._send(200, body, 'application/json; charset=utf-8')
                    elif path == '/stream.mjpg':
                        self._stream()
                    else:
                        self._send(404, b'not found', 'text/plain')
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def _stream(self):
                self.send_response(200)
                self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                last_seq = -1
                while rclpy.ok():
                    with node.frame_cond:
                        node.frame_cond.wait_for(lambda: node.frame_seq != last_seq, timeout=1.0)
                        if node.frame_seq == last_seq or node.jpeg is None:
                            continue
                        jpeg, last_seq = node.jpeg, node.frame_seq
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n')
                    self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b'\r\n')

        return Handler

    def destroy_node(self):
        self.server.shutdown()
        self.server.server_close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
