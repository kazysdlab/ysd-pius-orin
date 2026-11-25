"""Vehicle-side controller: predicted waypoints -> steering / throttle / brake (no ROS dependency).

Used by waypoint_controller_node (50 Hz) and by tools/sim_controller.py (closed-loop simulation).

- Re-projection: the network predicts waypoints at 10-15 Hz. Between predictions the car moves, so the last
  waypoints are kept in an odometry frame (integrated from the wheel speed and the IMU yaw rate) and
  re-expressed in the current rear-axle frame at every control step.
- Lateral: Pure Pursuit on the rear axle. The look-ahead point is where the path crosses a circle of radius
  l_d = clamp(base + gain * v); delta = atan(2 L sin(alpha) / l_d). The steering angle is turned into the
  normalised steering command with vehicle.steer_table (measured, see tools/steering_calibration.py).
- Longitudinal: target speed = set speed, lowered in curves so that v^2 * curvature <= max lateral
  acceleration; throttle = feed-forward + PI on the measured speed; brake when clearly too fast.
  The motor controller turns the throttle into a speed of its own (PIUS 補足資料, アクセル操作), so the
  feed-forward is the throttle that gives the target speed, looked up in controller.throttle_table.
"""
import math
from collections import deque

import numpy as np


# [throttle 0..1, speed m/s] from the PIUS 補足資料 (アクセル操作, reference values): pulse 1.2 ms -> 0 km/h,
# 1.4 -> 0.5, 1.5 -> 1, 1.6 -> 2, 1.8 -> 4, 2.0 -> 5 km/h. Throttle t is sent as 1.0 + t ms (sender_node).
THROTTLE_TABLE = [[0.2, 0.0], [0.4, 0.139], [0.5, 0.278], [0.6, 0.556], [0.8, 1.111], [1.0, 1.389]]


class ThrottleMap:
    """controller.throttle_table [[throttle, speed m/s], ...]: the throttle that gives a speed."""

    def __init__(self, table):
        t = sorted((float(th), float(v)) for th, v in table)
        self.th = np.array([th for th, _ in t])
        self.v = np.array([v for _, v in t])
        if len(t) < 2 or not np.all(np.diff(self.th) > 0) or not np.all(np.diff(self.v) >= 0):
            raise ValueError("throttle_table の速度がスロットルに対して単調に増えていません")

    def throttle_for(self, speed):
        """Smallest throttle for the speed (the start of the table for 0, the end beyond the top speed)."""
        if speed <= self.v[0]:
            return float(self.th[0]) if speed > 0 else 0.0
        return float(np.interp(speed, self.v, self.th))


class SteerMap:
    """vehicle.steer_table [[command -1..1, wheel angle deg (left +)], ...] and its inverse."""

    def __init__(self, table):
        t = sorted((float(c), float(a)) for c, a in table)
        self.cmd = np.array([c for c, _ in t])
        self.ang = np.array([a for _, a in t])
        order = np.argsort(self.ang)
        self.ang_sorted, self.cmd_by_ang = self.ang[order], self.cmd[order]
        if not (np.all(np.diff(self.ang) < 0) or np.all(np.diff(self.ang) > 0)):
            raise ValueError("steer_table の角度が指令に対して単調ではありません")
        self.max_left = float(self.ang.max())
        self.max_right = float(self.ang.min())

    def to_command(self, angle_deg):
        a = min(max(angle_deg, self.max_right), self.max_left)
        return float(np.interp(a, self.ang_sorted, self.cmd_by_ang))

    def to_angle(self, command):
        return float(np.interp(command, self.cmd, self.ang))


def _lookahead_point(path, ld):
    """First point at distance ld from the origin along the polyline (origin first). None if behind."""
    pts = [(0.0, 0.0)] + [tuple(p) for p in path if p[0] > -0.5]
    for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
        d0, d1 = math.hypot(x0, y0), math.hypot(x1, y1)
        if d0 <= ld <= d1 and d1 > d0:
            # Solve |p0 + s (p1 - p0)| = ld for s in [0, 1]
            dx, dy = x1 - x0, y1 - y0
            a, b, c = dx * dx + dy * dy, 2 * (x0 * dx + y0 * dy), x0 * x0 + y0 * y0 - ld * ld
            disc = max(b * b - 4 * a * c, 0.0)
            s = (-b + math.sqrt(disc)) / (2 * a)
            s = min(max(s, 0.0), 1.0)
            return x0 + s * dx, y0 + s * dy
    far = max(pts, key=lambda p: math.hypot(*p))
    return far if math.hypot(*far) > 0.3 and far[0] > 0 else None


def path_curvature(path):
    """Largest |curvature| [1/m] of consecutive point triples (Menger curvature), 0 for < 3 points."""
    pts = [(0.0, 0.0)] + [tuple(p) for p in path]
    k = 0.0
    for (ax, ay), (bx, by), (cx, cy) in zip(pts, pts[1:], pts[2:]):
        ab, bc, ca = math.hypot(bx - ax, by - ay), math.hypot(cx - bx, cy - by), math.hypot(ax - cx, ay - cy)
        if ab * bc * ca < 1e-6:
            continue
        cross = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax)
        k = max(k, abs(2 * cross / (ab * bc * ca)))
    return k


class WaypointController:
    def __init__(self, p):
        self.p = p
        self.L = float(p.get('wheelbase_m', 1.5))
        self.steer_map = SteerMap(p.get('steer_table', [[-1.0, 23.8], [0.0, 0.0], [1.0, -23.8]]))
        self.throttle_map = ThrottleMap(p.get('throttle_table', THROTTLE_TABLE))
        self.pose = (0.0, 0.0, 0.0)          # odometry x, y, heading
        self.pose_hist = deque()             # (t, x, y, psi) for re-projection
        self.last_t = None
        self.wp_odom = None                  # waypoints in the odometry frame
        self.wp_time = None
        self.integral = 0.0
        self.steer_cmd = 0.0
        self.last_step = None

    # ---- inputs -----------------------------------------------------------------------------------------
    def update_odometry(self, t, speed, yaw_rate):
        """speed [m/s] signed (negative when reversing), yaw_rate [rad/s] (left +)."""
        if self.last_t is not None and t > self.last_t:
            dt = min(t - self.last_t, 0.2)
            x, y, psi = self.pose
            mid = psi + 0.5 * yaw_rate * dt
            self.pose = (x + speed * dt * math.cos(mid), y + speed * dt * math.sin(mid), psi + yaw_rate * dt)
        self.last_t = t
        self.pose_hist.append((t, *self.pose))
        while self.pose_hist and self.pose_hist[0][0] < t - 2.0:
            self.pose_hist.popleft()

    def _pose_at(self, t):
        if not self.pose_hist:
            return self.pose
        best = min(self.pose_hist, key=lambda h: abs(h[0] - t))
        return best[1:]

    def set_waypoints(self, t, points):
        """points [[x, y], ...] in the rear-axle frame of the image taken at time t."""
        x0, y0, p0 = self._pose_at(t)
        c, s = math.cos(p0), math.sin(p0)
        self.wp_odom = [(x0 + c * px - s * py, y0 + s * px + c * py) for px, py in points]
        self.wp_time = t

    def current_path(self):
        """Waypoints in the current rear-axle frame."""
        if self.wp_odom is None:
            return []
        x, y, psi = self.pose
        c, s = math.cos(psi), math.sin(psi)
        return [(c * (px - x) + s * (py - y), -s * (px - x) + c * (py - y)) for px, py in self.wp_odom]

    def reset(self):
        """Forget the integrator and the steering rate state (called while not in AUTO)."""
        self.integral = 0.0
        self.last_step = None

    # ---- control ----------------------------------------------------------------------------------------
    def step(self, t, v_meas):
        """One control step. Returns a dict with steer (-1..1), throttle (0..1), brake (0..1) and diagnostics."""
        p = self.p
        path = self.current_path()
        v = max(v_meas, 0.0)
        ld = min(max(p['lookahead_base_m'] + p['lookahead_gain_s'] * v, p['lookahead_min_m']), p['lookahead_max_m'])
        target = _lookahead_point(path, ld)
        out = {"path": [[round(x, 3), round(y, 3)] for x, y in path], "lookahead_m": ld}
        if target is None:
            out.update(steer=self.steer_cmd, throttle=0.0, brake=1.0, ok=False, reason="経路が短すぎる・後ろ向き")
            return out
        tx, ty = target
        dist = math.hypot(tx, ty)
        alpha = math.atan2(ty, tx)
        kappa_pp = 2.0 * math.sin(alpha) / dist
        delta = math.atan(self.L * kappa_pp)
        cmd = self.steer_map.to_command(math.degrees(delta))
        dt = 0.0 if self.last_step is None else max(0.0, t - self.last_step)
        max_change = p['steer_rate_limit'] * dt if self.last_step is not None else 2.0
        self.steer_cmd += min(max(cmd - self.steer_cmd, -max_change), max_change)

        kappa = max(abs(kappa_pp), path_curvature(path))
        v_target = p['target_speed_mps']
        if kappa > 1e-3:
            v_target = min(v_target, math.sqrt(p['max_lateral_acc'] / kappa))
        v_target = max(v_target, min(p['min_speed_mps'], p['target_speed_mps']))

        err = v_target - v
        throttle = self.throttle_map.throttle_for(v_target) + p['speed_kp'] * err + p['speed_ki'] * self.integral
        brake = 0.0
        if 0.0 < throttle < p['max_throttle'] and dt > 0:
            self.integral = min(max(self.integral + err * dt, -2.0), 2.0)      # integrate only when not saturated
        throttle = min(max(throttle, 0.0), p['max_throttle'])
        if err < -0.1 or v > p['max_speed_mps']:
            throttle = 0.0
            brake = min(max(p['brake_gain'] * (-err - 0.1), 0.0), 1.0)
            if v > p['max_speed_mps']:
                brake = max(brake, 0.3)
        self.last_step = t
        out.update(steer=round(self.steer_cmd, 4), throttle=round(throttle, 4), brake=round(brake, 4), ok=True,
                   reason="", lookahead=[round(tx, 3), round(ty, 3)], delta_deg=round(math.degrees(delta), 2),
                   curvature=round(kappa, 4), v_target=round(v_target, 3), v_meas=round(v, 3))
        return out
