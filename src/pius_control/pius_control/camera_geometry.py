"""Projection of ground points (rear-axle frame) into the camera image (no ROS dependency).

Camera model: OpenCV fisheye (equidistant + k1..k4), intrinsics K / D in pixels of the 1280x720 capture.
Pose: camera position [forward, left, up] from the rear-axle centre on the ground, and orientation
[roll, pitch (down +), yaw (left +)] in degrees. Values live in config.yaml `camera_geometry` and come from
tools/camera_calibration.py (checkerboard for K / D, ground markers for the pose).
"""
import math

import numpy as np

IMAGE_SIZE = (1280, 720)


def rotation(rpy_deg):
    """Camera body axes (x forward, y left, z up) expressed in the vehicle frame: Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = (math.radians(a) for a in rpy_deg)
    rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0, math.sin(p)], [0, 1, 0], [-math.sin(p), 0, math.cos(p)]])
    rz = np.array([[math.cos(y), -math.sin(y), 0], [math.sin(y), math.cos(y), 0], [0, 0, 1]])
    return rz @ ry @ rx


# vehicle-style camera body axes (x fwd, y left, z up) -> OpenCV optical axes (x right, y down, z fwd)
BODY_TO_OPTICAL = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=float)


def world_to_optical(points_xyz, geom):
    R = rotation(geom.get("rpy_deg", [0, 0, 0]))
    t = np.asarray(geom.get("position_m", [0, 0, 1]), dtype=float)
    body = (np.asarray(points_xyz, dtype=float) - t) @ R          # = R^T (p - t) for each row
    return body @ BODY_TO_OPTICAL.T


def fisheye_project(optical, K, D):
    """OpenCV fisheye model. optical [N, 3] -> pixels [N, 2], valid mask (in front of the camera)."""
    K = np.asarray(K, dtype=float)
    k1, k2, k3, k4 = (list(D) + [0, 0, 0, 0])[:4]
    X, Y, Z = optical[:, 0], optical[:, 1], optical[:, 2]
    valid = Z > 0.05
    r = np.hypot(X, Y)
    theta = np.arctan2(r, Z)
    td = theta * (1 + k1 * theta**2 + k2 * theta**4 + k3 * theta**6 + k4 * theta**8)
    scale = np.where(r > 1e-9, td / np.maximum(r, 1e-9), 0.0)
    u = K[0, 0] * X * scale + K[0, 1] * Y * scale + K[0, 2]
    v = K[1, 1] * Y * scale + K[1, 2]
    return np.stack([u, v], axis=1), valid


def project_ground(points_xy, geom, normalized=True):
    """Ground points [[x forward, y left], ...] (rear-axle frame, z = 0) -> image points.

    Returns a list with [u, v] (normalised 0..1 of the 1280x720 image when `normalized`) or None for points
    behind the camera.
    """
    if not points_xy:
        return []
    pts = np.array([[x, y, 0.0] for x, y in points_xy])
    uv, valid = fisheye_project(world_to_optical(pts, geom), geom["K"], geom.get("D", [0, 0, 0, 0]))
    if normalized:
        uv = uv / np.array(IMAGE_SIZE, dtype=float)
    return [[round(float(a), 4), round(float(b), 4)] if ok else None for (a, b), ok in zip(uv, valid)]
