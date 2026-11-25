"""Provisional (not yet measured) parameters, so that they are not forgotten.

A config.yaml section can list keys whose values are provisional:

    wheel:
      circumference_m: 1.277
      unverified: [circumference_m]      # remove the key from this list once the real value is in

Every node that uses such a value warns at startup, the dashboard shows a banner, the health monitor lists
them, recordings store them (attribute `unverified_params`) and train_waypoint.py prints them. Values that
make AUTO unsafe when wrong (CRITICAL) also keep the controller from producing AUTO commands, unless
controller.allow_unverified is true (bench tests only).
"""
import os

import yaml

CONFIG_PATH = '/home/orin/dev/ws/colcon_ws/config.yaml'

# How to get the real value (shown in warnings and by tools/check_calibration.py)
HOW_TO = {
    'wheel.circumference_m': 'タイヤ外周を実測（人が乗った状態で 5 回転押して距離を測る。マニュアル 3.8）',
    'wheel.imu_forward_axis': 'IMU を固定した後、前進だけで加速・減速した走行ログから tools/imu_forward_axis.py で推定（測定ガイド手順 5 の A）',
    'camera.crop': 'ダッシュボードの破線の枠に空が入らないよう調整（マニュアル 3.9）',
    'camera.preview_white_balance': '白い紙を写して tools/camera_white_balance.py。屋外でも測り、値が近いか確認（表示用のみ。マニュアル 3.11）',
    'vehicle.yaw_sign': '左回り・右回りの円を走り tools/trajectory_check.py で向きの変化の符号を確認（マニュアル 9.3）',
    'vehicle.steer_table': 'いくつかの一定のステアで円を走り tools/steering_calibration.py で対応表を作る（マニュアル 10.8）',
    'camera_geometry.K': 'チェッカーボードを撮影し tools/camera_calibration.py intrinsics（マニュアル 10.9）',
    'camera_geometry.D': '同上（レンズの歪み係数）',
    'camera_geometry.position_m': '地面の印を撮影し tools/camera_calibration.py extrinsics（マニュアル 10.9）',
    'camera_geometry.rpy_deg': '同上（カメラの向き）',
}
DEFAULT_HOW_TO = '実車で調整する（マニュアル 8.8）'

# Wrong values here make the car steer or move unsafely in AUTO
CRITICAL = {'vehicle.steer_table', 'vehicle.yaw_sign', 'wheel.circumference_m'}


def load_config(path=CONFIG_PATH):
    if not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def unverified(cfg, sections=None):
    """[(name 'section.key', value, how_to, critical)] for every key listed in a section's `unverified`."""
    items = []
    for section, body in (cfg or {}).items():
        if not isinstance(body, dict) or (sections is not None and section not in sections):
            continue
        for key in body.get('unverified') or []:
            name = f'{section}.{key}'
            items.append((name, body.get(key), HOW_TO.get(name, DEFAULT_HOW_TO), name in CRITICAL))
    return items


def names(items):
    return [name for name, _, _, _ in items]


def warn_lines(items, title='仮の値（未確定）のパラメータがあります'):
    lines = [f'【要確認】{title}:']
    for name, value, how, critical in items:
        mark = '（AUTO に必須）' if critical else ''
        lines.append(f'  - {name} = {value}{mark} … {how}')
    return lines
