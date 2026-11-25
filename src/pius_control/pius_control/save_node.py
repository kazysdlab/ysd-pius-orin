import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, NavSatFix
from control_msgs.msg import NormalizedControls
from std_msgs.msg import Bool, String
import numpy as np
import h5py
import yaml
import os
import time
import json
import atexit
from collections import deque

# v2: per-frame timestamps and the image color order attribute. v3: IMU/GPS columns (NaN when unavailable)
# v4: wheel speed columns and high-rate streams (all IMU samples, GPS, wheel pulses) for path ground truth
# v5: 320x128 images cropped from the 1280x720 camera (sky removed); camera geometry attributes
# v6: frames recorded while the car moves (not only with throttle, so pedal driving, coasting and braking are
#     included); steering / throttle / brake are NaN when no controller command is received; IMU mounting attributes
H5_FORMAT_VERSION = 6

# IMU/GPS columns recorded with every frame: name -> width (1 = scalar)
IMU_COLUMNS = {
    'imu_euler': 3,   # roll, pitch, yaw [deg]
    'imu_acc': 3,     # x, y, z [m/s^2]
    'imu_gyro': 3,    # x, y, z [deg/s]
    'gps': 6,         # latitude, longitude [deg], altitude [m], speed [m/s], heading [deg], satellites
    'gps_fix': 1,     # 1 = position fix, 0 = no fix
    'imu_stamp': 1,   # time the IMU values were received [s]
    'wheel_speed': 1,        # estimated vehicle speed from the wheel sensor [m/s] (NaN = unknown)
    'wheel_pulse_count': 1,  # cumulative wheel pulses (attribute wheel_pulses_per_rev per revolution)
}

# High-rate streams recorded independently of the camera frames: group -> [(dataset, width), ...]
# Units follow the ROS messages: orientation quaternion (x, y, z, w), angular velocity rad/s, acceleration m/s^2.
STREAMS = {
    'imu_hr': [('stamp', 1), ('orientation', 4), ('gyro', 3), ('acc', 3)],
    'gps_hr': [('stamp', 1), ('lat', 1), ('lon', 1), ('alt', 1), ('fix', 1)],
    'wheel': [('pulse_stamp', 1), ('pulse_count', 1)],
}


class SaveNode(Node):
    def __init__(self):
        super().__init__('save_node')

        # Load configuration
        config_path = '/home/orin/dev/ws/colcon_ws/config.yaml'
        cfg = {}
        self.wheel_circumference = 0.0
        self.wheel_pulses_per_rev = 1
        self.imu_attrs = {}
        self.camera_attrs = {}
        if not os.path.exists(config_path):
            self.get_logger().error(f"Configuration file not found at {config_path}!")
        else:
            with open(config_path, 'r') as f:
                full_cfg = yaml.safe_load(f) or {}
                cfg = full_cfg.get('save', {})
                self.wheel_circumference = float(full_cfg.get('wheel', {}).get('circumference_m', 0.0))
                self.wheel_pulses_per_rev = max(1, int(full_cfg.get('wheel', {}).get('pulses_per_rev', 1)))
                # How the IMU is mounted at recording time (the ground-truth path needs it; it changes when the IMU moves)
                axis = full_cfg.get('wheel', {}).get('imu_forward_axis', [-0.338, 0.941])
                self.imu_attrs = {'imu_forward_axis': [float(axis[0]), float(axis[1])],
                                  'vehicle_yaw_sign': float(full_cfg.get('vehicle', {}).get('yaw_sign', 1))}
                cam = full_cfg.get('camera', {})
                self.camera_attrs = {'camera_capture_size': cam.get('capture_size', []), 'camera_crop': cam.get('crop', []),
                                     'camera_flip_method': cam.get('flip_method', 0)}
                # Provisional values at recording time (e.g. the tyre circumference used for the wheel speed)
                from pius_control.calibration import names, unverified
                self.camera_attrs['unverified_params'] = ','.join(names(unverified(full_cfg))) or 'none'
        self.input_topic_image = cfg.get('input_topic_image', '/camera/image_raw')
        self.input_topic_controls = cfg.get('input_topic_controls', '/cmd/normalized')
        self.input_topic_record = cfg.get('input_topic_record', '/cmd/record')
        self.input_topic_mode = cfg.get('input_topic_mode', '/mux/mode')
        self.save_dir = cfg.get('save_dir', '/home/orin/dev/ws/colcon_ws/data')
        self.file_prefix = cfg.get('file_prefix', 'pius_drive')
        # When to record frames: "moving" = while the wheel sensor says the car moves (and stop_hold_sec after it
        # stopped), with or without controller commands; "throttle" = previous behaviour (throttle >= threshold)
        self.record_when = str(cfg.get('record_when', 'moving'))
        self.min_speed = float(cfg.get('min_speed_mps', 0.05))
        self.stop_hold = float(cfg.get('stop_hold_sec', 2.0))
        self.save_on_throttle_only = bool(cfg.get('save_on_throttle_only', True))
        self.min_throttle_threshold = float(cfg.get('min_throttle_threshold', 0.05))
        self.max_control_age = float(cfg.get('max_control_age_sec', 0.3))
        self.input_topic_imu = cfg.get('input_topic_imu', '/imu/status')
        self.max_imu_age = float(cfg.get('max_imu_age_sec', 0.3))
        self.pre_record_sec = float(cfg.get('pre_record_sec', 10.0))
        self.buffer_size = int(cfg.get('buffer_size', 100))
        self.verbose = int(cfg.get('verbose', 1))

        # Ensure save directory exists
        os.makedirs(self.save_dir, exist_ok=True)

        # Datasets
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.h5_filename = f"{self.file_prefix}_{timestamp}.h5"
        self.h5_filepath = os.path.join(self.save_dir, self.h5_filename)
        self.h5_file = None
        self.saved_frames = 0

        # State
        self.logging_enabled = True # Enabled by default, toggled via /cmd/record
        self.latest_controls = None
        self.latest_controls_stamp = 0.0
        self.current_mode = None
        self.mode_stamp = 0.0
        self.prev_frame = None
        self.last_saved_stamp = 0.0
        self.skip_reason = "起動待ち"
        self.latest_imu = None
        self.latest_imu_stamp = 0.0
        self.latest_wheel = None
        self.latest_wheel_stamp = 0.0
        self.last_moving_stamp = None
        self.streams = {group: deque() for group in STREAMS}
        self.first_frame_stamp = None
        self.stream_totals = {group: 0 for group in STREAMS}

        # Subscriptions
        self.sub_image = self.create_subscription(Image, self.input_topic_image, self._image_callback, rclpy.qos.qos_profile_sensor_data)
        self.sub_controls = self.create_subscription(NormalizedControls, self.input_topic_controls, self._controls_callback, 10)
        self.sub_record = self.create_subscription(Bool, self.input_topic_record, self._record_callback, 10)
        self.sub_mode = self.create_subscription(String, self.input_topic_mode, self._mode_callback, 10)
        self.sub_imu = self.create_subscription(String, self.input_topic_imu, self._imu_callback, 10)
        self.create_subscription(Imu, cfg.get('input_topic_imu_raw', '/imu/data'), self._imu_raw_callback, 100)
        self.create_subscription(NavSatFix, cfg.get('input_topic_gps', '/gps/fix'), self._gps_callback, 20)
        self.create_subscription(String, cfg.get('input_topic_wheel_pulse', '/wheel/pulse'), self._wheel_pulse_callback, 50)
        self.create_subscription(String, cfg.get('input_topic_wheel_status', '/wheel/status'), self._wheel_status_callback, 10)
        self.stream_timer = self.create_timer(2.0, self._flush_streams)

        # Status for the dashboard
        self.status_pub = self.create_publisher(String, '/save/status', 10)
        self.health_pub = self.create_publisher(String, '/system/health', 10)
        self.status_timer = self.create_timer(0.5, self._publish_status)

        # Buffers
        self.buf_images = []
        self.buf_steerings = []
        self.buf_throttles = []
        self.buf_brakes = []
        self.buf_image_stamps = []
        self.buf_control_stamps = []
        self.buf_imu = {name: [] for name in IMU_COLUMNS}

        self.get_logger().info(f"【データ保存】HDF5データ保存ノードを初期化しました。保存ファイル: {self.h5_filepath}")
        atexit.register(self._flush_and_close)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _controls_callback(self, msg: NormalizedControls):
        self.latest_controls = msg
        self.latest_controls_stamp = self._now()

    def _imu_callback(self, msg: String):
        try:
            self.latest_imu = json.loads(msg.data)
            self.latest_imu_stamp = self._now()
        except ValueError:
            pass

    # --- high-rate streams (recorded whenever logging is enabled, regardless of the frame conditions) ---
    def _stream_append(self, group, row):
        if self.logging_enabled:
            self.streams[group].append(row)

    def _imu_raw_callback(self, msg: Imu):
        o, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self._stream_append('imu_hr', [stamp, o.x, o.y, o.z, o.w, w.x, w.y, w.z, a.x, a.y, a.z])

    def _gps_callback(self, msg: NavSatFix):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        fix = 1.0 if msg.status.status >= 0 else 0.0
        nan = float('nan')
        lat, lon, alt = (msg.latitude, msg.longitude, msg.altitude) if fix else (nan, nan, nan)
        self._stream_append('gps_hr', [stamp, lat, lon, alt, fix])

    def _wheel_pulse_callback(self, msg: String):
        try:
            p = json.loads(msg.data)
            self._stream_append('wheel', [float(p['stamp']), float(p['count'])])
        except (ValueError, KeyError, TypeError):
            pass

    def _wheel_status_callback(self, msg: String):
        try:
            self.latest_wheel = json.loads(msg.data)
            self.latest_wheel_stamp = self._now()
        except ValueError:
            return
        speed = self.latest_wheel.get('speed')
        if speed is not None and speed > self.min_speed:
            self.last_moving_stamp = self.latest_wheel_stamp

    def _motion(self, now):
        """(moving, reversing, reason): from the wheel sensor; reason is set when the motion is unknown."""
        w = self.latest_wheel
        if w is None or now - self.latest_wheel_stamp > self.max_imu_age:
            return None, False, "車速が届いていません（車速センサーのノードの停止）"
        if w.get('speed') is None:
            return None, False, "車速が不明です（外周が未設定、または起動直後）"
        moving = self.last_moving_stamp is not None and now - self.last_moving_stamp <= self.stop_hold
        return moving, w.get('direction') == -1, ""

    def _flush_streams(self):
        if self.h5_file is not None:
            self._write_streams()
            return
        # The file is created at the first frame-buffer flush. Until then keep stream data from
        # pre_record_sec before the first buffered frame (or the last pre_record_sec if no frame yet).
        start = self.first_frame_stamp if self.first_frame_stamp is not None else self._now()
        cutoff = start - self.pre_record_sec
        for rows in self.streams.values():
            while rows and rows[0][0] < cutoff:
                rows.popleft()

    def _write_streams(self):
        try:
            for group, columns in STREAMS.items():
                rows = self.streams[group]
                if not rows:
                    continue
                data = np.array(rows, dtype='float64')
                rows.clear()
                g = self.h5_file.require_group(group)
                col = 0
                for name, width in columns:
                    values = data[:, col] if width == 1 else data[:, col:col + width]
                    col += width
                    if name not in g:
                        shape = (0,) if width == 1 else (0, width)
                        g.create_dataset(name, shape=shape, maxshape=(None,) + shape[1:], dtype='float64',
                                         chunks=(500,) + shape[1:])
                    ds = g[name]
                    n = ds.shape[0]
                    ds.resize(n + len(values), axis=0)
                    ds[n:] = values
                self.stream_totals[group] += len(data)
            self.h5_file.flush()
        except Exception as e:
            self.get_logger().error(f"Error writing high-rate streams to HDF5 file: {e}")

    def _imu_row(self, now):
        # The IMU is optional: missing or stale values are stored as NaN and never block recording
        nan3 = [float('nan')] * 3
        imu = self.latest_imu if (self.latest_imu is not None and now - self.latest_imu_stamp <= self.max_imu_age) else None
        wheel = self.latest_wheel if (self.latest_wheel is not None and now - self.latest_wheel_stamp <= self.max_imu_age) else None
        wheel_cols = {
            'wheel_speed': float('nan') if wheel is None or wheel.get('speed') is None else float(wheel['speed']),
            'wheel_pulse_count': float('nan') if wheel is None else float(wheel.get('count', float('nan'))),
        }
        if imu is None:
            return {'imu_euler': nan3, 'imu_acc': nan3, 'imu_gyro': nan3,
                    'gps': [float('nan')] * 6, 'gps_fix': float('nan'), 'imu_stamp': float('nan'), **wheel_cols}

        def vec(key):
            v = imu.get(key) or []
            return [float(x) for x in v] if len(v) == 3 else nan3

        g = imu.get('gps', {})
        # Without a fix the receiver reports 0/0; store NaN so it can never be mistaken for a real position
        gps = [g.get(k) if g.get('fix') else None for k in ('lat', 'lon', 'alt', 'speed', 'heading')] + [g.get('sats')]
        return {
            'imu_euler': vec('euler'),
            'imu_acc': vec('acc'),
            'imu_gyro': vec('gyro'),
            'gps': [float('nan') if x is None else float(x) for x in gps],
            'gps_fix': 1.0 if g.get('fix') else 0.0,
            'imu_stamp': self.latest_imu_stamp,
            **wheel_cols,
        }

    def _record_callback(self, msg: Bool):
        if self.logging_enabled != msg.data:
            self.logging_enabled = msg.data
            self.get_logger().info(f"【データ保存】保存状態が切り替わりました: {'[記録中]' if self.logging_enabled else '[一時停止]'}")

    def _mode_callback(self, msg: String):
        self.mode_stamp = self._now()
        if self.current_mode != msg.data:
            self.current_mode = msg.data
            if msg.data == "auto":
                self.get_logger().info("【データ保存】AUTO走行中はモデル自身の出力が混入するため記録しません。")

    def _skip(self, reason):
        if self.skip_reason != reason and self.verbose >= 1:
            self.get_logger().info(f"【データ保存】記録を停止中: {reason}")
        self.skip_reason = reason

    def _convert_msg_to_numpy(self, msg: Image) -> np.ndarray:
        h, w, c = msg.height, msg.width, 3
        step = int(msg.step)
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        bytes_per_pixel = c
        if step == w * bytes_per_pixel:
            frame = buf.reshape((h, w, c))
        else:
            row_view = buf.reshape((h, step))
            useful = row_view[:, : (w * c)]
            frame = useful.reshape((h, w, c))
        return frame

    def _image_callback(self, msg: Image):
        now = self._now()

        try:
            frame = self._convert_msg_to_numpy(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image msg: {e}")
            return

        # A live camera always has sensor noise, so a bit-identical frame means the feed is frozen
        frozen = self.prev_frame is not None and np.array_equal(frame, self.prev_frame)
        self.prev_frame = frame

        # 0. Check if data logging is enabled
        if not self.logging_enabled:
            self._skip("OPTIONSボタンで一時停止中")
            return

        # 1. Only record while a human is driving
        if self.current_mode is None or now - self.mode_stamp > 1.0:
            self._skip("走行モードが不明（MUXノードからの状態が届いていません）")
            return
        if self.current_mode != "manual":
            self._skip("AUTO走行中は記録しません")
            return

        # 2. Controls (optional in "moving" mode): stale values would be a dead node's last value, so use NaN
        fresh = self.latest_controls is not None and now - self.latest_controls_stamp <= self.max_control_age
        if self.record_when != "moving" and not fresh:
            self._skip("操作量が届いていません（コントローラー系の停止）")
            return

        if frozen:
            self._skip("カメラ画像が更新されていません（フリーズの可能性）")
            return

        # 3. Extract steering, throttle and brake (channel 0, 1, 2); NaN without a fresh controller command
        nan = float('nan')
        steering, throttle, brake = (0.0, 0.0, 0.0) if fresh else (nan, nan, nan)
        if fresh:
            for channel_data in self.latest_controls.data:
                if channel_data.channel == 0:
                    steering = channel_data.normalized
                elif channel_data.channel == 1:
                    throttle = channel_data.normalized
                elif channel_data.channel == 2:
                    brake = channel_data.normalized

        if self.record_when == "moving":
            # 4. Record while the car moves (any way of driving: pedal, controller, coasting, braking) and for
            #    stop_hold_sec after it stopped. Reversing is not learned (the path labels exclude it).
            moving, reversing, reason = self._motion(now)
            if moving is None:
                self._skip(reason)
                return
            if reversing or (fresh and throttle < 0.0):
                self._skip("後退中は記録しません")
                return
            if not moving:
                self._skip("停止中")
                return
        else:
            # 4. Reverse is not learned (the throttle head is 0..1), so reverse frames would only be label noise
            if throttle < 0.0:
                self._skip("後退中は記録しません")
                return
            # 5. Check throttle threshold
            if self.save_on_throttle_only and abs(throttle) < self.min_throttle_threshold:
                self._skip("スロットルが閾値未満（停止中）")
                return

        # 6. Append to buffer
        image_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.buf_images.append(frame)
        self.buf_steerings.append(steering)
        self.buf_throttles.append(throttle)
        self.buf_brakes.append(brake)
        self.buf_image_stamps.append(image_stamp)
        self.buf_control_stamps.append(self.latest_controls_stamp if fresh else float('nan'))
        if self.first_frame_stamp is None:
            self.first_frame_stamp = now
        for name, value in self._imu_row(now).items():
            self.buf_imu[name].append(value)
        self.last_saved_stamp = now
        if self.skip_reason:
            self.get_logger().info("【データ保存】記録を再開しました。")
            self.skip_reason = ""

        if self.verbose >= 2:
            self.get_logger().info(f"Buffered frame {len(self.buf_images)}/{self.buffer_size}. Steering: {steering:.2f}, Throttle: {throttle:.2f}")

        # 7. Flush if buffer is full
        if len(self.buf_images) >= self.buffer_size:
            self._write_buffer()

    def _publish_status(self):
        recording = (self._now() - self.last_saved_stamp) < 1.0
        status = {
            "enabled": self.logging_enabled,
            "recording": recording,
            "reason": "" if recording else self.skip_reason,
            "frames": self.saved_frames + len(self.buf_images),
            "file": self.h5_filename,
        }
        msg = String()
        msg.data = json.dumps(status, ensure_ascii=False)
        self.status_pub.publish(msg)

        health = String()
        health.data = f"save:{'RECORDING' if recording else 'IDLE'}"
        self.health_pub.publish(health)

    def _write_buffer(self):
        if not self.buf_images:
            return

        try:
            if self.h5_file is None:
                h, w, c = self.buf_images[0].shape
                self.h5_file = h5py.File(self.h5_filepath, 'a')
                self.h5_file.attrs['color_order'] = 'BGR'
                self.h5_file.attrs['format_version'] = H5_FORMAT_VERSION
                self.h5_file.attrs['wheel_circumference_m'] = self.wheel_circumference
                self.h5_file.attrs['wheel_pulses_per_rev'] = self.wheel_pulses_per_rev
                self.h5_file.attrs['record_when'] = self.record_when
                for key, value in {**self.camera_attrs, **self.imu_attrs}.items():
                    self.h5_file.attrs[key] = value
                if 'image' not in self.h5_file:
                    self.h5_file.create_dataset('image', shape=(0, h, w, c), maxshape=(None, h, w, c), dtype='uint8', chunks=(1, h, w, c))
                for name, dtype in (('steering', 'float32'), ('throttle', 'float32'), ('brake', 'float32'),
                                    ('image_stamp', 'float64'), ('control_stamp', 'float64')):
                    if name not in self.h5_file:
                        self.h5_file.create_dataset(name, shape=(0,), maxshape=(None,), dtype=dtype, chunks=(100,))
                for name, width in IMU_COLUMNS.items():
                    if name not in self.h5_file:
                        dtype = 'float64' if name in ('gps', 'imu_stamp', 'wheel_pulse_count') else 'float32'
                        shape = (0,) if width == 1 else (0, width)
                        self.h5_file.create_dataset(name, shape=shape, maxshape=(None,) + shape[1:], dtype=dtype,
                                                    chunks=(100,) + shape[1:])

            B = len(self.buf_images)
            curr_size = self.h5_file['image'].shape[0]
            new_size = curr_size + B

            columns = {
                'image': self.buf_images,
                'steering': self.buf_steerings,
                'throttle': self.buf_throttles,
                'brake': self.buf_brakes,
                'image_stamp': self.buf_image_stamps,
                'control_stamp': self.buf_control_stamps,
                **self.buf_imu,
            }
            for name, values in columns.items():
                self.h5_file[name].resize(new_size, axis=0)
                self.h5_file[name][curr_size:new_size] = np.array(values)
            self.h5_file.flush()
            self.saved_frames = new_size

            self.get_logger().info(f"【データ保存】{B}フレームをHDF5ファイルへ書き込みました。(累計保存フレーム数: {new_size})")

            for values in columns.values():
                values.clear()
        except Exception as e:
            self.get_logger().error(f"Error writing to HDF5 file: {e}")
            return
        self._write_streams()

    def _flush_and_close(self):
        if self.buf_images:
            self.get_logger().info("【データ保存】終了前にバッファ内の残りのフレームを書き込んでいます...")
            self._write_buffer()
        if self.h5_file is not None:
            self._write_streams()
            totals = ", ".join(f"{g} {n}" for g, n in self.stream_totals.items())
            self.get_logger().info(f"【データ保存】高頻度データの記録数: {totals}")
            self.h5_file.close()
            self.h5_file = None
            self.get_logger().info("【データ保存】HDF5ファイルをクローズしました。")

    def destroy_node(self):
        self._flush_and_close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SaveNode()
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
