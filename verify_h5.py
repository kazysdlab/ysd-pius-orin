import h5py
import os
import sys
import glob
import numpy as np
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'src', 'pius_control', 'pius_control'))
try:
    from waypoint_labels import imu_mounting, load_h5, make_labels, reconstruct
    from calibration import load_config
    CONFIG = load_config(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config.yaml'))
except ImportError:
    reconstruct = None


def label_summary(filepath):
    """Share of frames that get waypoint ground truth (>= 3 of 5 points), or None without IMU / wheel data."""
    if reconstruct is None:
        return None
    try:
        data = load_h5(filepath)
    except Exception:
        return None
    if not data['circumference'] or data['frame_t'] is None or len(data['imu_t']) < 2:
        return {'ratio': 0.0, 'reason': 'タイヤ外周または IMU の記録がありません'}
    axis, sign, _ = imu_mounting(data, CONFIG)
    traj = reconstruct(data['imu_t'], data['acc'], data['gyro'], data['pulse_t'], data['circumference'],
                       forward_axis=axis, yaw_sign=sign, pulses_per_rev=data['pulses_per_rev'])
    _, ok = make_labels(traj, data['frame_t'], 5, 1.0, max_time=30.0)
    return {'ratio': float((ok.sum(axis=1) >= 3).mean()) if len(ok) else 0.0,
            'accepted': len(traj['accepted_pulses']), 'pulses': traj['n_pulses'],
            'distance': float(traj['arc'][-1]) if len(traj['arc']) else 0.0}

def control_lag(image_stamps, control_stamps):
    """Largest gap between a frame and its controller command [s] (frames without a command are skipped)."""
    lag = np.abs(image_stamps - control_stamps)
    lag = lag[np.isfinite(lag)]
    return float(np.max(lag)) if len(lag) else None


def control_stats(prefix, values):
    """min / max / std / mean of the frames that have a controller command (None when there is none)."""
    v = values[np.isfinite(values)]
    if len(v) == 0:
        return {f'{prefix}_{k}': None for k in ('min', 'max', 'std', 'mean')}
    return {f'{prefix}_min': np.min(v), f'{prefix}_max': np.max(v), f'{prefix}_std': np.std(v), f'{prefix}_mean': np.mean(v)}


def report_controls(report):
    print(f"【ステアリング操作データチェック】（指令があるフレームのみ）")
    print(f"  ・出力範囲 (最小/最大) : {report['st_min']:+.2f} / {report['st_max']:+.2f}")
    print(f"  ・平均操作量           : {report['st_mean']:+.2f}")
    if report['st_std'] < 0.01:
        print("  => 警告: ステアリングの操作値が完全に一定です。 ⚠️")
    else:
        print("  => 状態: 正常 (操舵変化が記録されています)  [OK]")
    print("-" * 60)

    # Throttle Check
    print(f"【スロットル加減速データチェック】")
    print(f"  ・出力範囲 (最小/最大) : {report['th_min']:+.2f} / {report['th_max']:+.2f}")
    print(f"  ・平均操作量           : {report['th_mean']:+.2f}")
    if report['th_std'] < 0.01:
        print("  => 警告: スロットルの操作値が完全に一定です。 ⚠️")
    else:
        print("  => 状態: 正常 (加減速変化が記録されています)  [OK]")


def control_warnings(report):
    """Constant steering / throttle in the frames that have a controller command."""
    return int(report['st_std'] < 0.01) + int(report['th_std'] < 0.01)


def count_duplicate_frames(images, block=500):
    """Counts frames that are bit-identical to the previous one (a live camera always has noise, so these indicate a frozen feed)."""
    duplicates = 0
    prev_last = None
    for start in range(0, len(images), block):
        chunk = images[start:start + block]
        flat = chunk.reshape(len(chunk), -1)
        duplicates += int(np.sum(np.all(flat[1:] == flat[:-1], axis=1)))
        if prev_last is not None and np.array_equal(prev_last, flat[0]):
            duplicates += 1
        prev_last = flat[-1]
    return duplicates

def imu_summary(f):
    """Share of frames with IMU values and with a GPS fix (format_version >= 3), else None."""
    if 'imu_euler' not in f:
        return None
    euler = f['imu_euler'][:]
    fix = f['gps_fix'][:] if 'gps_fix' in f else np.zeros(len(euler))
    n = max(len(euler), 1)
    return {
        'imu_ratio': float(np.sum(~np.isnan(euler[:, 0]))) / n,
        'fix_ratio': float(np.sum(fix == 1.0)) / n,
    }

def stream_summary(f):
    """High-rate streams (format_version >= 4): samples, duration and median rate per stream."""
    out = {}
    for group, stamp in (('imu_hr', 'stamp'), ('gps_hr', 'stamp'), ('wheel', 'pulse_stamp')):
        if group in f and stamp in f[group] and len(f[group][stamp]) > 0:
            s = f[group][stamp][:]
            dt = np.diff(s)
            out[group] = {'n': len(s), 'span': float(s[-1] - s[0]),
                          'rate': float(1.0 / np.median(dt)) if len(dt) and np.median(dt) > 0 else 0.0}
    return out

def analyze_file(filepath):
    """Analyzes a single HDF5 file and returns a dictionary of stats."""
    file_size_mb = os.path.getsize(filepath) / (1024 * 1024)
    try:
        with h5py.File(filepath, 'r') as f:
            required_keys = ['image', 'steering', 'throttle']
            missing_keys = [k for k in required_keys if k not in f]
            if missing_keys:
                return {
                    'status': 'error',
                    'error': f'必要なキーが不足しています: {missing_keys}',
                    'filename': os.path.basename(filepath),
                    'filepath': filepath,
                    'size_mb': file_size_mb
                }
            
            images = f['image']
            steerings = f['steering'][:]
            throttles = f['throttle'][:]
            num_frames = len(steerings)
            
            if num_frames == 0:
                return {
                    'status': 'error',
                    'error': 'データ数が0フレームです。',
                    'filename': os.path.basename(filepath),
                    'filepath': filepath,
                    'size_mb': file_size_mb
                }
                
            color_order = f.attrs.get('color_order', None)
            if isinstance(color_order, bytes):
                color_order = color_order.decode()

            duration = num_frames / 5.0
            gaps = None
            if 'image_stamp' in f and num_frames > 1:
                stamps = f['image_stamp'][:]
                intervals = np.diff(stamps)
                duration = float(np.sum(intervals[intervals <= 0.5]))
                gaps = {
                    'median_interval': float(np.median(intervals)),
                    'count': int(np.sum(intervals > 0.5)),
                    'max': float(np.max(intervals)),
                    'max_control_lag': control_lag(stamps, f['control_stamp'][:]) if 'control_stamp' in f else None,
                }

            return {
                'status': 'ok',
                'filename': os.path.basename(filepath),
                'filepath': filepath,
                'size_mb': file_size_mb,
                'frames': num_frames,
                'duration': duration,
                'color_order': color_order,
                'format_version': int(f.attrs.get('format_version', 0)),
                'camera_crop': list(f.attrs.get('camera_crop', [])),
                'gaps': gaps,
                'duplicates': count_duplicate_frames(images),
                'imu': imu_summary(f),
                'streams': stream_summary(f),
                'labels': label_summary(filepath) if int(f.attrs.get('format_version', 0)) >= 4 else None,
                'img_shape': images.shape,
                # Format v6 records frames without a controller command too (pedal driving): NaN there
                'ctrl_ratio': float(np.isfinite(steerings).mean()),
                **control_stats('st', steerings),
                **control_stats('th', throttles),
            }
    except Exception as e:
        return {
            'status': 'error',
            'error': str(e),
            'filename': os.path.basename(filepath),
            'filepath': filepath,
            'size_mb': file_size_mb
        }

def print_detail_report(report):
    """Prints a detailed diagnosis report for a single file."""
    print("=" * 60)
    print("      PIUS 走行データ詳細診断レポート")
    print("=" * 60)
    print(f"■ 対象ファイル : {report['filename']}")
    print(f"■ ファイルサイズ: {report['size_mb']:.2f} MB")
    print("-" * 60)
    
    if report['status'] == 'error':
        print(f"【判定】データ破損 ❌ (エラー: {report['error']})")
        print("=" * 60)
        return False
        
    print(f"■ 総記録フレーム数 : {report['frames']} フレーム")
    print(f"■ 推定記録時間     : {report['duration']:.1f} 秒")
    print("-" * 60)
    
    # Image Check
    img_shape = report['img_shape']
    print(f"【画像データチェック】")
    print(f"  ・解像度: {img_shape[2]}x{img_shape[1]} (幅x高)")
    color_order = report['color_order']
    color_text = color_order if color_order else "記録なし（旧形式。実機収集データはBGR、DonkeyCar変換データはRGBの可能性）"
    print(f"  ・チャンネル数: {img_shape[3]} (色の並び: {color_text})")
    print(f"  ・画像総数: {img_shape[0]}枚")
    warnings = 0
    crop = report.get('camera_crop') or []
    print(f"  ・記録形式: v{report.get('format_version', 0)}" + (f"（カメラ 1280×720 のうち [x, y, 幅, 高さ] = {crop} を使用）" if crop else ""))
    if img_shape[1] == 128 and img_shape[2] == 320:
        print("  => 状態: 正常 (waypoint モデルの入力 320x128 に合致しています)  [OK]")
    else:
        print(f"  => 警告: 画像サイズが {img_shape[2]}x{img_shape[1]} です。waypoint モデル（320x128）の学習には使えません。")
        warnings += 1
    if report['duplicates'] > 0:
        print(f"  => 警告: 直前と完全に同一の画像が {report['duplicates']} フレームあります。カメラのフリーズ中に記録された可能性があります。 ⚠️")
        warnings += 1
    else:
        print("  => 状態: 正常 (画像のフリーズは検出されませんでした)  [OK]")
    print("-" * 60)

    # Timing Check
    print(f"【時系列チェック】")
    gaps = report['gaps']
    if gaps is None:
        print("  ・タイムスタンプ: 記録なし（旧形式のため、フレームの連続性は確認できません）")
    else:
        print(f"  ・フレーム間隔の中央値     : {gaps['median_interval'] * 1000:.0f} ms")
        print(f"  ・0.5秒を超える途切れ      : {gaps['count']} 箇所 (最大 {gaps['max']:.1f} 秒)")
        if gaps['max_control_lag'] is not None:
            print(f"  ・画像と操作量の時刻差(最大): {gaps['max_control_lag'] * 1000:.0f} ms")
        print("  => 途切れ（記録一時停止・停止中など）をまたぐペアは学習時に自動で除外されます  [OK]")
    print("-" * 60)

    # IMU / GPS
    print(f"【IMU/GPSチェック】")
    imu = report['imu']
    if imu is None:
        print("  ・IMU/GPSの記録なし（フォーマットバージョン2以前）")
    else:
        print(f"  ・IMUの値があるフレーム : {imu['imu_ratio'] * 100:.1f} %（未接続・値が古い間は NaN）")
        print(f"  ・GPS測位中のフレーム   : {imu['fix_ratio'] * 100:.1f} %")
    streams = report['streams']
    names = {'imu_hr': 'IMU 全サンプル', 'gps_hr': 'GPS', 'wheel': '車輪パルス'}
    if streams:
        for g, s in streams.items():
            rate = f"、中央値 {s['rate']:.1f} Hz" if g != 'wheel' else ""
            print(f"  ・{names[g]:12s}: {s['n']} 件（{s['span']:.0f} 秒分{rate}）")
    else:
        print("  ・高頻度データ（IMU 全サンプル・車輪パルス）の記録なし（フォーマットバージョン3以前、またはセンサー未接続）")
    labels = report.get('labels')
    if labels is not None:
        if 'reason' in labels:
            print(f"  => 警告: waypoint の正解を作れません（{labels['reason']}） ⚠️")
            warnings += 1
        else:
            print(f"  ・waypoint の正解があるフレーム: {labels['ratio'] * 100:.1f} %"
                  f"（車輪パルス採用 {labels['accepted']}/{labels['pulses']}、復元した走行距離 {labels['distance']:.0f} m）")
            if labels['ratio'] < 0.5:
                print("  => 警告: 正解を作れるフレームが半分未満です（車速センサー・IMU の接続、外周の設定を確認） ⚠️")
                warnings += 1
    print("-" * 60)

    # Controller commands (format v6 also records pedal driving without them; waypoint training does not need them)
    print(f"【操作量（コントローラーの指令）】 指令があるフレーム: {report['ctrl_ratio'] * 100:.0f} %")
    if report['st_std'] is None:
        print("  => 操作量なし（ペダル運転など）。waypoint モデルの学習には不要です  [OK]")
    else:
        report_controls(report)
        warnings += control_warnings(report)
    print("=" * 60)

    # Comprehensive advice
    print("【総合診断とアドバイス】")
    if report['frames'] < 300:
        print("  💡 アドバイス: データ量が極端に少ないです（20秒未満）。")
        print("     安定した自動運転モデルを育てるには、数分間分（目安：1000〜3000フレーム以上）の走行データを収集することをお勧めします。")
        warnings += 1
        
    if warnings == 0:
        print("  ✨ 判定: 正常データです！ ❌ -> ⭕")
        print("     画像と操縦データが正しく同期して記録されています。このまま AI学習（train_waypoint.py）へ進んでください！")
        return True
    else:
        print("  ⚠️ 判定: 一部警告があります。上記のアドバイスを確認し、必要に応じて再測定を行ってください。")
        return False

def verify_all_files(data_dir):
    """Verifies all HDF5 files in the directory and shows a summary table."""
    h5_files = glob.glob(os.path.join(data_dir, '*.h5'))
    if not h5_files:
        print("=" * 60)
        print("【エラー】データフォルダ内に HDF5ファイル (.h5) が見つかりません。")
        print("=" * 60)
        return
        
    print("=" * 80)
    print("                 PIUS 走行データセット全体診断レポート")
    print("=" * 80)
    print(f"■ 対象ディレクトリ : {data_dir}")
    print(f"■ 検出ファイル数   : {len(h5_files)} ファイル")
    print("-" * 80)
    print(f"{'ファイル名':<35} | {'サイズ':<8} | {'フレーム数':<8} | {'時間(秒)':<8} | {'診断結果':<10}")
    print("-" * 80)
    
    total_frames = 0
    total_size_mb = 0.0
    ok_count = 0
    err_count = 0
    warn_count = 0
    
    all_reports = []
    
    for filepath in sorted(h5_files):
        report = analyze_file(filepath)
        all_reports.append(report)
        filename_truncated = report['filename'][:35]
        
        if report['status'] == 'error':
            status_str = "❌ 破損/エラー"
            err_count += 1
            print(f"{filename_truncated:<35} | {report['size_mb']:6.2f}MB | {'-':^8} | {'-':^8} | {status_str}")
        else:
            total_frames += report['frames']
            total_size_mb += report['size_mb']
            
            # Simple check for individual warning
            constant = report['st_std'] is not None and (report['st_std'] < 0.01 or report['th_std'] < 0.01)
            has_warning = (report['frames'] < 300) or constant or (report['duplicates'] > 0)
            if report['duplicates'] > 0:
                status_str = f"⚠️ 同一画像{report['duplicates']}枚"
                warn_count += 1
            elif has_warning:
                status_str = "⚠️ 警告有り"
                warn_count += 1
            else:
                status_str = "⭕ 正常"
                ok_count += 1
                
            print(f"{filename_truncated:<35} | {report['size_mb']:6.2f}MB | {report['frames']:8d} | {report['duration']:8.1f} | {status_str}")
            
    print("-" * 80)
    print("【データセット全体集計結果】")
    print(f"  ・合計ファイルサイズ : {total_size_mb:.2f} MB")
    print(f"  ・合計フレーム数     : {total_frames} フレーム")
    print(f"  ・合計推定走行時間   : {sum(r['duration'] for r in all_reports if r['status'] == 'ok'):.1f} 秒")
    print(f"  ・ステータス内訳     : 正常={ok_count}, 警告={warn_count}, エラー={err_count}")
    print("=" * 80)
    
    print("【総合評価とアドバイス】")
    if err_count > 0:
        print("  ❌ 致命的: 破損したファイルが含まれています。破損ファイルはデータフォルダから除外してください。")
    elif total_frames < 1000:
        print("  💡 アドバイス: データセット全体の合計データ量が少ないです（目安1分未満）。")
        print("     AIがコース形状を安定して覚えるには、合計1000〜3000フレーム以上の多様な走行データを集めることを強く推奨します。")
    else:
        print("  ✨ 判定: 学習準備完了！ ❌ -> ⭕")
        print("     蓄積された複数の走行データが正しくマージされ、AI学習（train_waypoint.py）へ送られます！")
    print("=" * 80)

def main():
    parser = argparse.ArgumentParser(description="PIUS HDF5走行データ検証ツール")
    parser.add_argument('--dir', type=str, default=None, help='データディレクトリのパス')
    parser.add_argument('--all', action='store_true', help='データフォルダ内のすべてのファイルを確認し、集計レポートを出力する')
    parser.add_argument('--file', type=str, default=None, help='特定の HDF5ファイル (.h5) を指定して詳細チェックを行う')
    args = parser.parse_args()
    
    # Setup data directory
    data_dir = args.dir
    if not data_dir:
        data_dir = '/home/orin/dev/ws/colcon_ws/data'
        if not os.path.exists(data_dir):
            data_dir = '/home/yl00/pius/data'
            
    if args.file:
        # Check specific file
        filepath = args.file
        if not os.path.exists(filepath):
            # Try searching in the data dir
            filepath = os.path.join(data_dir, args.file)
            
        if not os.path.exists(filepath):
            print(f"【エラー】指定されたファイルが見つかりません: {args.file}")
            return
            
        report = analyze_file(filepath)
        print_detail_report(report)
        
    elif args.all:
        # Check all files
        verify_all_files(data_dir)
        
    else:
        # Default: Verify latest file
        files = glob.glob(os.path.join(data_dir, '*.h5'))
        if not files:
            print("=" * 60)
            print("【エラー】データフォルダ内に HDF5ファイル (.h5) が見つかりません。")
            print("=" * 60)
            return
        latest_file = max(files, key=os.path.getctime)
        report = analyze_file(latest_file)
        print_detail_report(report)

if __name__ == '__main__':
    main()
