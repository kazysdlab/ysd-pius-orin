#!/usr/bin/env python3
import os
import json
import argparse
import numpy as np
import h5py
from PIL import Image

def convert_donkey_to_h5(data_dir, output_file, batch_size=100):
    print(f"=== Donkey Car dataset to PIUS HDF5 Converter ===")
    print(f"Input Directory: {data_dir}")
    print(f"Output HDF5 Path: {output_file}")
    
    # 1. Read manifest.json
    manifest_path = os.path.join(data_dir, 'manifest.json')
    if not os.path.exists(manifest_path):
        raise FileNotFoundError(f"manifest.json not found in {data_dir}")
        
    catalog_paths = []
    with open(manifest_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                data = json.loads(line)
                if isinstance(data, dict) and 'paths' in data:
                    catalog_paths = data['paths']
                    break
            except Exception:
                pass
    if not catalog_paths:
        # Some Donkey Car versions store catalog names differently, check directory for catalog files
        catalog_paths = [f for f in os.listdir(data_dir) if f.startswith('catalog_') and f.endswith('.catalog')]
        
    print(f"Found {len(catalog_paths)} catalog files.")
    
    # Make sure output directory exists
    os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)
    
    # Track statistics
    total_records = 0
    skipped_records = 0
    
    # Buffers to write in batches
    buf_images = []
    buf_steerings = []
    buf_throttles = []
    
    # Open HDF5 File
    with h5py.File(output_file, 'w') as h5_file:
        # Same channel order as the Jetson camera / save_node, so both sources can be mixed for training
        h5_file.attrs['color_order'] = 'BGR'
        h5_file.attrs['source'] = 'donkeycar'

        # Create datasets with resize support (first dimension is infinite)
        h5_image = h5_file.create_dataset('image', shape=(0, 120, 160, 3), maxshape=(None, 120, 160, 3), dtype='uint8', chunks=(1, 120, 160, 3))
        h5_steering = h5_file.create_dataset('steering', shape=(0,), maxshape=(None,), dtype='float32', chunks=(100,))
        h5_throttle = h5_file.create_dataset('throttle', shape=(0,), maxshape=(None,), dtype='float32', chunks=(100,))
        
        def flush_buffers():
            nonlocal buf_images, buf_steerings, buf_throttles
            if not buf_images:
                return
            
            b_size = len(buf_images)
            curr_size = h5_image.shape[0]
            new_size = curr_size + b_size
            
            # Resize datasets
            h5_image.resize(new_size, axis=0)
            h5_steering.resize(new_size, axis=0)
            h5_throttle.resize(new_size, axis=0)
            
            # Write buffers
            h5_image[curr_size:new_size] = np.array(buf_images, dtype='uint8')
            h5_steering[curr_size:new_size] = np.array(buf_steerings, dtype='float32')
            h5_throttle[curr_size:new_size] = np.array(buf_throttles, dtype='float32')
            
            # Clear buffers
            buf_images.clear()
            buf_steerings.clear()
            buf_throttles.clear()
            
        # 2. Iterate through catalog files
        for cat_name in sorted(catalog_paths):
            cat_path = os.path.join(data_dir, cat_name)
            if not os.path.exists(cat_path):
                print(f"Warning: Catalog file {cat_name} not found. Skipping.")
                continue
                
            print(f"Processing catalog: {cat_name}...")
            with open(cat_path, 'r') as f:
                for line in f:
                    try:
                        record = json.loads(line.strip())
                        
                        # Extract steering and throttle
                        # Donkey car uses "user/angle" and "user/throttle"
                        steering = record.get('user/angle', 0.0)
                        throttle = record.get('user/throttle', 0.0)
                        
                        # Extract image file name
                        img_filename = record.get('cam/image_array')
                        if not img_filename:
                            skipped_records += 1
                            continue
                            
                        # Load image
                        # Donkey car paths are sometimes relative to data directory or just filenames
                        img_path = os.path.join(data_dir, img_filename)
                        if not os.path.exists(img_path):
                            # Try under "images/" subdirectory specifically
                            img_path = os.path.join(data_dir, 'images', img_filename)
                            
                        if not os.path.exists(img_path):
                            # Try extracting just basename
                            img_path = os.path.join(data_dir, 'images', os.path.basename(img_filename))
                            
                        if not os.path.exists(img_path):
                            print(f"Warning: Image file not found: {img_filename}. Skipping record.")
                            skipped_records += 1
                            continue
                            
                        # Load image and convert to numpy BGR (120, 160, 3)
                        with Image.open(img_path) as img:
                            img_rgb = img.convert('RGB')
                            # Safety check: if image is not 160x120, resize it
                            if img_rgb.size != (160, 120):
                                img_rgb = img_rgb.resize((160, 120), Image.Resampling.BILINEAR)
                            img_arr = np.ascontiguousarray(np.array(img_rgb, dtype=np.uint8)[:, :, ::-1])
                            
                        # Append to buffers
                        buf_images.append(img_arr)
                        buf_steerings.append(steering)
                        buf_throttles.append(throttle)
                        total_records += 1
                        
                        # Flush if buffer is full
                        if len(buf_images) >= batch_size:
                            flush_buffers()
                            
                    except Exception as e:
                        print(f"Error parsing record on line in {cat_name}: {e}")
                        skipped_records += 1
                        
        # Final flush for remaining items
        flush_buffers()
        
    print(f"\nConversion completed successfully!")
    print(f"Total frames converted: {total_records}")
    print(f"Skipped frames: {skipped_records}")
    print(f"HDF5 dataset saved at: {output_file}")

def main():
    parser = argparse.ArgumentParser(description="Convert Donkey Car datasets to PIUS HDF5 format")
    parser.add_argument('--data_dir', type=str, default='/home/yl00/Downloads/mycar-20260625T025025Z-3-001/mycar/data', help='Path to donkey car data directory')
    parser.add_argument('--output', type=str, default='/home/yl00/pius/data/donkey_converted.h5', help='Path to output HDF5 file')
    parser.add_argument('--batch', type=int, default=100, help='Batch write size')
    args = parser.parse_args()
    
    convert_donkey_to_h5(args.data_dir, args.output, args.batch)

if __name__ == '__main__':
    main()
