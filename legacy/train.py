import os
import glob
import h5py
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import torchvision.models as models
import torchvision.transforms.functional as TF
import random
import argparse

# ---------------------------------------------------------
# 1. Model Definition (CNN + LSTM)
# ---------------------------------------------------------
class PiusModel(nn.Module):
    def __init__(self):
        super(PiusModel, self).__init__()
        
        # Backbone: ResNet-18 (Pretrained on ImageNet)
        resnet = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
        # Remove the final classification layer to get feature vectors
        self.backbone = nn.Sequential(*list(resnet.children())[:-1])
        cnn_out_dim = 512
        
        # Regression Heads (MLP)
        self.fc_steering = nn.Sequential(
            nn.Linear(cnn_out_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 1),
            nn.Tanh()  # Output range: [-1.0, 1.0]
        )
        
        self.fc_throttle = nn.Sequential(
            nn.Linear(cnn_out_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 1),
            nn.Sigmoid()  # Output range: [0.0, 1.0]
        )

    def forward(self, x):
        # x shape: [batch, C, H, W]
        features = self.backbone(x) # shape: [batch, 512, 1, 1]
        features = features.view(x.size(0), -1) # shape: [batch, 512]
        
        # Regression Heads
        steering = self.fc_steering(features)
        throttle = self.fc_throttle(features)
        
        # Concatenate outputs: shape [batch, 2]
        return torch.cat((steering, throttle), dim=1)

# ---------------------------------------------------------
# 2. Dataset and Data Augmentation (Lazy Loading Version)
# ---------------------------------------------------------
class DrivingDataset(Dataset):
    def __init__(self, mappings, shift=1, transform=False):
        """
        Args:
            mappings: List of tuples (filepath, img_idx, is_rgb)
            shift: Number of frames to shift the label forward (for predicting future action)
            transform: Boolean, whether to apply data augmentation
        """
        self.mappings = mappings
        self.shift = shift
        self.transform = transform

    def __len__(self):
        return len(self.mappings)

    def _augment_image(self, img_tensor, steering_val):
        """
        Applies random transforms to the image.
        """
        # 1. Random Horizontal Flip
        if random.random() > 0.5:
            img_tensor = TF.hflip(img_tensor)
            steering_val = -steering_val  # Flip steering angle!

        # 2. Random Affine (Translation only, simulate track shift)
        if random.random() > 0.5:
            max_translate_x = int(0.1 * img_tensor.shape[2]) # 10% width
            translate_x = random.randint(-max_translate_x, max_translate_x)
            img_tensor = TF.affine(img_tensor, angle=0, translate=[translate_x, 0], scale=1.0, shear=0)

        # 3. Color Jitter (Brightness, Contrast)
        if random.random() > 0.5:
            brightness_factor = random.uniform(0.7, 1.3)
            contrast_factor = random.uniform(0.7, 1.3)
            img_tensor = TF.adjust_brightness(img_tensor, brightness_factor)
            img_tensor = TF.adjust_contrast(img_tensor, contrast_factor)

        return img_tensor, steering_val

    def __getitem__(self, idx):
        filepath, img_idx, is_rgb = self.mappings[idx]

        # Open HDF5 on the fly. This is safe for multiprocessing (num_workers > 0).
        with h5py.File(filepath, 'r') as f:
            # 1. Extract single image [H, W, C]. The camera (and inference) uses BGR.
            img = f['image'][img_idx]
            if is_rgb:
                img = img[:, :, ::-1]
            
            # 2. Extract label (shifted forward)
            label_idx = img_idx + self.shift
            steering = float(f['steering'][label_idx])
            throttle = float(f['throttle'][label_idx])
            
        # 3. Convert to PyTorch tensor [C, H, W] in [0.0, 1.0] range
        img_chw = np.ascontiguousarray(img.transpose(2, 0, 1)) # HWC to CHW
        tensor_img = torch.tensor(img_chw, dtype=torch.float32) / 255.0
            
        # 4. Apply Augmentations (only during training)
        if self.transform:
            tensor_img, steering = self._augment_image(tensor_img, steering)
            
        target = torch.tensor([steering, throttle], dtype=torch.float32)
        return tensor_img, target

# ---------------------------------------------------------
# 3. Data Scan & Index Mapping (Lazy Loading Setup)
# ---------------------------------------------------------
def prepare_chunks(data_dir, shift=1, chunk_len=100, max_gap=0.5):
    """
    Split every file into contiguous chunks of chunk_len frames. Train/val are split per chunk
    (not per frame) so that nearly identical neighbouring frames never end up on both sides.

    Returns a list of chunks, each a list of (filepath, img_idx, is_rgb). A sample is kept only if
    its label frame (img_idx + shift) lies in the same chunk and, when timestamps are available
    (format_version >= 2), no gap between the two frames exceeds max_gap seconds.
    """
    h5_files = sorted(glob.glob(os.path.join(data_dir, '*.h5')))
    if not h5_files:
        raise ValueError(f"No HDF5 files found in data directory: {data_dir}")

    print(f"Found {len(h5_files)} HDF5 files. Mapping datasets...")

    chunks = []
    total_candidates = 0
    total_gap_dropped = 0

    for filepath in h5_files:
        with h5py.File(filepath, 'r') as f:
            if not ('image' in f and 'steering' in f and 'throttle' in f):
                print(f"Warning: Datasets missing in {filepath}. Skipping.")
                continue

            num_samples = len(f['image'])
            color_order = f.attrs.get('color_order', None)
            if isinstance(color_order, bytes):
                color_order = color_order.decode()
            is_rgb = color_order == 'RGB'

            valid = np.ones(max(num_samples - shift, 0), dtype=bool)
            if 'image_stamp' in f and len(valid) > 0:
                stamps = f['image_stamp'][:]
                big_gap = np.diff(stamps) > max_gap
                for k in range(shift):
                    valid &= ~big_gap[k:k + len(valid)]
            else:
                stamps = None

            note = []
            if color_order is None:
                note.append("color_order属性なし→BGRとして扱います（DonkeyCar変換データならconvert_donkey_to_h5.pyで再変換してください）")
            elif is_rgb:
                note.append("RGB→BGRに変換して読み込みます")
            if stamps is None:
                note.append("タイムスタンプなし（旧形式）→フレームの連続性チェックなし")

            file_candidates = 0
            file_kept = 0
            for start in range(0, num_samples, chunk_len):
                end = min(start + chunk_len, num_samples)
                chunk = [(filepath, i, is_rgb) for i in range(start, end - shift) if valid[i]]
                file_candidates += max(end - shift - start, 0)
                file_kept += len(chunk)
                if chunk:
                    chunks.append(chunk)
            total_candidates += file_candidates
            total_gap_dropped += file_candidates - file_kept
            print(f"  {os.path.basename(filepath)}: {file_kept} samples" + (f" ({' / '.join(note)})" if note else ""))

    if total_gap_dropped:
        print(f"Excluded {total_gap_dropped} of {total_candidates} samples whose next frame was not continuous (gap > {max_gap}s).")
    return chunks


def split_chunks(chunks, val_ratio=0.2, seed=42):
    rng = random.Random(seed)
    order = list(range(len(chunks)))
    rng.shuffle(order)
    total = sum(len(c) for c in chunks)
    target_val = total * val_ratio
    train_mappings, val_mappings = [], []
    for idx in order:
        if len(val_mappings) < target_val:
            val_mappings.extend(chunks[idx])
        else:
            train_mappings.extend(chunks[idx])
    return train_mappings, val_mappings

# ---------------------------------------------------------
# 4. Training Loop
# ---------------------------------------------------------
def train(args):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")
    
    # Map all samples across all HDF5 files without loading them into memory
    chunks = prepare_chunks(args.data_dir, shift=args.shift, chunk_len=args.chunk_len, max_gap=args.max_gap)

    # Train / Validation split by contiguous chunks (80% / 20%)
    train_mappings, val_mappings = split_chunks(chunks, val_ratio=0.2, seed=42)
    print(f"Train samples: {len(train_mappings)}, Val samples: {len(val_mappings)} "
          f"(split by chunks of {args.chunk_len} frames)")
    if not train_mappings or not val_mappings:
        raise ValueError("Not enough data to create both a training and a validation set.")

    train_dataset = DrivingDataset(train_mappings, shift=args.shift, transform=True)
    val_dataset = DrivingDataset(val_mappings, shift=args.shift, transform=False)
    
    # num_workers=0 is recommended on Jetson Orin to prevent multiprocessing overhead / memory replication
    train_loader = DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True, drop_last=True, num_workers=0)
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size, shuffle=False, num_workers=0)
    
    # Keep the ImageNet weights next to the models so training works offline after the first download
    os.environ.setdefault('TORCH_HOME', os.path.join(args.model_dir, 'torch_cache'))
    print(f"Pretrained weights cache: {os.environ['TORCH_HOME']}")

    # Model, Loss, Optimizer
    model = PiusModel().to(device)
    
    # Huber Loss (SmoothL1Loss) for robustness against outliers
    criterion = nn.SmoothL1Loss()
    optimizer = optim.Adam(model.parameters(), lr=args.lr)
    
    train_losses = []
    val_losses = []
    
    for epoch in range(args.epochs):
        model.train()
        epoch_train_loss = 0.0
        for batch_imgs, batch_targets in train_loader:
            batch_imgs, batch_targets = batch_imgs.to(device), batch_targets.to(device)
            
            optimizer.zero_grad()
            outputs = model(batch_imgs)
            loss = criterion(outputs, batch_targets)
            loss.backward()
            optimizer.step()
            
            epoch_train_loss += loss.item() * batch_imgs.size(0)
            
        epoch_train_loss /= len(train_loader.dataset)
        train_losses.append(epoch_train_loss)
        
        # Validation
        model.eval()
        epoch_val_loss = 0.0
        with torch.no_grad():
            for batch_imgs, batch_targets in val_loader:
                batch_imgs, batch_targets = batch_imgs.to(device), batch_targets.to(device)
                outputs = model(batch_imgs)
                loss = criterion(outputs, batch_targets)
                epoch_val_loss += loss.item() * batch_imgs.size(0)
        
        epoch_val_loss /= len(val_loader.dataset)
        val_losses.append(epoch_val_loss)
        
        print(f"Epoch [{epoch+1}/{args.epochs}] -> Train Loss: {epoch_train_loss:.6f}, Val Loss: {epoch_val_loss:.6f}")
        
    os.makedirs(args.model_dir, exist_ok=True)

    # Plot loss curves (matplotlib is optional: it is not installed in the Jetson image)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        plt.figure(figsize=(10, 5))
        plt.plot(train_losses, label='Train Loss')
        plt.plot(val_losses, label='Val Loss')
        plt.xlabel('Epoch')
        plt.ylabel('Huber Loss')
        plt.title('Training and Validation Loss')
        plt.legend()
        plot_path = os.path.join(args.model_dir, 'loss_plot.png')
        plt.savefig(plot_path)
        print(f"Saved loss plot to: {plot_path}")
    except ImportError:
        print("matplotlib is not installed; skipped the loss plot (pip install matplotlib to enable it).")
    
    # Save model weights (.pth)
    pth_path = os.path.join(args.model_dir, 'model.pth')
    torch.save(model.state_dict(), pth_path)
    print(f"Saved PyTorch weights to: {pth_path}")
 
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Pius ResNet-18 Training Script")
    parser.add_argument('--data_dir', type=str, default='/home/orin/dev/ws/colcon_ws/data', help='Directory containing HDF5 files')
    parser.add_argument('--model_dir', type=str, default='/home/orin/dev/ws/colcon_ws/models', help='Directory to save models')
    parser.add_argument('--epochs', type=int, default=20, help='Number of epochs to train')
    parser.add_argument('--batch_size', type=int, default=32, help='Batch size')
    parser.add_argument('--lr', type=float, default=1e-4, help='Learning rate')
    parser.add_argument('--shift', type=int, default=1, help='Label shift (forward prediction)')
    parser.add_argument('--chunk_len', type=int, default=100, help='Frames per contiguous chunk for the train/val split')
    parser.add_argument('--max_gap', type=float, default=0.5, help='Max seconds between a frame and its label frame (files with timestamps only)')
    args = parser.parse_args()
    
    train(args)
