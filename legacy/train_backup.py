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
import matplotlib.pyplot as plt
import argparse

# ---------------------------------------------------------
# 1. Model Definition (CNN + LSTM)
# ---------------------------------------------------------
class PiusModel(nn.Module):
    def __init__(self, hidden_size=128):
        super(PiusModel, self).__init__()
        
        # Backbone: ResNet-18 (Pretrained on ImageNet)
        resnet = models.resnet18(weights=models.ResNet18_Weights.DEFAULT)
        # Remove the final classification layer to get feature vectors
        self.backbone = nn.Sequential(*list(resnet.children())[:-1])
        cnn_out_dim = 512
        
        # Temporal Module: LSTM
        self.lstm = nn.LSTM(input_size=cnn_out_dim, hidden_size=hidden_size, num_layers=1, batch_first=True)
        
        # Regression Heads (MLP)
        self.fc_steering = nn.Sequential(
            nn.Linear(hidden_size, 64),
            nn.ReLU(),
            nn.Linear(64, 1),
            nn.Tanh()  # Output range: [-1.0, 1.0]
        )
        
        self.fc_throttle = nn.Sequential(
            nn.Linear(hidden_size, 64),
            nn.ReLU(),
            nn.Linear(64, 1),
            nn.Sigmoid()  # Output range: [0.0, 1.0]
        )

    def forward(self, x):
        # x shape: [batch, seq_len, C, H, W]
        batch_size, seq_len, C, H, W = x.size()
        
        # Reshape to process all frames through CNN at once
        x = x.view(batch_size * seq_len, C, H, W)
        
        # CNN Feature Extraction
        cnn_features = self.backbone(x) # shape: [batch*seq_len, 512, 1, 1]
        cnn_features = cnn_features.view(batch_size, seq_len, -1) # shape: [batch, seq_len, 512]
        
        # LSTM Temporal Processing
        lstm_out, _ = self.lstm(cnn_features) # lstm_out shape: [batch, seq_len, hidden_size]
        
        # We only care about the output from the last time step
        last_hidden = lstm_out[:, -1, :] # shape: [batch, hidden_size]
        
        # Regression Heads
        steering = self.fc_steering(last_hidden)
        throttle = self.fc_throttle(last_hidden)
        
        # Concatenate outputs: shape [batch, 2]
        return torch.cat((steering, throttle), dim=1)

# ---------------------------------------------------------
# 2. Dataset and Data Augmentation
# ---------------------------------------------------------
class SequenceDrivingDataset(Dataset):
    def __init__(self, images, steerings, throttles, seq_len=5, shift=1, transform=False):
        """
        Args:
            images: np.array of shape [N, H, W, C]
            steerings: np.array of shape [N]
            throttles: np.array of shape [N]
            seq_len: Number of frames in a sequence
            shift: Number of frames to shift the label forward (for predicting future action)
            transform: Boolean, whether to apply data augmentation
        """
        self.images = images
        self.steerings = steerings
        self.throttles = throttles
        self.seq_len = seq_len
        self.shift = shift
        self.transform = transform
        
        # Maximum valid index where we can get a full sequence AND the shifted label
        self.max_idx = len(self.images) - self.seq_len - self.shift

    def __len__(self):
        return max(0, self.max_idx + 1)

    def _augment_sequence(self, sequence_tensors, steering_val):
        """
        Applies consistent random transforms to the entire image sequence.
        """
        # 1. Random Horizontal Flip
        if random.random() > 0.5:
            sequence_tensors = [TF.hflip(img) for img in sequence_tensors]
            steering_val = -steering_val  # Flip steering angle!

        # 2. Random Affine (Translation only, simulate track shift)
        if random.random() > 0.5:
            max_translate_x = int(0.1 * sequence_tensors[0].shape[2]) # 10% width
            translate_x = random.randint(-max_translate_x, max_translate_x)
            sequence_tensors = [TF.affine(img, angle=0, translate=[translate_x, 0], scale=1.0, shear=0) for img in sequence_tensors]

        # 3. Color Jitter (Brightness, Contrast)
        if random.random() > 0.5:
            brightness_factor = random.uniform(0.7, 1.3)
            contrast_factor = random.uniform(0.7, 1.3)
            sequence_tensors = [TF.adjust_brightness(img, brightness_factor) for img in sequence_tensors]
            sequence_tensors = [TF.adjust_contrast(img, contrast_factor) for img in sequence_tensors]

        return sequence_tensors, steering_val

    def __getitem__(self, idx):
        # 1. Extract sequence of images
        # Images are loaded as [seq_len, H, W, C] (uint8)
        seq_imgs = self.images[idx : idx + self.seq_len]
        
        # 2. Extract label (shifted forward)
        label_idx = idx + self.seq_len - 1 + self.shift
        steering = float(self.steerings[label_idx])
        throttle = float(self.throttles[label_idx])
        
        # 3. Convert to PyTorch tensors [C, H, W] in [0.0, 1.0] range
        # ResNet expects inputs in [0.0, 1.0], normalized.
        seq_tensors = []
        for i in range(self.seq_len):
            img_chw = seq_imgs[i].transpose(2, 0, 1) # HWC to CHW
            tensor_img = torch.tensor(img_chw, dtype=torch.float32) / 255.0
            seq_tensors.append(tensor_img)
            
        # 4. Apply Augmentations (only during training)
        if self.transform:
            seq_tensors, steering = self._augment_sequence(seq_tensors, steering)
            
        # 5. Stack sequence into [seq_len, C, H, W]
        input_sequence = torch.stack(seq_tensors, dim=0)
        
        # Note: ResNet-18 normally expects ImageNet normalization. 
        # For simplicity and to match custom domain, we skip standard mean/std normalization here,
        # but keep it in [0, 1].
        
        target = torch.tensor([steering, throttle], dtype=torch.float32)
        return input_sequence, target

# ---------------------------------------------------------
# 3. Data Loading
# ---------------------------------------------------------
def load_data(data_dir):
    h5_files = glob.glob(os.path.join(data_dir, '*.h5'))
    if not h5_files:
        raise ValueError(f"No HDF5 files found in data directory: {data_dir}")
        
    print(f"Found {len(h5_files)} HDF5 files. Loading datasets...")
    
    all_images = []
    all_steerings = []
    all_throttles = []
    
    for filepath in h5_files:
        print(f"Reading {filepath}...")
        with h5py.File(filepath, 'r') as f:
            if 'image' in f and 'steering' in f and 'throttle' in f:
                all_images.append(f['image'][:])
                all_steerings.append(f['steering'][:])
                all_throttles.append(f['throttle'][:])
            else:
                print(f"Warning: Datasets missing in {filepath}. Skipping.")
                
    images = np.concatenate(all_images, axis=0)
    steerings = np.concatenate(all_steerings, axis=0)
    throttles = np.concatenate(all_throttles, axis=0)
    
    print(f"Total dataset loaded: {len(images)} samples.")
    return images, steerings, throttles

# ---------------------------------------------------------
# 4. Training Loop
# ---------------------------------------------------------
def train(args):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")
    
    # Load data
    images, steerings, throttles = load_data(args.data_dir)
    
    num_samples = len(images)
    indices = np.arange(num_samples)
    
    # Simple split (not ideal for time-series if shuffled completely, 
    # but we just split the flat arrays before sequencing for simplicity here. 
    # Better approach: split contiguous chunks. Let's do a contiguous split to avoid sequence leakage.)
    split = int(num_samples * 0.8)
    
    train_images = images[:split]
    train_steerings = steerings[:split]
    train_throttles = throttles[:split]
    
    val_images = images[split:]
    val_steerings = steerings[split:]
    val_throttles = throttles[split:]
    
    train_dataset = SequenceDrivingDataset(train_images, train_steerings, train_throttles, 
                                           seq_len=args.seq_len, shift=args.shift, transform=True)
    val_dataset = SequenceDrivingDataset(val_images, val_steerings, val_throttles, 
                                         seq_len=args.seq_len, shift=args.shift, transform=False)
    
    train_loader = DataLoader(train_dataset, batch_size=args.batch_size, shuffle=True, drop_last=True)
    val_loader = DataLoader(val_dataset, batch_size=args.batch_size, shuffle=False)
    
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
        
    # Plot loss curves
    plt.figure(figsize=(10, 5))
    plt.plot(train_losses, label='Train Loss')
    plt.plot(val_losses, label='Val Loss')
    plt.xlabel('Epoch')
    plt.ylabel('Huber Loss')
    plt.title('Training and Validation Loss')
    plt.legend()
    plot_path = os.path.join(args.model_dir, 'loss_plot.png')
    os.makedirs(args.model_dir, exist_ok=True)
    plt.savefig(plot_path)
    print(f"Saved loss plot to: {plot_path}")
    
    # Save model weights (.pth)
    pth_path = os.path.join(args.model_dir, 'model.pth')
    torch.save(model.state_dict(), pth_path)
    print(f"Saved PyTorch weights to: {pth_path}")
    
    # Note: ONNX export is skipped or commented out for LSTM models to avoid complexity.
    # We will use PyTorch native inference on Jetson.

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Pius CNN+LSTM Training Script")
    parser.add_argument('--data_dir', type=str, default='/home/orin/dev/ws/colcon_ws/data', help='Directory containing HDF5 files')
    parser.add_argument('--model_dir', type=str, default='/home/orin/dev/ws/colcon_ws/models', help='Directory to save outputs')
    parser.add_argument('--epochs', type=int, default=20, help='Number of epochs to train')
    parser.add_argument('--batch_size', type=int, default=32, help='Batch size')
    parser.add_argument('--lr', type=float, default=1e-4, help='Learning rate')
    parser.add_argument('--seq_len', type=int, default=5, help='LSTM sequence length')
    parser.add_argument('--shift', type=int, default=1, help='Label shift (forward prediction)')
    args = parser.parse_args()
    
    train(args)
