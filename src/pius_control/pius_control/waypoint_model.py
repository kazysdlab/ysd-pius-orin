"""Waypoint prediction model (image only) shared by training (train_waypoint.py) and inference (inference_node).

Front camera image (BGR uint8, H x W = 128 x 320 after the camera node's crop and resize)
  -> ResNet-18 (ImageNet pretrained) -> 512 x 4 x 10 feature map
  -> 1x1 conv to 64 channels, flattened (2560, keeps where things are instead of global average pooling)
  -> linear -> 512-d image feature
  -> trajectory branch: GRUCell unrolled n_points steps, each step predicts the offset to the next waypoint;
     the cumulative sum gives waypoints every `spacing` metres of travelled distance, in the rear-axle frame
     (x forward, y left, metres)
  -> (optional) control branch: steering (tanh) and throttle (sigmoid), used only as an auxiliary loss

The vehicle side turns the waypoints into steering (Pure Pursuit) and a target speed (PID on the measured
speed), so the network has no speed or yaw-rate input (that avoids learning to copy the current state).
"""
import torch
import torch.nn as nn
import torchvision.models as models

# Saved with the weights so that inference rebuilds exactly the same model
DEFAULT_CONFIG = {
    "arch": "waypoint_resnet18_v1",
    "image_size": [128, 320],      # H, W of the model input
    "color_order": "BGR",          # what the camera node publishes; converted to RGB inside the model
    "n_points": 5,
    "spacing": 1.0,                # metres of travelled distance between waypoints
    "reduced_channels": 64,
    "feature_dim": 512,
    "hidden_dim": 256,
    "control_branch": True,
}

IMAGENET_MEAN = (0.485, 0.456, 0.406)
IMAGENET_STD = (0.229, 0.224, 0.225)


class WaypointModel(nn.Module):
    def __init__(self, config=None, pretrained=False):
        super().__init__()
        self.config = dict(DEFAULT_CONFIG, **(config or {}))
        c = self.config
        resnet = models.resnet18(weights=models.ResNet18_Weights.DEFAULT if pretrained else None)
        self.backbone = nn.Sequential(*list(resnet.children())[:-2])    # keep the spatial feature map
        h, w = c["image_size"]
        fh, fw = (h + 31) // 32, (w + 31) // 32                          # ResNet downsamples by 32
        self.reduce = nn.Sequential(nn.Conv2d(512, c["reduced_channels"], 1), nn.ReLU(inplace=True))
        self.feature = nn.Sequential(
            nn.Flatten(), nn.Linear(c["reduced_channels"] * fh * fw, c["feature_dim"]), nn.ReLU(inplace=True))

        self.init_hidden = nn.Linear(c["feature_dim"], c["hidden_dim"])
        self.gru = nn.GRUCell(input_size=2, hidden_size=c["hidden_dim"])
        self.delta = nn.Linear(c["hidden_dim"], 2)

        self.control = nn.Sequential(
            nn.Linear(c["feature_dim"], 256), nn.ReLU(inplace=True), nn.Linear(256, 2)) if c["control_branch"] else None

        # Input normalisation lives in the model so that training, inference and TensorRT share it
        self.register_buffer("mean", torch.tensor(IMAGENET_MEAN).view(1, 3, 1, 1))
        self.register_buffer("std", torch.tensor(IMAGENET_STD).view(1, 3, 1, 1))
        self.bgr = c["color_order"].upper() == "BGR"

    def forward(self, image):
        """image: float tensor [B, 3, H, W] in 0..1, channels in config["color_order"].

        Returns waypoints [B, n_points, 2] (metres, rear-axle frame), and controls [B, 2] (steering -1..1,
        throttle 0..1) or None without the control branch.
        """
        x = image.flip(1) if self.bgr else image
        x = (x - self.mean) / self.std
        feat = self.feature(self.reduce(self.backbone(x)))

        h = torch.tanh(self.init_hidden(feat))
        pos = torch.zeros(image.shape[0], 2, dtype=feat.dtype, device=feat.device)
        points = []
        for _ in range(self.config["n_points"]):
            h = self._gru_step(pos, h)  # the input is the current waypoint (TCP style)
            pos = pos + self.delta(h)
            points.append(pos)
        waypoints = torch.stack(points, dim=1)

        controls = None
        if self.control is not None:
            out = self.control(feat)
            controls = torch.stack([torch.tanh(out[:, 0]), torch.sigmoid(out[:, 1])], dim=1)
        return waypoints, controls

    def _gru_step(self, x, h):
        if not torch.onnx.is_in_onnx_export():
            return self.gru(x, h)
        # The fused CUDA GRU cell cannot be exported to ONNX (TensorRT): same equations with plain ops
        gi = nn.functional.linear(x, self.gru.weight_ih, self.gru.bias_ih)
        gh = nn.functional.linear(h, self.gru.weight_hh, self.gru.bias_hh)
        i_r, i_z, i_n = gi.chunk(3, dim=1)
        h_r, h_z, h_n = gh.chunk(3, dim=1)
        r = torch.sigmoid(i_r + h_r)
        z = torch.sigmoid(i_z + h_z)
        n = torch.tanh(i_n + r * h_n)
        return (1 - z) * n + z * h


def to_tensor(images_hwc_uint8):
    """[B, H, W, 3] or [H, W, 3] uint8 numpy (as published by the camera) -> float tensor [B, 3, H, W] in 0..1."""
    t = torch.as_tensor(images_hwc_uint8)
    if t.dim() == 3:
        t = t.unsqueeze(0)
    return t.permute(0, 3, 1, 2).float().div_(255.0)


def save_checkpoint(path, model, extra=None):
    torch.save({"config": model.config, "state_dict": model.state_dict(), **(extra or {})}, path)


def load_checkpoint(path, map_location="cpu"):
    """Returns (model, checkpoint dict). Raises ValueError for files that are not waypoint models."""
    ckpt = torch.load(path, map_location=map_location, weights_only=False)
    if not isinstance(ckpt, dict) or "config" not in ckpt or "state_dict" not in ckpt:
        raise ValueError("waypoint モデルの形式ではありません（旧形式の操作量モデルの可能性があります）")
    model = WaypointModel(ckpt["config"], pretrained=False)
    model.load_state_dict(ckpt["state_dict"])
    return model, ckpt
