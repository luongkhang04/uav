from __future__ import annotations

import math

import numpy as np
from sensor_msgs.msg import Image


def yaw_from_ros_quat(q: object) -> float:
    x = float(q.x)
    y = float(q.y)
    z = float(q.z)
    w = float(q.w)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def depth_image_to_meters(
    msg: Image,
    max_depth_m: float,
    depth_scale: float = 1.0,
) -> np.ndarray:
    encoding = msg.encoding.upper()
    dtype, scale, channels = _depth_encoding(encoding, max_depth_m)
    if msg.is_bigendian:
        dtype = dtype.newbyteorder(">")

    raw = np.frombuffer(msg.data, dtype=dtype)
    if msg.height <= 0 or msg.width <= 0:
        raise RuntimeError("Depth image has invalid dimensions.")

    width = int(msg.width)
    height = int(msg.height)
    itemsize = dtype.itemsize
    row_values = int(msg.step // itemsize) if msg.step else width * channels
    min_row_values = width * channels

    if row_values >= min_row_values and raw.size >= row_values * height:
        image = raw[:row_values * height].reshape(height, row_values)
        image = image[:, :min_row_values].reshape(height, width, channels)
    else:
        expected = width * height * channels
        if raw.size < expected:
            raise RuntimeError(
                "Depth image data is smaller than width, height, and encoding."
            )
        image = raw[:expected].reshape(height, width, channels)

    depth = image[:, :, 0].astype(np.float32) * float(scale)
    depth *= float(depth_scale)
    depth[~np.isfinite(depth)] = np.nan
    return depth


def _depth_encoding(
    encoding: str,
    max_depth_m: float,
) -> tuple[np.dtype, float, int]:
    channels = _channels_from_encoding(encoding)
    if encoding.startswith("32FC"):
        return np.dtype(np.float32), 1.0, channels
    if encoding in {"16UC1", "MONO16"} or encoding.startswith("16UC"):
        return np.dtype(np.uint16), 0.001, channels
    if encoding in {"8UC1", "MONO8"} or encoding.startswith("8UC"):
        return np.dtype(np.uint8), float(max_depth_m) / 255.0, channels
    raise RuntimeError(f"Unsupported depth image encoding: {encoding}")


def _channels_from_encoding(encoding: str) -> int:
    if encoding.endswith("C4"):
        return 4
    if encoding.endswith("C3"):
        return 3
    if encoding.endswith("C2"):
        return 2
    return 1


def resize_depth(
    depth_m: np.ndarray,
    width: int,
    height: int,
) -> np.ndarray:
    if depth_m.shape == (height, width):
        return depth_m
    try:
        import cv2

        return cv2.resize(
            depth_m,
            (width, height),
            interpolation=cv2.INTER_AREA,
        )
    except ImportError:
        y_idx = np.linspace(0, depth_m.shape[0] - 1, height).astype(int)
        x_idx = np.linspace(0, depth_m.shape[1] - 1, width).astype(int)
        return depth_m[np.ix_(y_idx, x_idx)]


def clean_depth(depth_m: np.ndarray, max_depth_m: float) -> np.ndarray:
    depth_m = np.nan_to_num(depth_m, nan=max_depth_m, posinf=max_depth_m)
    return np.clip(depth_m, 0.0, max_depth_m)


def closest_depth_grid(
    depth_m: np.ndarray,
    rows: int,
    cols: int,
) -> np.ndarray:
    features: list[float] = []
    row_splits = np.array_split(depth_m, rows, axis=0)
    for row in row_splits:
        col_splits = np.array_split(row, cols, axis=1)
        for cell in col_splits:
            features.append(float(np.nanmin(cell)))
    return np.asarray(features, dtype=np.float32)


def build_feature_names(rows: int = 5, cols: int = 5) -> list[str]:
    image_names = [
        f"Depth_{row + 1}{col + 1}"
        for row in range(rows)
        for col in range(cols)
    ]
    state_names = [
        "XY distance",
        "Relative altitude",
        "Relative yaw",
        "XY velocity",
        "Z velocity",
        "Yaw rate",
    ]
    return image_names + state_names
