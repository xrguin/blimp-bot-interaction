"""Dependency-free 8-bit RGB PNG encoding (used by the camera recorder and the web camera feed)."""
from __future__ import annotations

import struct
import zlib

import numpy as np


def png_bytes(rgb: np.ndarray, level: int = 6) -> bytes:
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))   # filter byte 0 per row

    def chunk(tag, data):
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)

    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, level)) + chunk(b"IEND", b""))


def write_png(path, rgb: np.ndarray):
    with open(path, "wb") as f:
        f.write(png_bytes(rgb))


def encode_image(rgb: np.ndarray, quality: int = 80):
    """Return (bytes, media_type): JPEG through OpenCV or Pillow when available, else PNG."""
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    try:
        import cv2
        ok, buf = cv2.imencode(".jpg", rgb[..., ::-1], [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        if ok:
            return buf.tobytes(), "image/jpeg"
    except ImportError:
        pass
    try:
        from io import BytesIO
        from PIL import Image
        out = BytesIO()
        Image.fromarray(rgb).save(out, format="JPEG", quality=int(quality))
        return out.getvalue(), "image/jpeg"
    except ImportError:
        pass
    return png_bytes(rgb), "image/png"
