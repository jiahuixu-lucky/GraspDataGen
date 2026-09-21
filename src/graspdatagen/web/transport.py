"""Send mesh arrays as aligned binary buffers instead of millions of JSON numbers."""

import json
import struct
from typing import Any

import numpy as np


def encode_payload(payload: dict[str, Any]) -> bytes:
    buffers: list[bytes] = []
    offset = 0

    def array_buffer(array: np.ndarray) -> dict[str, Any]:
        nonlocal offset
        # Vertices/UVs use the same float32 precision as Three.js BufferGeometry;
        # triangle indices remain exact uint32 values. Server annotation arrays
        # retain their original float64 values for topology checks and NPZ saves.
        dtype = np.dtype("<u4" if array.dtype.kind in "iu" else "<f4")
        buffer = array.astype(dtype, copy=False).tobytes()
        descriptor = {"buffer_type": dtype.name, "offset": offset, "length": array.size}
        buffers.append(buffer)
        offset += len(buffer)
        return descriptor

    header = json.dumps(payload, default=array_buffer, separators=(",", ":")).encode()
    header += b" " * (-len(header) % 4)
    return b"".join([struct.pack("<I", len(header)), header, *buffers])
