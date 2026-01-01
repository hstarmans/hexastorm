import logging
import struct
import zlib
from pathlib import Path
from typing import Dict, Tuple, Any, Union

import numpy as np
from hexastorm.config import Spi


def write_binary_file(
    pixeldata: np.ndarray,
    params: Any,  # Can be dict or Numba typed dict
    filepath: Union[str, Path],
    compression_level: int = 9,
) -> None:
    """
    Writes pixeldata to a compressed binary file with SPI formatting.

    Args:
        pixeldata: The raw bit data (numpy array).
        params: Dictionary containing geometry settings (lanewidth, facetsinlane, etc.).
        filepath: Destination path.
        compression_level: Zlib compression level (0-9).
    """
    out_path = Path(filepath)
    # Ensure parent directory exists
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # 1. Setup Parameters
    # Handle both Numba typed dict and standard dict
    lanewidth = float(params["lanewidth"])
    facets = int(params["facetsinlane"])
    bits_in_scanline = int(params["bitsinscanline"])
    samplexsize = float(params["samplexsize"])
    stepsperline = float(params["stepsperline"])

    lanes = int(np.ceil(samplexsize / lanewidth))
    bytes_in_line = int(np.ceil(bits_in_scanline / 8))

    # 2. Reshape & Global Reversal (Vectorized)
    try:
        expected_size = lanes * facets * bytes_in_line
        if pixeldata.size != expected_size:
            logging.info(f"Resizing data: {pixeldata.size} -> {expected_size}")
            # Note: resize in-place might affect the array passed by reference,
            # but usually patternfile creates a new array anyway.
            pixeldata.resize(expected_size, refcheck=False)

        grid = pixeldata.reshape(lanes, facets, bytes_in_line)
    except ValueError as e:
        raise ValueError(f"Data shape mismatch: {e}")

    # Reverse the image data bytes (Corresponds to bits[::-1])
    grid = grid[:, :, ::-1]

    # 3. Pre-calculate Headers
    # Calculate the 7-byte configuration header once
    def make_header(direction):
        half_period = int((bits_in_scanline - 1) // (stepsperline * 2))
        # (HalfPeriod << 1) | Direction
        payload = (half_period << 1) | direction
        return payload.to_bytes(7, "little")

    # Create full headers: [CMD_SCANLINE (0x03 usually)] + [Config Bytes]
    # Note: Your original code used Spi.Instructions.scanline.
    # Ensure this matches the byte value (e.g., b'\x03')
    scanline_cmd = Spi.Instructions.scanline.to_bytes(1, "big")
    header_bwd = scanline_cmd + make_header(0)
    header_fwd = scanline_cmd + make_header(1)

    # Calculate Padding
    total_len = len(header_fwd) + bytes_in_line
    pad_len = (8 - (total_len % 8)) % 8
    padding = b"\x00" * pad_len
    spi_write_cmd = Spi.Commands.write.to_bytes(1, "big")

    # 4. Write Loop
    compressor = zlib.compressobj(level=compression_level)

    # 1MB Buffer
    IO_BUFFER_SIZE = 1024 * 1024
    write_buffer = bytearray()

    with open(out_path, "wb") as f:
        # File Header
        f.write(compressor.compress(struct.pack("<f", lanewidth)))
        f.write(compressor.compress(struct.pack("<I", facets)))
        f.write(compressor.compress(struct.pack("<I", lanes)))

        for lane_idx in range(lanes):
            # Select Header
            is_forward = lane_idx % 2 == 0
            header = header_fwd if is_forward else header_bwd
            # Get all facets for this lane
            lane_data = grid[lane_idx]

            for facet_idx in range(facets):
                # Assemble payload: [Header] + [Image Data (reversed)] + [Padding]
                full_payload = header + lane_data[facet_idx].tobytes() + padding

                # SPI Chunking (Replaces byte_to_cmd_list)
                # Loop over payload in 8-byte chunks
                for i in range(0, len(full_payload), 8):
                    chunk = full_payload[i : i + 8]
                    # Reverse chunk for SPI endianness
                    chunk_reversed = chunk[::-1]
                    write_buffer.extend(spi_write_cmd)
                    write_buffer.extend(chunk_reversed)

                if len(write_buffer) >= IO_BUFFER_SIZE:
                    f.write(compressor.compress(write_buffer))
                    write_buffer = bytearray()

        if write_buffer:
            f.write(compressor.compress(write_buffer))
        f.write(compressor.flush())


def read_binary_file(
    filepath: Union[str, Path], laser_timing_cfg: Dict, bits_in_scanline: int
) -> Tuple[int, int, float, np.ndarray]:
    """
    Reads and decodes a binary laser file.
    """
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"Binary file not found: {path}")

    with open(path, "rb") as f:
        compressed_data = f.read()
        data = zlib.decompress(compressed_data)

    # Parse File Header
    lanewidth, facets_in_lane, lanes = struct.unpack("<fII", data[:12])

    # Geometry for parsing
    words_in_line = Spi.words_scanline(laser_timing_cfg)
    bytes_in_line = int(np.ceil(bits_in_scanline / 8))
    total_lines = lanes * facets_in_lane

    # Load Raw Data
    raw_payload = np.frombuffer(data, dtype=np.uint8, offset=12)

    expected_size = total_lines * words_in_line * 9
    if raw_payload.size != expected_size:
        trunc_lines = raw_payload.size // (words_in_line * 9)
        logging.warning(
            f"File size mismatch. Expected {total_lines} lines, found {trunc_lines}."
        )
        total_lines = trunc_lines
        raw_payload = raw_payload[: total_lines * words_in_line * 9]

    # Reshape to (Lines, Words per Line, 9 Bytes per Word)
    grid = raw_payload.reshape(total_lines, words_in_line, 9)

    # --- UNDO SPI PIPELINE ---

    # A. Separate Headers (Word 0) from Data (Words 1..N)
    # The first word of every line is the Command/Header word. We discard it.
    data_grid = grid[:, 1:, :]

    # B. Remove the first byte (Write Command 0x01) from every Data Word
    # The remaining 8 bytes are actual pixel data.
    payload_bytes = data_grid[:, :, 1:]

    # C. Reverse the SPI Chunks
    # The writing process reversed these 8-byte chunks. We reverse them back.
    payload_bytes = payload_bytes[..., ::-1]

    # D. Flatten lines
    lines = payload_bytes.reshape(total_lines, -1)

    # E. Trim padding (if image width isn't a perfect multiple of SPI words)
    lines = lines[:, :bytes_in_line]

    # F. Global Reverse
    # The writing process reversed the entire line. We reverse it back.
    lines = lines[:, ::-1]

    return facets_in_lane, lanes, lanewidth, lines.flatten()
