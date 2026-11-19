import logging
import struct
import zlib
from pathlib import Path
from typing import Any, Self

import numpy as np

from hexastorm import ulabext
from hexastorm.config import Spi

logger = logging.getLogger(__name__)


def bits_to_events_np(line_bits: np.ndarray, laser_ticks: int) -> list[tuple[int, int]]:
    """Convert a 1D binary scanline numpy array into (tick, intensity) events."""
    return ulabext.bits_to_events(np.asarray(line_bits, dtype=np.uint8), laser_ticks)


def events_to_bits_np(
    events: list[tuple[int, int]], scanline_length: int, laser_ticks: int
) -> np.ndarray:
    """Reconstruct a 1D uint8 binary scanline array from (tick, intensity) events."""
    scanline_length = int(scanline_length)
    bits = np.zeros(scanline_length, dtype=np.uint8)
    if not events or scanline_length <= 0:
        return bits

    tick_step = int(laser_ticks)
    valid = [(int(t), int(v)) for t, v in events if 0 <= int(t) < ulabext.SENTINEL_TICK]
    for i, (tick, intensity) in enumerate(valid):
        start_px = max(0, tick // tick_step)
        if start_px >= scanline_length:
            break
        if i + 1 < len(valid):
            end_px = min(scanline_length, valid[i + 1][0] // tick_step)
        else:
            end_px = scanline_length
        bits[start_px:end_px] = 1 if intensity > 0 else 0
    return bits


def tagged_bin_path(path: str | Path, params: Any) -> Path:
    """Append exposure count and optical-correction tag to output filename stem."""
    out = Path(path)
    n_exp = int(params["exposures"])
    cor_tag = "cor" if params["optical_correction"] else "nocor"
    return out.with_name(f"{out.stem}_e{n_exp}_{cor_tag}{out.suffix}")


class EventStreamWriter:
    """Streams variable-length (tick, intensity) scanline events into a zlib-compressed SPI .pat/.bin file."""

    IO_BUFFER_SIZE = 1024 * 1024

    def __init__(
        self,
        filepath: str | Path,
        lanewidth: float,
        facets_in_lane: int,
        lanes: int,
        steps_per_line: float,
        bits_in_scanline: int,
        compression_level: int = 9,
    ):
        self.out_path = Path(filepath)
        self.lanewidth = float(lanewidth)
        self.facets_in_lane = int(facets_in_lane)
        self.lanes = int(lanes)
        self.steps_per_line = float(steps_per_line)
        self.bits_in_scanline = int(bits_in_scanline)
        self.compression_level = int(compression_level)

        self._scanline_cmd = Spi.Instructions.scanline
        self._spi_write_cmd = Spi.Commands.write.to_bytes(1, "big")
        self._compressor = zlib.compressobj(level=self.compression_level)
        self._write_buffer = bytearray()
        self._fh = None

    def __enter__(self) -> Self:
        self.out_path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = open(self.out_path, "wb")
        header = struct.pack("<fII", self.lanewidth, self.facets_in_lane, self.lanes)
        self._fh.write(self._compressor.compress(header))
        return self

    def write_scanline(self, events: list[tuple[int, int]], direction: int) -> None:
        """Pack one scanline's events into SPI write frames and buffer compressed output."""
        full_payload = ulabext.pack_scanline_events(
            events=events,
            steps_line=self.steps_per_line,
            direction=direction,
            scanline_length=self.bits_in_scanline,
            scanline_cmd=self._scanline_cmd,
        )
        for i in range(0, len(full_payload), 8):
            chunk = full_payload[i : i + 8]
            self._write_buffer.extend(self._spi_write_cmd)
            self._write_buffer.extend(chunk[::-1])

        if len(self._write_buffer) >= self.IO_BUFFER_SIZE:
            self._fh.write(self._compressor.compress(self._write_buffer))
            self._write_buffer = bytearray()

    def write_raw_spi(self, raw_spi: bytes | bytearray | memoryview) -> None:
        """Compress and append pre-framed 9-byte SPI write packets directly."""
        if self._write_buffer:
            self._fh.write(self._compressor.compress(self._write_buffer))
            self._write_buffer = bytearray()
        if raw_spi:
            self._fh.write(self._compressor.compress(raw_spi))

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        try:
            if self._fh is not None and exc_type is None:
                if self._write_buffer:
                    self._fh.write(self._compressor.compress(self._write_buffer))
                    self._write_buffer = bytearray()
                self._fh.write(self._compressor.flush())
        finally:
            if self._fh is not None:
                self._fh.close()
                self._fh = None


def write_binary_file(
    pixeldata: np.ndarray,
    params: dict[str, Any],
    filepath: str | Path,
    compression_level: int = 9,
    invert_hw_scan_dir: bool = True,
) -> None:
    """
    Encodes and writes pixel data to a compressed event-driven binary file for FPGA streaming.

    For each scanline:
    1.  Extracts the 1-bit line in hardware sweep order (reversed relative to image order).
    2.  Run-length encodes transitions into 32-bit `[24-bit tick | 8-bit intensity]` events.
    3.  Prefixes an 8-byte scanline header containing direction, event word count, and step timing.
    4.  Packetizes every 8-byte word into a 9-byte SPI write frame and compresses with zlib.

    File Format:
        [Header: lanewidth(f32), facets(u32), lanes(u32)]
        [Zlib Compressed Event-Driven SPI Stream]

    Args:
        pixeldata: The raw 1-bit packed raster data (numpy array, bitorder='big').
        params: Geometry settings dict (must contain lanewidth, facetsinlane, laser_ticks, etc.).
        filepath: Output destination path.
        compression_level: Zlib compression level (0-9).
    """
    out_path = Path(filepath)

    # 1. Setup Parameters
    lanewidth = float(params["lanewidth"])
    facets = int(params["facetsinlane"])
    bits_in_scanline = int(params["bitsinscanline"])
    samplexsize = float(params["samplexsize"])
    stepsperline = float(params["stepsperline"])
    laser_ticks = int(params["laser_ticks"])

    lanes = int(np.ceil(samplexsize / lanewidth))
    bytes_in_line = int(np.ceil(bits_in_scanline / 8))

    # 2. Reshape into (lanes, facets, bytes_in_line)
    try:
        expected_size = lanes * facets * bytes_in_line
        if pixeldata.size != expected_size:
            logger.debug(f"Resizing data: {pixeldata.size} -> {expected_size}")
            pixeldata.resize(expected_size, refcheck=False)

        grid = pixeldata.reshape(lanes, facets, bytes_in_line)
    except ValueError as e:
        raise ValueError(f"Data shape mismatch: {e}") from e

    # 3. Encode & Write Loop
    logger.info(f"Writing event-driven binary file to {out_path}...")
    with EventStreamWriter(
        filepath=out_path,
        lanewidth=lanewidth,
        facets_in_lane=facets,
        lanes=lanes,
        steps_per_line=stepsperline,
        bits_in_scanline=bits_in_scanline,
        compression_level=compression_level,
    ) as writer:
        for lane_idx in range(lanes):
            direction = 1 if lane_idx % 2 == 0 else 0
            lane_data = grid[lane_idx]

            for facet_idx in range(facets):
                # Unpack image-order bits and optionally reverse to hardware scan order
                line_bits_img = np.unpackbits(lane_data[facet_idx], bitorder="big")[
                    :bits_in_scanline
                ]
                line_bits_hw = (
                    line_bits_img[::-1] if invert_hw_scan_dir else line_bits_img
                )
                events = bits_to_events_np(line_bits_hw, laser_ticks)
                writer.write_scanline(events, direction)


def read_binary_file(
    filepath: str | Path, laser_timing_cfg: dict, bits_in_scanline: int
) -> tuple[int, int, float, np.ndarray]:
    """
    Reads, decompresses, and decodes an event-driven binary laser file back into raw pixel data.

    Reconstructs the 1-bit bitmap per scanline from the SPI-framed `[tick, intensity]`
    event stream so visualization and verification tools receive the exact original bitmap.

    Args:
        filepath: Path to the .bin / .pat file.
        laser_timing_cfg: Configuration dict containing `laser_ticks`.
        bits_in_scanline: Expected number of valid bits per line.

    Returns:
        Tuple containing: (facets_in_lane, lanes, lanewidth, flattened_pixel_data)
    """
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"Binary file not found: {path}")

    with open(path, "rb") as f:
        data = zlib.decompress(f.read())

    # Parse File Header
    lanewidth, facets_in_lane, lanes = struct.unpack("<fII", data[:12])

    laser_ticks = int(laser_timing_cfg["laser_ticks"])
    bits_in_scanline = int(bits_in_scanline)
    bytes_in_line = int(np.ceil(bits_in_scanline / 8))
    total_lines = lanes * facets_in_lane

    raw_payload = np.frombuffer(data, dtype=np.uint8, offset=12)
    lines = np.zeros((total_lines, bytes_in_line), dtype=np.uint8)

    offset = 0
    parsed_lines = 0
    payload_len = raw_payload.size

    while parsed_lines < total_lines:
        if offset + 9 > payload_len:
            logger.warning(
                f"File size mismatch. Expected {total_lines} lines, found {parsed_lines}."
            )
            lines = lines[:parsed_lines]
            break

        # Word 0: [1B SPI write cmd][8B reversed header]
        word0 = raw_payload[offset + 1 : offset + 9][::-1].tobytes()
        offset += 9

        instr = word0[0]
        if instr == Spi.Instructions.last_scanline:
            parsed_lines += 1
            continue
        if instr != Spi.Instructions.scanline:
            raise ValueError(
                f"Unexpected scanline instruction {instr:#x} at line {parsed_lines}"
            )

        payload_int = int.from_bytes(word0[1:8], "little")
        event_words = (payload_int >> 1) & 0xFFFF

        if offset + event_words * 9 > payload_len:
            logger.warning(
                f"Truncated event stream at line {parsed_lines} "
                f"(expected {event_words} event words)."
            )
            lines = lines[:parsed_lines]
            break

        events: list[tuple[int, int]] = []
        for _ in range(event_words):
            ev_bytes = raw_payload[offset + 1 : offset + 9][::-1].tobytes()
            offset += 9
            ev0, ev1 = struct.unpack("<II", ev_bytes)
            t0 = (ev0 >> 8) & 0xFFFFFF
            if t0 < ulabext.SENTINEL_TICK:
                events.append((t0, ev0 & 0xFF))
            t1 = (ev1 >> 8) & 0xFFFFFF
            if t1 < ulabext.SENTINEL_TICK:
                events.append((t1, ev1 & 0xFF))

        line_bits_hw = events_to_bits_np(events, bits_in_scanline, laser_ticks)
        line_bits_img = line_bits_hw[::-1]
        lines[parsed_lines] = np.packbits(line_bits_img, bitorder="big")[:bytes_in_line]
        parsed_lines += 1

    return facets_in_lane, lanes, lanewidth, lines.flatten()
