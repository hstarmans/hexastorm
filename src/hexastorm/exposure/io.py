import logging
import struct
import zlib
from pathlib import Path
from typing import Any, Self

import numpy as np

from hexastorm import ulabext
from hexastorm.config import Spi

logger = logging.getLogger(__name__)


def _events_to_array_np(
    events: list[tuple[int, int]], num_samples: int, scanline_ticks: int, binary: bool
) -> np.ndarray:
    n = int(num_samples)
    total_ticks = int(scanline_ticks)
    arr = np.zeros(n, dtype=np.uint8)
    if not events or n <= 0 or total_ticks <= 0:
        return arr

    valid = [(int(t), int(v)) for t, v in events if 0 <= int(t) < ulabext.SENTINEL_TICK]
    for i, (tick, intensity) in enumerate(valid):
        start_px = max(0, (tick * n) // total_ticks)
        if start_px >= n:
            break
        if i + 1 < len(valid):
            end_px = min(n, (valid[i + 1][0] * n) // total_ticks)
        else:
            end_px = n
        val = (1 if intensity > 0 else 0) if binary else max(0, min(255, intensity))
        arr[start_px:end_px] = val
    return arr


def bits_to_events_np(
    line_bits: np.ndarray, scanline_ticks: int
) -> list[tuple[int, int]]:
    """Convert a 1D binary scanline numpy array into (tick, intensity) events."""
    return ulabext.bits_to_events(np.asarray(line_bits, dtype=np.uint8), scanline_ticks)


def events_to_bits_np(
    events: list[tuple[int, int]], num_bits: int, scanline_ticks: int
) -> np.ndarray:
    """Reconstruct a 1D uint8 binary array of length ``num_bits`` from tick events."""
    return _events_to_array_np(events, num_bits, scanline_ticks, binary=True)


def intensities_to_events_np(
    line_vals: np.ndarray, scanline_ticks: int
) -> list[tuple[int, int]]:
    """Run-length encode a 1D uint8 intensity array into (tick, intensity) events."""
    return ulabext.intensities_to_events(
        np.asarray(line_vals, dtype=np.uint8), scanline_ticks
    )


def events_to_intensities_np(
    events: list[tuple[int, int]], num_samples: int, scanline_ticks: int
) -> np.ndarray:
    """Reconstruct a 1D uint8 intensity array (0..255) of length ``num_samples`` from tick events."""
    return _events_to_array_np(events, num_samples, scanline_ticks, binary=False)


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
        scanline_ticks: int,
        compression_level: int = 9,
    ):
        self.out_path = Path(filepath)
        self.lanewidth = float(lanewidth)
        self.facets_in_lane = int(facets_in_lane)
        self.lanes = int(lanes)
        self.steps_per_line = float(steps_per_line)
        self.scanline_ticks = int(scanline_ticks)
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
            scanline_ticks=self.scanline_ticks,
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


def _write_raster_stream(
    grid: np.ndarray,
    params: dict[str, Any],
    filepath: str | Path,
    compression_level: int,
    invert_hw_scan_dir: bool,
    grayscale: bool,
) -> None:
    """Shared EventStreamWriter loop for packed 1-bit binary or 8-bit grayscale raster grids."""
    out_path = Path(filepath)
    lanewidth = float(params["lanewidth"])
    facets = int(params["facetsinlane"])
    bits_in_scanline = int(params["bitsinscanline"])
    scanline_ticks = int(params["scanline_ticks"])
    stepsperline = float(params["stepsperline"])
    lanes = grid.shape[0]

    encoder = intensities_to_events_np if grayscale else bits_to_events_np
    mode_label = "grayscale " if grayscale else ""
    logger.info(f"Writing {mode_label}event-driven binary file to {out_path}...")
    with EventStreamWriter(
        filepath=out_path,
        lanewidth=lanewidth,
        facets_in_lane=facets,
        lanes=lanes,
        steps_per_line=stepsperline,
        scanline_ticks=scanline_ticks,
        compression_level=compression_level,
    ) as writer:
        for lane_idx in range(lanes):
            direction = 1 if lane_idx % 2 == 0 else 0
            lane_data = grid[lane_idx]
            for facet_idx in range(facets):
                if grayscale:
                    line_vals_img = lane_data[facet_idx]
                else:
                    line_vals_img = np.unpackbits(lane_data[facet_idx], bitorder="big")[
                        :bits_in_scanline
                    ]
                line_vals_hw = (
                    line_vals_img[::-1] if invert_hw_scan_dir else line_vals_img
                )
                writer.write_scanline(encoder(line_vals_hw, scanline_ticks), direction)


def write_binary_file(
    pixeldata: np.ndarray,
    params: dict[str, Any],
    filepath: str | Path,
    compression_level: int = 9,
    invert_hw_scan_dir: bool = True,
) -> None:
    """Encode packed 1-bit pixel data into an event-driven ``.pat``/``.bin`` file."""
    lanewidth = float(params["lanewidth"])
    facets = int(params["facetsinlane"])
    bits_in_scanline = int(params["bitsinscanline"])
    samplexsize = float(params["samplexsize"])

    lanes = int(np.ceil(samplexsize / lanewidth))
    bytes_in_line = int(np.ceil(bits_in_scanline / 8))

    try:
        expected_size = lanes * facets * bytes_in_line
        if pixeldata.size != expected_size:
            logger.debug(f"Resizing data: {pixeldata.size} -> {expected_size}")
            pixeldata.resize(expected_size, refcheck=False)
        grid = pixeldata.reshape(lanes, facets, bytes_in_line)
    except ValueError as e:
        raise ValueError(f"Data shape mismatch: {e}") from e

    _write_raster_stream(
        grid, params, filepath, compression_level, invert_hw_scan_dir, grayscale=False
    )


def write_grayscale_binary_file(
    intensitydata: np.ndarray,
    params: dict[str, Any],
    filepath: str | Path,
    compression_level: int = 9,
    invert_hw_scan_dir: bool = True,
) -> None:
    """Encode 8-bit ``0..255`` grayscale intensity arrays into an RLE event ``.pat``/``.bin`` file."""
    lanewidth = float(params["lanewidth"])
    facets = int(params["facetsinlane"])
    bits_in_scanline = int(params["bitsinscanline"])
    samplexsize = float(params["samplexsize"])

    lanes = int(np.ceil(samplexsize / lanewidth))
    expected_size = lanes * facets * bits_in_scanline
    flat = np.asarray(intensitydata, dtype=np.uint8).ravel()
    if flat.size != expected_size:
        flat = flat.copy()
        flat.resize(expected_size, refcheck=False)
    grid = flat.reshape(lanes, facets, bits_in_scanline)

    _write_raster_stream(
        grid, params, filepath, compression_level, invert_hw_scan_dir, grayscale=True
    )


def read_binary_events(
    filepath: str | Path,
) -> tuple[int, int, float, list[tuple[int, list[tuple[int, int]]]]]:
    """Read raw per-scanline ``(direction, [(tick, intensity), ...])`` events without rasterizing."""
    path = Path(filepath)
    if not path.exists():
        raise FileNotFoundError(f"Binary file not found: {path}")

    with open(path, "rb") as f:
        data = zlib.decompress(f.read())

    lanewidth, facets_in_lane, lanes = struct.unpack("<fII", data[:12])
    total_lines = lanes * facets_in_lane
    raw_payload = np.frombuffer(data, dtype=np.uint8, offset=12)

    lines_events: list[tuple[int, list[tuple[int, int]]]] = []
    offset = 0
    parsed_lines = 0
    payload_len = raw_payload.size

    while parsed_lines < total_lines:
        if offset + 9 > payload_len:
            logger.warning(
                f"File size mismatch. Expected {total_lines} lines, found {parsed_lines}."
            )
            break

        word0 = raw_payload[offset + 1 : offset + 9][::-1].tobytes()
        offset += 9

        instr = word0[0]
        if instr == Spi.Instructions.last_scanline:
            lines_events.append((0, []))
            parsed_lines += 1
            continue
        if instr != Spi.Instructions.scanline:
            raise ValueError(
                f"Unexpected scanline instruction {instr:#x} at line {parsed_lines}"
            )

        payload_int = int.from_bytes(word0[1:8], "little")
        direction = payload_int & 1
        event_words = (payload_int >> 1) & 0xFFFF

        if offset + event_words * 9 > payload_len:
            logger.warning(
                f"Truncated event stream at line {parsed_lines} "
                f"(expected {event_words} event words)."
            )
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

        lines_events.append((direction, events))
        parsed_lines += 1

    return facets_in_lane, lanes, lanewidth, lines_events


def read_binary_file(
    filepath: str | Path, laser_timing_cfg: dict, bits_in_scanline: int
) -> tuple[int, int, float, np.ndarray]:
    """Reads, decompresses, and rasterizes an event-driven ``.bin``/``.pat`` file to 1-bit bytes."""
    facets_in_lane, lanes, lanewidth, lines_events = read_binary_events(filepath)

    scanline_ticks = int(laser_timing_cfg["scanline_ticks"])
    bits_in_scanline = int(bits_in_scanline)
    bytes_in_line = int(np.ceil(bits_in_scanline / 8))
    lines = np.zeros((len(lines_events), bytes_in_line), dtype=np.uint8)

    for idx, (_direction, events) in enumerate(lines_events):
        line_bits_hw = events_to_bits_np(events, bits_in_scanline, scanline_ticks)
        line_bits_img = line_bits_hw[::-1]
        lines[idx] = np.packbits(line_bits_img, bitorder="big")[:bytes_in_line]

    return facets_in_lane, lanes, lanewidth, lines.flatten()


def read_grayscale_binary_file(
    filepath: str | Path, laser_timing_cfg: dict, bits_in_scanline: int
) -> tuple[int, int, float, np.ndarray]:
    """Reads, decompresses, and reconstructs 8-bit ``0..255`` intensity samples from a ``.bin``/``.pat`` file."""
    facets_in_lane, lanes, lanewidth, lines_events = read_binary_events(filepath)

    scanline_ticks = int(laser_timing_cfg["scanline_ticks"])
    bits_in_scanline = int(bits_in_scanline)
    lines = np.zeros((len(lines_events), bits_in_scanline), dtype=np.uint8)

    for idx, (_direction, events) in enumerate(lines_events):
        line_vals_hw = events_to_intensities_np(events, bits_in_scanline, scanline_ticks)
        lines[idx] = line_vals_hw[::-1]

    return facets_in_lane, lanes, lanewidth, lines.flatten()
