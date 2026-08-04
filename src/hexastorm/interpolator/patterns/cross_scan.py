import logging
from pathlib import Path
from typing import Any

import fire
import numpy as np

from ...config import PlatformConfig
from ..interpolator import Interpolator

logger = logging.getLogger(__name__)


class CrossScanPatternGen:
    """
    Generates direct-printing pattern files (.bin / .pat) to measure cross-scan
    (orthogonal) facet errors directly on substrate.

    The pattern exposes 4 lanes:
    - Lane 0: Facet 0 exposed N times (Solid reference line).
    - Lane 1: Facet 0 (Solid line) + Facet 1 (Dashed line), exposed N times.
    - Lane 2: Facet 0 (Solid line) + Facet 2 (Dashed line), exposed N times.
    - Lane 3: Facet 0 (Solid line) + Facet 3 (Dashed line), exposed N times.
    """

    def __init__(self, exposures: int = 1, correction: bool | dict = False):
        self.cfg = PlatformConfig(test=False)
        self.exposures = exposures
        self.correction = correction
        self.interpolator = Interpolator(correction=correction, exposures=exposures)

    def set_correction(
        self, correction: bool | dict | None = None, exposures: int | None = None
    ):
        """Enables or updates optical correction parameters."""
        self.interpolator.set_optical_params(
            correction=correction, exposures=exposures
        )

    def generate_pattern(
        self,
        dash_len_bits: int = 100,
        gap_len_bits: int = 100,
        margin_bits: int = 50,
        filename: str | Path = "cross_scan_test.bin",
    ) -> dict:
        """
        Generates the 4-lane cross-scan test pattern, writes it to binary file,
        and saves a plot visualization.

        Args:
            dash_len_bits (int): Number of ON bits per dash.
            gap_len_bits (int): Number of OFF bits between dashes.
            margin_bits (int): Margin offset from scanline edges.
            filename (str | Path): Output destination filename.

        Returns:
            dict: The pattern data dictionary returned by readbin().
        """
        bitsinscanline = int(self.interpolator.params["bitsinscanline"])
        num_facets = int(self.cfg.laser_timing["facets"])
        exposures = self.interpolator.exposures
        lanes = 4

        active_start = margin_bits
        active_end = bitsinscanline - margin_bits

        if active_end <= active_start:
            raise ValueError(
                f"margin_bits ({margin_bits}) is too large for bitsinscanline ({bitsinscanline})"
            )

        active_len = active_end - active_start

        # Create dashed pattern array for target facets
        dash_cycle = [1] * dash_len_bits + [0] * gap_len_bits
        cycle_len = len(dash_cycle)
        if cycle_len <= 0:
            raise ValueError("dash_len_bits + gap_len_bits must be > 0")

        dashed_pattern = np.array(
            (dash_cycle * (active_len // cycle_len + 1))[:active_len],
            dtype=np.uint8,
        )

        total_lines = lanes * exposures * num_facets
        ptrn = np.zeros((total_lines, bitsinscanline), dtype=np.uint8)

        # Fill bit matrix for each lane
        for lane_idx in range(lanes):
            for exp_idx in range(exposures):
                # Facet 0 line index: Always solid line
                f0_line = (lane_idx * exposures + exp_idx) * num_facets + 0
                ptrn[f0_line, active_start:active_end] = 1

                # Target facet for this lane (if lane_idx > 0)
                if lane_idx > 0:
                    fk_line = (lane_idx * exposures + exp_idx) * num_facets + lane_idx
                    ptrn[fk_line, active_start:active_end] = dashed_pattern

        # Update interpolator params for proper geometry calculation & file header formatting
        self.interpolator.params["lanes"] = float(lanes)
        self.interpolator.params["samplexsize"] = float(
            lanes * self.interpolator.params["lanewidth"]
        )
        self.interpolator.params["facetsinlane"] = float(exposures * num_facets)

        # Flatten and pack bits
        factor = int(self.interpolator.params["downsamplefactor"])
        ptrn_flat = np.repeat(ptrn.flatten(), factor)
        ptrn_packed = np.packbits(ptrn_flat, bitorder=self.interpolator.bitorder)

        # Write binary pattern file
        logger.info(
            f"Generating cross-scan pattern: lanes={lanes}, exposures={exposures}, "
            f"dash_len={dash_len_bits}, gap_len={gap_len_bits}"
        )
        self.interpolator.writebin(ptrn_packed, filename)

        # Read back and plot pattern
        pattern_data = self.interpolator.readbin()
        plot_name = Path(filename).stem
        self.interpolator.plotptrn(pattern_data, filename=plot_name)

        return pattern_data


def main(
    exposures: int = 1,
    dash_len_bits: int = 100,
    gap_len_bits: int = 100,
    margin_bits: int = 50,
    filename: str = "cross_scan_test.bin",
    correction: bool = False,
):
    """CLI entry point to generate cross-scan pattern files directly."""
    from ...log_setup import configure_logging

    configure_logging(logging.INFO)
    gen = CrossScanPatternGen(exposures=exposures, correction=correction)
    result = gen.generate_pattern(
        dash_len_bits=dash_len_bits,
        gap_len_bits=gap_len_bits,
        margin_bits=margin_bits,
        filename=filename,
    )
    logger.info("Cross-scan pattern generation complete.")
    return result


if __name__ == "__main__":
    fire.Fire(main)
