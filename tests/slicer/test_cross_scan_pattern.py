from pathlib import Path

import numpy as np
import pytest

from hexastorm.interpolator.patterns.cross_scan import CrossScanPatternGen


def test_cross_scan_pattern_uncorrected(tmp_path):
    gen = CrossScanPatternGen(exposures=1, correction=False)
    out_bin = tmp_path / "test_cross_scan_uncorrected.bin"

    result = gen.generate_pattern(
        dash_len_bits=50,
        gap_len_bits=50,
        margin_bits=20,
        filename=out_bin,
    )

    assert "data" in result
    assert "metadata" in result
    assert result["metadata"]["lanes"] == 4
    assert result["metadata"]["facetsinlane"] == 4

    data = result["data"]
    assert len(data) > 0
    assert np.any(data == 1)


def test_cross_scan_pattern_large_optical_correction(tmp_path):
    """
    Tests pattern generation and plot rendering with a large custom optical correction dictionary.
    Simulates large orthogonal offsets (e.g., +100 um, +200 um, -150 um).
    """
    huge_correction = {
        "0": {"scan": 0.0, "orth": 0.0},
        "1": {"scan": 0.030, "orth": 0.100},  # +100 um orthogonal shift
        "2": {"scan": -0.020, "orth": 0.200},  # +200 um orthogonal shift
        "3": {"scan": 0.050, "orth": -0.150},  # -150 um orthogonal shift
    }

    gen = CrossScanPatternGen(exposures=2, correction=huge_correction)
    out_bin = tmp_path / "test_cross_scan_corrected.bin"

    result = gen.generate_pattern(
        dash_len_bits=80,
        gap_len_bits=40,
        margin_bits=30,
        filename=out_bin,
    )

    assert result["metadata"]["lanes"] == 4
    assert result["metadata"]["facetsinlane"] == 8  # 2 exposures * 4 facets

    # Verify that plotptrn executes cleanly with large corrections
    plot_canvas = gen.interpolator.plotptrn(result, filename="test_large_correction_plot")
    assert plot_canvas is not None
    assert plot_canvas.ndim == 3  # RGB image canvas
    assert np.any(plot_canvas > 0)  # Contains non-black rendered pixels
