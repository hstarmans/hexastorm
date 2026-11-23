import sys

from .core import (
    PrismCurveLite as PrismCurveLite,
)
from .core import (
    VirtualStackerUlab as VirtualStackerUlab,
)
from .core import (
    analyze_facets_ulab as analyze_facets_ulab,
)
from .core import (
    calculate_facet_shifts_ulab as calculate_facet_shifts_ulab,
)
from .core import (
    compute_angle_ulab as compute_angle_ulab,
)
from .core import (
    get_dots_ulab as get_dots_ulab,
)
from .core import (
    make_dot_centers_mm as make_dot_centers_mm,
)
from .core import (
    make_tick_dot_events as make_tick_dot_events,
)
from .core import (
    rotate_dots_ulab as rotate_dots_ulab,
)
from .core import (
    slice_dot_grid_events as slice_dot_grid_events,
)
from .core import (
    subtract_dark_frame_u8 as subtract_dark_frame_u8,
)
from .core import (
    summarize_facet_dots_ulab as summarize_facet_dots_ulab,
)
from .vector_lite import (
    HAS_VECTOR_CORE as HAS_VECTOR_CORE,
)
from .vector_lite import (
    circle_polygon as circle_polygon,
)
from .vector_lite import (
    decode_spi_scanlines as decode_spi_scanlines,
)
from .vector_lite import (
    slice_dot_grid_vector_core as slice_dot_grid_vector_core,
)
from .vector_lite import (
    slice_polygons_to_spi as slice_polygons_to_spi,
)

__all__ = [
    "HAS_VECTOR_CORE",
    "PrismCurveLite",
    "VirtualStackerUlab",
    "analyze_facets_ulab",
    "calculate_facet_shifts_ulab",
    "circle_polygon",
    "compute_angle_ulab",
    "decode_spi_scanlines",
    "get_dots_ulab",
    "make_dot_centers_mm",
    "make_tick_dot_events",
    "rotate_dots_ulab",
    "slice_dot_grid_events",
    "slice_dot_grid_vector_core",
    "slice_polygons_to_spi",
    "subtract_dark_frame_u8",
    "summarize_facet_dots_ulab",
]

if sys.implementation.name != "micropython":
    from .simulator import FakeScanhead as FakeScanhead

    __all__.append("FakeScanhead")

    try:
        from .analyzer import (
            append_history_record as append_history_record,
        )
        from .analyzer import (
            calibration as calibration,
        )
        from .analyzer import (
            get_dots as get_dots,
        )
        from .analyzer import (
            update_statistical_history as update_statistical_history,
        )
        from .stack import LaserStackSimulator as LaserStackSimulator

        __all__.extend(
            [
                "LaserStackSimulator",
                "append_history_record",
                "calibration",
                "get_dots",
                "update_statistical_history",
            ]
        )
    except ImportError:
        pass
