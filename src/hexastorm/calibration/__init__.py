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

__all__ = [
    "PrismCurveLite",
    "VirtualStackerUlab",
    "analyze_facets_ulab",
    "calculate_facet_shifts_ulab",
    "compute_angle_ulab",
    "get_dots_ulab",
    "make_dot_centers_mm",
    "make_tick_dot_events",
    "rotate_dots_ulab",
    "slice_dot_grid_events",
    "subtract_dark_frame_u8",
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
