import argparse
from pathlib import Path

from hexastorm.config import PlatformConfig
from hexastorm.core import Dispatcher, SPIParser
from hexastorm.lasers import Laserhead
from hexastorm.movement import Polynomial
from hexastorm.platforms import Firestarter


def _log_build_summary(build_dir: Path) -> None:
    """Print nextpnr resource utilization and clock frequency from ``top.tim``."""
    tim_path = build_dir / "top.tim"
    if not tim_path.exists():
        return

    lc_line = None
    ram_line = None
    fmax_line = None
    for line in tim_path.read_text(encoding="utf-8").splitlines():
        if "ICESTORM_LC:" in line:
            lc_line = line.replace("Info:", "").strip()
        elif "ICESTORM_RAM:" in line:
            ram_line = line.replace("Info:", "").strip()
        elif "Max frequency for clock" in line:
            fmax_line = line.replace("Info:", "").strip()

    for item in (lc_line, ram_line, fmax_line):
        if item:
            print(item)


def build(
    component: str = "dispatcher",
    program: bool = True,
    remote: str | None = None,
    device: str | None = None,
    verbose: bool = False,
):
    """
    Build and optionally program the FPGA bitstream.

    :param component: Component to synthesize ('dispatcher', 'parser', 'lasers', 'polynomial')
    :param program: Whether to program/flash the board after synthesis
    :param remote: Remote SSH host (e.g. 'hexastorm@pihome.home')
    :param device: Target serial device path on host (e.g. '/dev/ttyACM0')
    :param verbose: Verbose build logging (defaults to False to avoid yowasp-yosys PTY buffer traps)
    """
    platform = Firestarter(remote_host=remote, device=device)
    cfg = PlatformConfig(test=False)

    components = {
        "dispatcher": lambda: Dispatcher(cfg),
        "parser": lambda: SPIParser(cfg.hdl_cfg),
        "lasers": lambda: Laserhead(cfg),
        "polynomial": lambda: Polynomial(cfg),
    }

    if component.lower() not in components:
        raise ValueError(
            f"Unknown component '{component}'. Available: {list(components.keys())}"
        )

    el = components[component.lower()]()
    build_dir = Path("build")
    platform.build(
        el,
        build_dir=str(build_dir),
        do_program=program,
        verbose=verbose,
    )
    if not verbose:
        _log_build_summary(build_dir)


def main():
    parser = argparse.ArgumentParser(
        description="Hexastorm FPGA Bitstream Build & Flashing Tool"
    )
    parser.add_argument(
        "--component",
        "-c",
        default="dispatcher",
        choices=["dispatcher", "parser", "lasers", "polynomial"],
        help="Target component to build (default: dispatcher)",
    )
    parser.add_argument(
        "--no-program",
        dest="program",
        action="store_false",
        help="Skip programming/flashing the board after synthesis",
    )
    parser.add_argument(
        "--program",
        dest="program",
        action="store_true",
        default=True,
        help="Program/flash the FPGA bitstream after synthesis (default)",
    )
    parser.add_argument(
        "--remote",
        "-r",
        default=None,
        help="Remote SSH host to flash (e.g. hexastorm@pihome.home)",
    )
    parser.add_argument(
        "--device",
        "-d",
        default=None,
        help="Target serial device on local/remote host (default: /dev/ttyACM0)",
    )
    parser.add_argument(
        "--quiet",
        "-q",
        dest="verbose",
        action="store_false",
        default=False,
        help="Disable verbose synthesis output (default)",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        dest="verbose",
        action="store_true",
        help="Enable verbose synthesis output",
    )

    args = parser.parse_args()
    build(
        component=args.component,
        program=args.program,
        remote=args.remote,
        device=args.device,
        verbose=args.verbose,
    )


if __name__ == "__main__":
    main()
