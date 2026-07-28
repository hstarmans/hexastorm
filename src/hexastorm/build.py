import argparse

from hexastorm.config import PlatformConfig
from hexastorm.core import Dispatcher, SPIParser
from hexastorm.lasers import Laserhead
from hexastorm.movement import Polynomial
from hexastorm.platforms import Firestarter


def build(
    component: str = "dispatcher",
    program: bool = True,
    remote: str | None = None,
    device: str | None = None,
    verbose: bool = True,
):
    """
    Build and optionally program the FPGA bitstream.

    :param component: Component to synthesize ('dispatcher', 'parser', 'lasers', 'polynomial')
    :param program: Whether to program/flash the board after synthesis
    :param remote: Remote SSH host (e.g. 'hexastorm@pihome.home')
    :param device: Target serial device path on host (e.g. '/dev/ttyACM0')
    :param verbose: Verbose build logging
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
    platform.build(
        el,
        do_program=program,
        verbose=verbose,
    )


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
        help="Skip programming/flashing the FPGA bitstream after synthesis",
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
        help="Disable verbose synthesis output",
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
