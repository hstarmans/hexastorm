from .interface import BaseHost
from ..core import Dispatcher
from ..platforms import Firestarter
# from ..motor import Driver


class TestHost(BaseHost):
    """
    Host interface to interact with the FPGA for Amaranth HDL tests.
    """

    def __init__(self):
        super().__init__(test=True)
        self.spi_tries = 10

    def build(self, do_program=False, verbose=True, mod="all"):
        """
        Builds the FPGA code using Amaranth HDL, Yosys, Nextpnr, and Icepack.

        Parameters:
        - do_program (bool): If True, flashes the FPGA using fomu-flash and resets afterwards.
        - verbose (bool): If True, prints output of Yosys, Nextpnr, and Icepack.
        - mod (str): Specifies which module to build. Options: 'all', 'motor'.
        """
        platform = Firestarter()
        if mod == "all":
            module = Dispatcher(platform)
        # elif mod == "motor":
        #     module = Driver(platform, top=True)
        else:
            raise Exception(f"Print building {mod} is not supported.")
        platform.build(
            module,
            do_program=do_program,
            verbose=verbose,
        )
