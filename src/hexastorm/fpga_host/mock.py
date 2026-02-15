from asyncio import TimeoutError

from .interface import BaseHost


class MockHost(BaseHost):
    """
    Host interface to interact with the FPGA for Amaranth HDL tests.
    """

    def __init__(self, fifo_full, sim):
        super().__init__(test=True)
        self.spi_tries = 10
        self.fifo_full = fifo_full
        self.sim = sim

    async def send_command(self, command, timeout=0):
        command = bytearray(command)
        response = bytearray(command)

        if timeout and self.sim.get(self.fifo_full):
            trial = 0
            while self.sim.get(self.fifo_full):
                trial += 1
                await self.fpga_state
                if trial >= self.spi_tries - 1:
                    raise TimeoutError
        # function is created in Amaranth HDL
        response[:] = await self.spi_exchange_data(command)
        return response
