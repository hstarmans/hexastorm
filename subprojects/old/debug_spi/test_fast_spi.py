from amaranth.sim import Simulator
from spi_interface_fast import SPIDeviceInterfaceFast


def test_spi_slave():
    dut = SPIDeviceInterfaceFast(word_size=8)

    sim = Simulator(dut)
    sim.add_clock(1 / 24e6)  # 24 MHz system clock

    def spi_driver():
        # SPI pattern: master drives sck, sdi, cs
        # Send 0b10101100 (0xAC), expect same echoed back
        yield dut.spi.cs.eq(0)
        data = 0b10101100
        for i in range(8):
            bit = (data >> (7 - i)) & 1
            yield dut.spi.sdi.eq(bit)
            # SPI mode 0: sample rising edge
            yield dut.spi.sck.eq(0)
            yield
            yield dut.spi.sck.eq(1)
            yield
        yield dut.spi.cs.eq(1)
        for _ in range(5):
            yield

    def monitor():
        # Load outgoing word
        yield dut.word_out.eq(0x55)
        yield
        while True:
            wc = yield dut.word_complete
            if wc:
                print(f"Word received: {yield dut.word_in:08b}")
            yield

    sim.add_sync_process(spi_driver)
    sim.add_sync_process(monitor)

    sim.run_until(5e-6, run_passive=True)


if __name__ == "__main__":
    test_spi_slave()
