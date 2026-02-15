#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

"""SPI and derived interfaces."""

from amaranth import Signal, Module, Cat, Elaboratable
from ..utils import LunaGatewareTestCase


class SPIBus:
    """
    Modern replacement for the luna SPIBus Record.
    Uses simple Signals to avoid 'amaranth.hdl.rec' deprecation warnings.
    """

    def __init__(self):
        self.sck = Signal(name="sck")
        self.sdi = Signal(name="sdi")
        self.sdo = Signal(name="sdo")
        self.cs = Signal(name="cs")


class SPICommandInterface(Elaboratable):
    """Variant of an SPIDeviceInterface that accepts command-prefixed data.

    I/O signals:
        I: sck           -- SPI clock, from the SPI master
        I: sdi           -- SPI data in
        O: sdo           -- SPI data out
        I: cs            -- chip select, active high (internal logic)

        O: command       -- the command read from the SPI bus
        O: command_ready -- a new command is ready

        O: word_received -- the most recent word received
        O: word_complete -- strobe indicating a new word is present on word_in
        I: word_to_send  -- the word to be loaded; latched in on next word_complete and while cs is low

        O: idle          -- true iff the register interface is currently doing nothing
        O: stalled       -- true iff the register interface cannot accept data until this transaction ends
    """

    def __init__(self, command_size=8, word_size=32):
        self.command_size = command_size
        self.word_size = word_size

        #
        # I/O port.
        #

        # SPI
        self.spi = SPIBus()

        # Command I/O.
        self.command = Signal(self.command_size)
        self.command_ready = Signal()

        # Data I/O
        self.word_received = Signal(self.word_size)
        self.word_to_send = Signal.like(self.word_received)
        self.word_complete = Signal()

        # Status
        self.idle = Signal()
        self.stalled = Signal()

    def elaborate(self, platform):
        m = Module()
        spi = self.spi

        # Detect falling edge of SPI clock
        past_sck = Signal()
        m.d.sync += past_sck.eq(spi.sck)
        sample_edge = past_sck & ~spi.sck

        # Bit counter: counts the number of bits received.
        max_bit_count = max(self.word_size, self.command_size)
        bit_count = Signal(range(0, max_bit_count + 1))

        # Shift registers for our command and data.
        current_command = Signal.like(self.command)
        current_word = Signal.like(self.word_received)

        # De-assert our control signals unless explicitly asserted.
        m.d.sync += [self.command_ready.eq(0), self.word_complete.eq(0)]

        with m.FSM() as fsm:
            m.d.comb += [
                self.idle.eq(fsm.ongoing("IDLE")),
                self.stalled.eq(fsm.ongoing("STALL")),
            ]

            # STALL: entered when we can't accept new bits -- either when
            # CS starts asserted, or when we've received more data than expected.
            with m.State("STALL"):
                # Wait for CS to clear.
                with m.If(~spi.cs):
                    m.next = "IDLE"

            # We ignore all data until chip select is asserted, as that data Isn't For Us (TM).
            # We'll spin and do nothing until the bus-master addresses us.
            with m.State("IDLE"):
                m.d.sync += bit_count.eq(0)

                with m.If(spi.cs):
                    m.next = "RECEIVE_COMMAND"

            # Once CS is low (active high here), we'll shift in our command.
            with m.State("RECEIVE_COMMAND"):
                # If CS is de-asserted early; our transaction is being aborted.
                with m.If(~spi.cs):
                    m.next = "IDLE"

                # Continue shifting in data until we have a full command.
                with m.If(bit_count < self.command_size):
                    with m.If(sample_edge):
                        m.d.sync += [
                            bit_count.eq(bit_count + 1),
                            current_command.eq(Cat(spi.sdi, current_command[:-1])),
                        ]

                # ... and then pass that command out to our controller.
                with m.Else():
                    m.d.sync += [
                        bit_count.eq(0),
                        self.command_ready.eq(1),
                        self.command.eq(current_command),
                    ]
                    m.next = "PROCESSING"

            # Give our controller a wait state to prepare any response...
            with m.State("PROCESSING"):
                m.next = "LATCH_OUTPUT"

            # ... and then latch in the response to transmit.
            with m.State("LATCH_OUTPUT"):
                m.d.sync += current_word.eq(self.word_to_send)
                m.next = "SHIFT_DATA"

            # Finally, exchange data.
            with m.State("SHIFT_DATA"):
                # If CS is de-asserted early; our transaction is being aborted.
                with m.If(~spi.cs):
                    m.next = "IDLE"

                m.d.sync += spi.sdo.eq(current_word[-1])

                # Continue shifting data until we have a full word.
                with m.If(bit_count < self.word_size):
                    with m.If(sample_edge):
                        m.d.sync += [
                            bit_count.eq(bit_count + 1),
                            current_word.eq(Cat(spi.sdi, current_word[:-1])),
                        ]

                # ... and then output that word on our bus.
                with m.Else():
                    m.d.sync += [
                        bit_count.eq(0),
                        self.word_complete.eq(1),
                        self.word_received.eq(current_word),
                    ]

                    # Go back to receiving more commands
                    m.next = "RECEIVE_COMMAND"

        return m


class SPIGatewareTestCase(LunaGatewareTestCase):
    """Extended version of the LunaGatewareTestCase.

    Adds three SPI-simulation methods:
        - spi_send_bit
        - spi_exchange_byte
        - spi_exchange_data
    """

    async def spi_send_bit(self, bit):
        """Sends a single bit over the SPI bus."""
        cycles_per_bit = 4
        spi = self.dut.spi
        sim = self.sim

        # Apply the new bit...
        sim.set(spi.sdi, bit)
        await self.advance_cycles(cycles_per_bit)

        # Rising edge of serial clock
        sim.set(spi.sck, 1)
        await self.advance_cycles(cycles_per_bit)

        # Sample the output bit
        return_value = sim.get(spi.sdo)
        await self.advance_cycles(cycles_per_bit)

        # Falling edge of serial clock
        sim.set(spi.sck, 0)
        await self.advance_cycles(cycles_per_bit)

        return return_value

    async def spi_exchange_byte(self, datum, *, msb_first=True):
        """Sends a byte over the virtual SPI bus."""
        bits = "{:08b}".format(datum)
        data_received = ""

        if not msb_first:
            bits = bits[::-1]

        for bit in bits:
            received = await self.spi_send_bit(int(bit))
            data_received += "1" if received else "0"

        if not msb_first:
            data_received = data_received[::-1]

        return int(data_received, 2)

    async def spi_exchange_data(self, data, msb_first=True):
        """Sends a list of bytes over our virtual SPI bus."""
        sim = self.sim
        sim.set(self.dut.spi.cs, 1)
        await sim.tick()

        response = bytearray()
        for byte in data:
            response_byte = await self.spi_exchange_byte(byte, msb_first=msb_first)
            response.append(response_byte)

        sim.set(self.dut.spi.cs, 0)
        await sim.tick()

        return response
