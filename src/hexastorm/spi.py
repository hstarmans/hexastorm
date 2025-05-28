#
# This file is part of LUNA.
#
# Copyright (c) 2020 Great Scott Gadgets <info@greatscottgadgets.com>
# SPDX-License-Identifier: BSD-3-Clause

""" SPI and derived interfaces. """

from .utils import LunaGatewareTestCase


class SPIGatewareTestCase(LunaGatewareTestCase):
    """ Extended version of the LunaGatewareTestCase.

    Adds three SPI-simulation methods:
        - spi_send_bit
        - spi_exchange_byte
        - spi_exchange_data
    """

    async def spi_send_bit(self, sim, bit):
        """ Sends a single bit over the SPI bus. """
        cycles_per_bit = 4
        spi = self.dut.spi

        # Apply the new bit...
        if hasattr(spi, 'sdi'):
            sim.set(spi.sdi, bit)
            await self.advance_cycles(sim, cycles_per_bit)

        # Rising edge of serial clock
        sim.set(spi.sck, 1)
        await self.advance_cycles(sim, cycles_per_bit)

        # Sample the output bit
        return_value = sim.get(spi.sdo)
        await self.advance_cycles(sim, cycles_per_bit)

        # Falling edge of serial clock
        sim.set(spi.sck, 0)
        await self.advance_cycles(sim, cycles_per_bit)

        return return_value

    async def spi_exchange_byte(self, sim, datum, *, msb_first=True):
        """ Sends a byte over the virtual SPI bus. """
        bits = "{:08b}".format(datum)
        data_received = ""

        if not msb_first:
            bits = bits[::-1]

        for bit in bits:
            received = await self.spi_send_bit(sim, int(bit))
            data_received += '1' if received else '0'

        if not msb_first:
            data_received = data_received[::-1]

        return int(data_received, 2)

    async def spi_exchange_data(self, sim, data, msb_first=True):
        """ Sends a list of bytes over our virtual SPI bus. """
        sim.set(self.dut.spi.cs, 1)
        await sim.tick()

        response = bytearray()
        for byte in data:
            response_byte = await self.spi_exchange_byte(sim, byte, msb_first=msb_first)
            response.append(response_byte)

        sim.set(self.dut.spi.cs, 0)
        await sim.tick()

        return response