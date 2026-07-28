"""Dummy machine module for CPython / development compatibility."""


class Pin:
    IN = 0
    OUT = 1
    PULL_UP = 1
    PULL_DOWN = 2
    OPEN_DRAIN = 3

    def __init__(self, id, mode=-1, pull=-1, *, value=None, drive=-1, alt=-1):
        self.id = id
        self.mode = mode
        self.pull = pull
        self._val = value or 0

    def value(self, val=None):
        if val is not None:
            self._val = val
        return self._val

    def on(self):
        self._val = 1

    def off(self):
        self._val = 0


class SPI:
    MSB = 0
    LSB = 1

    def __init__(
        self,
        id=-1,
        baudrate=1000000,
        *,
        polarity=0,
        phase=0,
        bits=8,
        firstbit=MSB,
        sck=None,
        mosi=None,
        miso=None,
    ):
        self.baudrate = baudrate

    def read(self, nbytes, write=0x00):
        return bytes(nbytes)

    def readinto(self, buf, write=0x00):
        pass

    def write(self, buf):
        pass

    def write_readinto(self, write_buf, read_buf):
        pass


class I2C:
    def __init__(self, id=-1, *, scl=None, sda=None, freq=400000):
        self.freq = freq

    def readfrom(self, addr, nbytes, stop=True):
        return bytes(nbytes)

    def readfrom_into(self, addr, buf, stop=True):
        pass

    def writeto(self, addr, buf, stop=True):
        return len(buf)


class PWM:
    def __init__(self, dest, freq=-1, duty=-1, *, duty_ns=-1, duty_u16=-1):
        self._freq = freq
        self._duty = duty

    def freq(self, value=None):
        if value is not None:
            self._freq = value
        return self._freq

    def duty(self, value=None):
        if value is not None:
            self._duty = value
        return self._duty

    def deinit(self):
        pass
