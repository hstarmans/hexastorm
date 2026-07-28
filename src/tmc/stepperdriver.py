"""Dummy tmc.stepperdriver module for CPython / development compatibility."""


class TMC_2209:
    def __init__(self, *args, **kwargs):
        pass

    def set_microsteps(self, *args, **kwargs):
        pass

    def set_current(self, *args, **kwargs):
        pass

    def read_register(self, *args, **kwargs):
        return 0

    def write_register(self, *args, **kwargs):
        pass
