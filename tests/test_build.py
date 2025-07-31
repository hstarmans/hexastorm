import unittest

from hexastorm.core import Dispatcher, SPIParser

from hexastorm.movement import Polynomial
from hexastorm.lasers import Laserhead
from hexastorm.platforms import Firestarter
from hexastorm.config import PlatformConfig


class TestBuild(unittest.TestCase):
    """Platform is passed to object which is then built"""

    # build directory created in folder python is executed
    def test_all(self):
        platform = Firestarter()
        platform.build(
            Dispatcher(PlatformConfig(test=False)),
            do_program=False,
            verbose=True,
        )

    def parser(self):
        platform = Firestarter()
        platform.build(
            SPIParser(PlatformConfig(test=False).hdl_cfg),
            do_program=False,
            verbose=True,
        )

    def lasers(self):
        platform = Firestarter()
        platform.build(
            Laserhead(PlatformConfig(test=False)),
            do_program=False,
            verbose=True,
        )

    def polynomial(self):
        platform = Firestarter()
        platform.build(
            Polynomial(PlatformConfig(test=False)),
            do_program=False,
            verbose=True,
        )


if __name__ == "__main__":
    unittest.main()
