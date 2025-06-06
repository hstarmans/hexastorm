import unittest

from hexastorm.core import Dispatcher, SPIParser

# from hexastorm.movement import Polynomial
from hexastorm.platforms import Firestarter


class TestBuild(unittest.TestCase):
    """Platform is passed to object which is then built"""

    # build directory created in folder python is executed
    def test_all(self):
        platform = Firestarter()
        platform.build(
            Dispatcher(Firestarter()),
            do_program=False,
            verbose=True,
        )

    def parser(self):
        platform = Firestarter()
        platform.build(
            SPIParser(Firestarter(), top=True), do_program=False, verbose=True
        )

    def polynomal(self):
        platform = Firestarter()
        platform.build(Polynomial(Firestarter()), do_program=False, verbose=True)


if __name__ == "__main__":
    unittest.main()
