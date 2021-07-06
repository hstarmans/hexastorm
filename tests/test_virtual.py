import unittest

from hexastorm.platforms import Firestarter
from hexastorm.core import Dispatcher, SPIParser
from hexastorm.movement import Polynomal


class TestBuild(unittest.TestCase):
    def test_parser(self):
        platform = Firestarter()
        platform.build(SPIParser(Firestarter(), top=True),
                       do_program=False, verbose=True)

    def test_dispatcher(self):
        platform = Firestarter()
        platform.build(Dispatcher(Firestarter()),
                       do_program=False, verbose=True)

    def test_polynomal(self):
        platform = Firestarter()
        platform.build(Polynomal(Firestarter()), do_program=False,
                       verbose=True)


if __name__ == "__main__":
    unittest.main()
