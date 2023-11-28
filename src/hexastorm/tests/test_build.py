import unittest

from ..core import Dispatcher, SPIParser
from ..movement import Polynomal
from ..platforms import Firestarter


class TestBuild(unittest.TestCase):
    def test_all(self):
        platform = Firestarter()
        platform.build(
            Dispatcher(Firestarter()), do_program=False, verbose=True
        )

    def parser(self):
        platform = Firestarter()
        platform.build(
            SPIParser(Firestarter(), top=True), do_program=False, verbose=True
        )

    def polynomal(self):
        platform = Firestarter()
        platform.build(
            Polynomal(Firestarter()), do_program=False, verbose=True
        )


if __name__ == "__main__":
    unittest.main()
