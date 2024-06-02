import unittest

from ..core import Dispatcher, SPIParser
from ..movement import Polynomal
from ..platforms import Firestarter


class TestBuild(unittest.TestCase):
    """Platform is passed to object which is then built"""

    def test_all(self, micropython=False):
        platform = Firestarter(micropython)
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
        platform.build(
            Polynomal(Firestarter()), do_program=False, verbose=True
        )


if __name__ == "__main__":
    unittest.main()
