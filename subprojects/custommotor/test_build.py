import unittest

from driver import Driver
from platforms import Firestarter


class TestBuild(unittest.TestCase):
    def test_parser(self):
        platform = Firestarter()
        platform.build(
            Driver(Firestarter(), top=True), do_program=False, verbose=True
        )


if __name__ == "__main__":
    unittest.main()
