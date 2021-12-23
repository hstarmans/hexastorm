import unittest
from time import sleep

from gpiozero import LED
from gpiozero.output_devices import Buzzer
from platforms import Firestarter
from driver import Driver


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls, build=False):
        platform = Firestarter()
        cls.reset_pin = LED(platform.reset_pin)
        cls.sensor_pin = Buzzer('19')
        cls.enable_pin = LED('21')
        if build:
            print("Building and programming board")
            platform = Firestarter()
            platform.build(Driver(Firestarter(), top=True),
                    do_program=True, verbose=True)
        else:
            print("Not programming board")

    def test_sensor(self):
        print('Reading sensor')
        while True:
            sleep(5)
            print(self.sensor_pin.value)

if __name__ == "__main__":
    unittest.main()