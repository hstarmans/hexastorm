import unittest
from time import sleep

from gpiozero import LED, Button
from platforms import Firestarter
from driver import Driver


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls, build=True):
        platform = Firestarter()
        cls.reset_pin = LED(platform.reset_pin)
        cls.sensor_pin = Button(9)
        cls.enable_pin = LED(21)
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
            sleep(1)
            print(self.sensor_pin.value)
    
    def test_motor(self):
        self.enable_pin.on()
        while True:
            sleep(1)
            print(self.sensor_pin.value)

if __name__ == "__main__":
    unittest.main()