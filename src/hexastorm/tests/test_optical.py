import os
import time
import unittest
import subprocess
import tempfile
from pathlib import Path
import inspect

import camera
import cv2 as cv
import numpy as np

import hexastorm.optical as feature
from hexastorm.constants import params
from hexastorm.platforms import Firestarter

TEST_DIR = Path(__file__).parents[0].resolve()
IMG_DIR = Path(TEST_DIR, "images")
TESTIMG_DIR = Path(TEST_DIR, "testimages")


def micropython(instruction, nofollow=False):
    """executes instruction using micropython on eps32s3
    
    command is executed by micropython interpreter connected via usb cable
    to the raspberry pi

    nofollow: if True, return immediately and leave the device running the script
    """
    with tempfile.NamedTemporaryFile(mode='w+t', delete=False) as temp_file:
        temp_file.write(inspect.cleandoc(instruction))
        temp_file_path = temp_file.name

    try:
        shell_command = ["mpremote", "resume", "run", temp_file_path]
        if nofollow:
            shell_command.insert(-1, "--no-follow")
        process = subprocess.Popen(shell_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        stdout, stderr = process.communicate()

        if process.returncode != 0:
            raise Exception(f"Error executing mpremote: {stderr.decode()}")
        else:
            print(stdout.decode())

    finally:
        os.unlink(temp_file_path) # Delete the temporary file.



class OpticalTest(unittest.TestCase):
    """Tests algorithms upon earlier taken images"""

    def test_laserline(self):
        """tests laser line detection"""
        img = cv.imread(str(Path(TESTIMG_DIR, "laserline1.jpg")))
        line = [vx, vy, x, y] = feature.detect_line(img)
        res = [0, 0, 1099, 719]
        for idx, val in enumerate(line):
            self.assertEqual(int(val), int(res[idx]))

    def test_laserwidth(self):
        """tests laser width detection"""
        img = cv.imread(str(Path(TESTIMG_DIR, "laserline.jpg")))
        dct = feature.cross_scan_error(img)
        dct2 = {"max": 86.36, "min": 21.58, "mean": 38.04, "median": 36.0}
        for k, v in dct.items():
            self.assertEqual(int(v), int(dct2[k]))

    def test_laserspot(self):
        """tests laser spot detection"""
        dct = {
            "laserspot1.jpg": np.array([26, 36]),
            "laserspot2.jpg": np.array([27, 41]),
        }
        for k, v in dct.items():
            img = cv.imread(str(Path(TESTIMG_DIR, k)))
            np.testing.assert_array_equal(
                feature.spotsize(img)["axes"].round(0), v
            )


class Tests(unittest.TestCase):
    """Optical test for scanhead"""

    @classmethod
    def setUpClass(cls):
        cls.cam = camera.Cam()
        cls.cam.init()
        micropython("""
            from control.laserhead import Laserhead
            lh = Laserhead()
        """
        )

    @classmethod
    def tearDownClass(cls):
        cls.cam.close()
        micropython("""
            lh.reset_state()
            lh.reset_fpga()
        """
        )

    def blinktest(self):
        """tries to blink red light on ESP32 using micropython shell

        Verifies communication with ESP32 board
        """
        micropython("""
            import machine
            import time
            led = machine.Pin(8, machine.Pin.OUT)
            print('LED ON')
            led.off()
            time.sleep(5)
            print('LED OFF')
            led.on()
        """, nofollow=True
        )
        print("Light should be red")
        time.sleep(5)
        print("Light should be blue")
        # you can get back to the shell but not exit the program

    def alignlaser(self, current=80):
        """align laser with prism

        Laser is aligned without camera
        """
        micropython(f"""
            lh.laser_current = {current}
            lh.enable_comp(laser0=True)
        """
        )
        print("Press enter to confirm laser is aligned with prism")
        input()
        micropython("""
            lh.enable_comp(laser0=False)
        """
        )

    def photo_line(self, current=80):
        """turn on laser and motor

        User can first preview image. After pressing escape,
        a final image is taken.

        current: value between 0 and 255 (a.u.)
        """
        micropython(f"""
            lh.laser_current = {current}
            lh.enable_comp(laser0=True, polygon=True)
        """
        )
        # 3000 rpm 4 facets --> 200 hertz
        # one facet per  1/200 = 5 ms
        self.cam.set_exposure(1400)
        print("This will open up a window")
        print("Press escape to quit live view")
        self.cam.live_view(0.6)
        self.takepicture()
        # img = self.takepicture()
        # print(feature.cross_scan_error(img))
        micropython("""
            lh.enable_comp(laser1=False, polygon=False)
        """
        )

    def photo_spot(self, current=80):
        """turn on laser
        User can first preview image. After pressing escape,
        a final image is taken.

        current: value between 0 and 255 (a.u.)
        """
        # NOTE: all ND filters and a single channel is used
        micropython(f"""
            lh.laser_current = {current}
            lh.enable_comp(laser1=True, polygon=False)
        """
        )
        self.cam.set_exposure(1300)
        print(
            "Calibrate the camera with live view \
               and press escape to confirm spot in vision"
        )
        self.cam.live_view(scale=0.6)
        img = self.takepicture()
        print(feature.spotsize(img))
        micropython("""
            lh.enable_comp(laser1=False, polygon=False)
        """
        )

    def photo_pattern(self):
        """line with a given pattern is projected and photo is taken

        pattern  --  list of bits [0] or [1,0,0]
        """
        pattern = [1] * 1 + [0] * 39
        lines = 10000
        platf = Firestarter(micropython=True)
        laser_params = params(platf)
        bits = int(laser_params["BITSINSCANLINE"])
        micropython(f"""
            pattern = {pattern}
            bits = lh.host.laser_params["BITSINSCANLINE"]
            line = (pattern*({bits}//len(pattern)) + pattern[: {bits} % len(pattern)])
            lh.write_line(line, repititions={lines})
            """, nofollow=True)
        self.cam.set_exposure(1400)
        self.cam.live_view(0.6)
        self.takepicture(times=10)

    # @executor
    # def searchcamera(self, timeout=3, build=False):
    #     """laser is synced with photodiode and a line is projected

    #     This is done for various line patterns and is
    #     used to detect the edges of the camera
    #     """
    #     # self.host.laser_params['SINGLE_LINE'] = True
    #     # self.host.laser_params['SINGLE_FACET'] = False
    #     if build:
    #         self.host.build()
    #     self.host.laser_current = 120
    #     # self.cam.set_exposure(36000)
    #     # yield from self.writepattern([0]*8+[1]*8)
    #     yield from self.host.enable_comp(laser1=True, polygon=False)
    #     self.cam.live_view(scale=0.6)
    #     # TODO: it doesn't catch the stopline
    #     # yield from self.host.writeline([])
    #     print(f"Wait for stopline to execute, {timeout} seconds")
    #     yield from self.host.enable_comp(laser1=False, polygon=False)
    #     # time.sleep(timeout)
    #     # yield from self.host.enable_comp(synchronize=False)

    def takepicture(self, times=1):
        "takes picture and store it with timestamp to this folder"
        for _ in range(times):
            img = self.cam.capture()
            grey_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            date_string = time.strftime("%Y-%m-%d-%H:%M:%S")
            print(f"Writing to {Path(IMG_DIR, date_string+'.jpg')}")
            if not os.path.exists(IMG_DIR):
                os.makedirs(IMG_DIR)
            cv.imwrite(str(Path(IMG_DIR, date_string + ".jpg")), grey_img)
            time.sleep(1)
        return img


if __name__ == "__main__":
    unittest.main()
