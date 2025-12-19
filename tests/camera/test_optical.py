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

TEST_DIR = Path(__file__).parent.resolve()
IMG_DIR = TEST_DIR / "images"
TESTIMG_DIR = TEST_DIR / "testimages"


def micropython(instruction, nofollow=False):
    """Executes instruction using micropython on esp32s3 via mpremote."""
    with tempfile.NamedTemporaryFile(mode="w+t", suffix=".py") as temp_file:
        temp_file.write(inspect.cleandoc(instruction))
        temp_file.flush()

        cmd = ["mpremote", "resume", "run", temp_file.name]
        if nofollow:
            cmd.insert(-1, "--no-follow")

        try:
            # check=True automatically raises an exception on non-zero exit codes
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            if result.stdout:
                print(result.stdout)
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"mpremote execution failed: {e.stderr}")


class OpticalTest(unittest.TestCase):
    """Tests algorithms upon earlier taken images"""

    def test_laserline(self):
        img = cv.imread(str(TESTIMG_DIR / "laserline1.jpg"))
        line = feature.detect_line(img)
        expected = [0, 0, 1099, 719]
        # Use list comparison for cleaner failure messages
        self.assertEqual([int(v) for v in line], expected)

    def test_laserwidth(self):
        img = cv.imread(str(TESTIMG_DIR / "laserline.jpg"))
        actual = feature.cross_scan_error(img)
        expected = {"max": 86.36, "min": 21.58, "mean": 38.04, "median": 36.0}
        for k, v in expected.items():
            # Use almostEqual for floats to avoid precision issues
            self.assertAlmostEqual(actual[k], v, delta=1.0)

    def test_laserspot(self):
        """tests laser spot detection"""
        dct = {
            "laserspot1.jpg": np.array([26, 36]),
            "laserspot2.jpg": np.array([27, 41]),
        }
        for k, v in dct.items():
            img = cv.imread(str(Path(TESTIMG_DIR, k)))
            np.testing.assert_array_equal(feature.spotsize(img)["axes"].round(0), v)


class Tests(unittest.TestCase):
    """Optical test for scanhead

    shutter speed: it is assumed 10 units is 1 ms
    """

    @classmethod
    def setUpClass(cls):
        cls.cam = camera.Cam()
        cls.cam.init()
        micropython("from tools import hst")

    @classmethod
    def tearDownClass(cls):
        cls.cam.close()
        micropython("hst.reset()")

    def blinktest(self):
        """Verifies communication with ESP32 board."""
        micropython(
            """
            import machine, time
            led = machine.Pin(8, machine.Pin.OUT)
            led.off() # Red ON
            time.sleep(2)
            led.on()  # Red OFF
        """,
            nofollow=True,
        )
        time.sleep(2)
        # you can get back to the shell but not exit the program

    def alignlaser(self, current=80):
        """align laser with prism

        Laser is aligned without camera
        """
        micropython(f"""
            hst.laser_current = {current}
            hst.enable_comp(laser0=True)
        """)
        print("Press enter to confirm laser is aligned with prism")
        input()
        micropython("""
            hst.enable_comp(laser0=False)
        """)

    def photo_line(self, current=80):
        """turn on laser and motor

        User can first preview image. After pressing escape,
        a final image is taken.

        current: value between 0 and 255 (a.u.)
        """
        micropython(f"""
            hst.laser_current = {current}
            hst.enable_comp(laser0=True, polygon=True)
        """)
        # 3000 rpm 4 facets --> 200 hertz
        # one facet per  1/200 = 5 ms
        self.cam.set_exposure(700)
        print("This will open up a window")
        print("Press escape to quit live view")
        self.cam.live_view(0.6)
        self.takepicture()
        # img = self.takepicture()
        # print(feature.cross_scan_error(img))
        micropython("""
            hst.enable_comp(laser1=False, polygon=False)
        """)

    def photo_spot(self, current=80):
        """turn on laser
        User can first preview image. After pressing escape,
        a final image is taken.

        current: value between 0 and 255 (a.u.)
        """
        # NOTE: all ND filters and a single channel is used
        micropython(f"""
            hst.laser_current = {current}
            hst.enable_comp(laser1=True, polygon=False)
        """)
        self.cam.set_exposure(300)
        print(
            "Calibrate the camera with live view \
               and press escape to confirm spot in vision"
        )
        self.cam.live_view(scale=0.6)
        img = self.takepicture()
        print(feature.spotsize(img))
        micropython("""
            hst.enable_comp(laser1=False, polygon=False)
        """)

    def photo_pattern(self):
        """line with a given pattern is projected and photo is taken

        pattern  --  list of bits [0] or [1,0,0]
        """
        pattern = [1] * 1 + [0] * 39
        lines = 10_000
        micropython(
            f"""
            pattern = {pattern}
            bits = hst.cfg.laser_timing["scanline_length"]
            line = (pattern*(bits//len(pattern)) + pattern[: bits % len(pattern)])
            hst.write_line(line, repetitions={lines})
            """,
            nofollow=True,
        )
        self.cam.set_exposure(400)
        self.cam.live_view(0.6)
        self.takepicture(times=1)

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

    def take_picture(self, count=1):
        """Captures images and saves with a safe timestamp."""
        IMG_DIR.mkdir(parents=True, exist_ok=True)

        last_img = None
        for _ in range(count):
            img = self.cam.capture()
            grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            # Use '-' instead of ':' for Windows/File system compatibility
            timestamp = time.strftime("%Y%m%d-%H%M%S")
            file_path = IMG_DIR / f"{timestamp}.jpg"

            cv.imwrite(str(file_path), grey)
            print(f"Saved: {file_path}")
            last_img = img
            if count > 1:
                time.sleep(1)
        return last_img


if __name__ == "__main__":
    unittest.main()
