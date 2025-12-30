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
    with tempfile.NamedTemporaryFile(mode="w+t", suffix=".py", delete=True) as temp_file:
        # Clean and write the instruction
        code = inspect.cleandoc(instruction)
        temp_file.write(code)
        temp_file.flush()

        cmd = ["mpremote", "resume", "run", temp_file.name]
        if nofollow:
            cmd.insert(-1, "--no-follow")

        try:
            # We capture both stdout and stderr
            result = subprocess.run(
                cmd, 
                capture_output=True, 
                text=True, 
                check=False # We handle the check manually for better detail
            )

            # 1. Check for Shell/Connection Errors (Exit Code)
            if result.returncode != 0:
                error_msg = result.stderr.strip() or result.stdout.strip()
                raise RuntimeError(f"mpremote shell error (Code {result.returncode}):\n{error_msg}")

            # 2. Check for MicroPython Exceptions inside stdout
            # MicroPython errors usually contain "Traceback" or "Error:"
            if "Traceback (most recent call last):" in result.stdout:
                raise RuntimeError(f"MicroPython Script Error:\n{result.stdout}")

            if result.stdout and not nofollow:
                print(f"MicroPython Output:\n{result.stdout}")

        except FileNotFoundError:
            raise RuntimeError("mpremote not found. Is it installed and in your PATH?")
        except Exception as e:
            raise RuntimeError(f"Unexpected error during execution: {e}")


class Tests(unittest.TestCase):
    """Optical test for scanhead

    shutter speed: it is assumed 100 units is ~1 ms
    """

    @classmethod
    def setUpClass(cls, current=110):
        cls.cam = camera.Cam()
        cls.cam.init()
        micropython(f"""from tools import lh as host
                        host.laser_current = {current}
        """)

    @classmethod
    def tearDownClass(cls):
        cls.cam.close()
        micropython("host.reset()")

    def invalidcommand_test(self):
        """Invalid commands can only be tested with nofollow is False
        
        If you set no follow to true it simply disconnects
        ."""
        micropython(
            """
            lh.superfunctie()
        """,
            nofollow=False,
        )
        time.sleep(2)


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
            
            host.enable_comp(laser0=True)
        """)
        print("Press enter to confirm laser is aligned with prism")
        input()
        micropython("""
            host.enable_comp(laser0=False)
        """)

    def photo_line(self, current=80):
        """turn on laser and motor

        User can first preview image. After pressing escape,
        a final image is taken.

        current: value between 0 and 255 (a.u.)
        """
        micropython(f"""
            host.enable_comp(laser0=True, polygon=True)
        """)
        # 3000 rpm 4 facets --> 200 hertz
        # one facet per  1/200 = 5 ms
        self.cam.set_exposure(10_000)
        print("This will open up a window")
        print("Press escape to quit live view")
        self.cam.live_view(0.6)
        self.take_picture()
        # img = self.take_picture()
        # print(feature.cross_scan_error(img))
        micropython("""
            host.enable_comp(laser1=False, polygon=False)
        """)

    def photo_spot(self, current=80):
        """turn on laser
        User can first preview image. After pressing escape,
        a final image is taken.

        current: value between 0 and 255 (a.u.)
        """
        # NOTE: all ND filters and a single channel is used
        micropython(f"""
            host.enable_comp(laser1=True, polygon=False)
        """)
        self.cam.set_exposure(300)
        print(
            "Calibrate the camera with live view \
               and press escape to confirm spot in vision"
        )
        self.cam.live_view(scale=0.6)
        img = self.take_picture()
        print(feature.spotsize(img))
        micropython("""
            host.enable_comp(laser1=False, polygon=False)
        """)

    def photo_pattern(self, facet=3):
        """line with a given pattern is projected and photo is taken

        pattern  --  list of bits [0] or [1,0,0]
        """
        pattern = [1] * 1 + [0] * 39
        lines = 10_000
        micropython(
            f"""
            pattern = {pattern}
            bits = host.cfg.laser_timing["scanline_length"]
            line = (pattern*(bits//len(pattern)) + pattern[: bits % len(pattern)])
            host.synchronize(True)
            if {facet}:
                shft = host.facet_shift()
                facet = ({facet} + shft) % host.cfg.laser_timing["facets"]
            else:
                facet = {facet}
            host.write_line(line, repetitions={lines}, facet=facet)
            """,
            nofollow=True,
        )
        self.cam.set_exposure(10_000)
        self.cam.live_view(0.6)
        self.take_picture(count=1)

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
