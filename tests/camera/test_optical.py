import time
import unittest
from pathlib import Path
import logging

import camera
import cv2 as cv

from hexastorm.calibration import run_full_calibration_analysis
from hexastorm.esp32_controller import ESP32Controller
from hexastorm.config import PlatformConfig

logger = logging.getLogger(__name__)

TEST_DIR = Path(__file__).parent.resolve()
IMG_DIR = TEST_DIR / "images"
TESTIMG_DIR = TEST_DIR / "testimages"


class Tests(unittest.TestCase):
    """Optical test for scanhead

    shutter speed: it is assumed 100 units is ~1 ms
    """

    @classmethod
    def setUpClass(cls):
        # Default current matching original parameter
        current_val = 110

        cls.cfg = PlatformConfig(test=False)

        cls.cam = camera.Cam()
        cls.cam.init()

        # Initialize Controller
        cls.esp = ESP32Controller(timeout=4.0)

        # Define setup logic locally
        def setup_board(curr):
            global host
            from tools import lh as host # noqa: F401 
            host.laser_current = curr

        # Execute setup
        # We pass 'curr' as a direct argument to the function call
        cls.esp.exec_func(setup_board, curr=current_val)

    @classmethod
    def tearDownClass(cls):
        cls.cam.close()
        try:
            if hasattr(cls, 'esp') and cls.esp:
                # Simple cleanup command
                cls.esp.exec_wait("host.reset()")
                cls.esp.close()
        except Exception as e:
            logger.warning(f"Error during tearDown: {e}")

    def align_laser(self):
        """
        Align laser with prism.
        Laser is aligned without camera.
        """
        def enable_laser():
            global host
            host.enable_comp(laser1=True)

        def disable_laser():
            global host
            host.enable_comp(laser1=False)

        self.esp.exec_func(enable_laser)

        logger.info("Press enter to confirm laser is aligned with prism")
        input()

        self.esp.exec_func(disable_laser)

    def photo_line(self):
        """
        Turn on laser and motor.
        User can first preview image. After pressing escape, a final image is taken.
        """
        def start_preview():
            global host
            host.enable_comp(laser0=True, polygon=True)

        self.esp.exec_func(start_preview)

        # 3000 rpm 4 facets --> 200 hertz
        # one facet per 1/200 = 5 ms
        self.cam.set_exposure(10_000)
        logger.info("This will open up a window")
        logger.info("Press escape to quit live view")

        self.cam.live_view(0.6)
        self.take_picture()

        def stop_preview():
            global host
            host.enable_comp(laser1=False, polygon=False)
            
        self.esp.exec_func(stop_preview)

    def photo_spot(self):
        """
        Turn on laser.
        User can first preview image. After pressing escape, a final image is taken.
        """
        def start_spot():
            global host
            host.enable_comp(laser1=True, polygon=False)

        self.esp.exec_func(start_spot)

        self.cam.set_exposure(300)
        logger.info(
            "Calibrate the camera with live view "
            "and press escape to confirm spot in vision"
        )
        self.cam.live_view(scale=0.6)
        self.take_picture()

        def stop_spot():
            host.enable_comp(laser1=False, polygon=False) # noqa: F821
            
        self.esp.exec_func(stop_spot)

    def take_picture(self, count=1, name=None):
        """
        Captures images using the camera class.
        """
        IMG_DIR.mkdir(parents=True, exist_ok=True)

        last_img = None
        for i in range(count):
            img = self.cam.capture()
            grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

            if name:
                if count > 1:
                    stem = Path(name).stem
                    ext = Path(name).suffix
                    file_name = f"{stem}_{i}{ext}"
                else:
                    file_name = name
            else:
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                file_name = f"{timestamp}.jpg"

            file_path = IMG_DIR / file_name

            cv.imwrite(str(file_path), grey)
            logger.info(f"Saved: {file_path}")
            last_img = img
            if count > 1:
                time.sleep(1)
        return last_img
    

class TestDynamic(Tests):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()

        def get_facet_zero_mapping():
            global host, spoll, sys
            import sys, uselect

            host.synchronize(True) 
            spoll = uselect.poll()
            spoll.register(sys.stdin, uselect.POLLIN)
            
            true_facet = host.remap(0) 
            return true_facet

        cls.esp.exec_func(get_facet_zero_mapping)
        cls.facet_zero = 0
        
        logger.info(f"TestDynamic Setup Complete. Facet 0 maps to: {cls.facet_zero}")

    def picture_line(self, line, fct=None, preview=True, takepicture=True, name=None):
        """
        Projects a line pattern on a specific facet and takes a picture.
        Uses serial handshaking (P -> R) to ensure synchronization.
        """
        # Default to the dynamically mapped facet 0 found in setUpClass
        if fct is None:
            fct = self.facet_zero
        
        # Default filename
        if name is None:
            name = f"dynamic_facet_{fct}.jpg"

        def remote_pattern(line, fct):
            # Imports required inside the function for ESP32 execution
            global host, spoll, sys
            
            chunk = host.cfg.hdl_cfg.lines_chunk
            true_facet = host.remap(fct, measure=False) 
            
            while True:
                host.write_line(line, repetitions=chunk, facet=true_facet) 
                
                # Check for incoming commands (non-blocking)
                if spoll.poll(0):
                    cmd = sys.stdin.read(1)
                    if cmd == 'P': # PICTURE REQUEST
                        # Signal PC we are ready/stable
                        sys.stdout.write('R') 
                    elif cmd == 'Q': # QUIT REQUEST
                        return

        # Start the remote loop (wait=False is crucial here)
        self.esp.exec_func(
            remote_pattern, 
            wait=False, 
            line=line, 
            fct=fct
        )

        self.cam.set_exposure(10_000)
        
        # 1. Request Picture
        self.esp.serial.reset_input_buffer()
        self.esp.serial.write(b'P')
        
        # 2. Wait for 'R' response (with timeout)
        start_time = time.time()
        ready = False
        while (time.time() - start_time) < 2.0: # 3s timeout
            if self.esp.serial.in_waiting:
                if self.esp.serial.read(1) == b'R':
                    ready = True
                    break
            time.sleep(0.01)

        if ready:
            if preview:
                self.cam.live_view(0.6)
            if takepicture:
                self.take_picture(count=1, name=name)
        else:
            raise Exception("Timeout: ESP32 did not respond with 'R'")

        self.esp.serial.write(b'Q')

    def stability_pattern(self, facet=3, preview=True):
        """
        Line with a given pattern is projected and photo is taken.
        """
        assert facet in (0, 1, 2, 3), "facet must be 0, 1, 2, or 3"

        # Python list is compatible with MicroPython
        fname = f"facet{facet}.jpg"
        pat = [1] * 1 + [0] * 39
        local_lines = 10_000
        bits = self.cfg.laser_timing["scanline_length"] 
        # extend pattern to line
        line = (pat * (bits // len(pat)) + pat[: bits % len(pat)])

        self.picture_line(line=line, fct=facet, name=fname, preview=preview)
    
    def full_calibration_cycle(self):
        """
        Automated sequence:
        1. Capture images for all 4 facets.
        2. Run calibration analysis.
        """
        capture = False
        num_facets = 4

        if capture:
            for facet in range(num_facets):
                logger.info(f"--- Capturing Calibration Image for Facet {i} ---")
                self.stability_pattern(facet=facet, preview=True)


        logger.info("--- Starting Calibration Analysis ---")
        run_full_calibration_analysis(
            image_dir=IMG_DIR,
            num_facets=num_facets,
            filename_pattern="facet{}.jpg",
            debug=False,
        )


if __name__ == "__main__":
    unittest.main()