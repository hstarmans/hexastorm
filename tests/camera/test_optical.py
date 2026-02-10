import time
import unittest
from pathlib import Path
import logging

import camera
import cv2 as cv

from hexastorm.calibration import run_full_calibration_analysis
from hexastorm.esp32_controller import ESP32Controller

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

        cls.cam = camera.Cam()
        cls.cam.init()

        # Initialize Controller
        cls.esp = ESP32Controller(timeout=4.0)

        # Define setup logic locally
        def setup_board():
            # We import here so the linter sees it, 
            # and so it runs on the ESP32 side.
            from tools import lh as host 
            # Inject 'curr' via kwargs below
            host.laser_current = curr  # noqa: F821

        # Execute setup
        # We pass 'curr' to avoid f-string injection
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
            host.enable_comp(laser1=True) # noqa: F821

        def disable_laser():
            host.enable_comp(laser1=False) # noqa: F821

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
            host.enable_comp(laser0=True, polygon=True) # noqa: F821

        self.esp.exec_func(start_preview)

        # 3000 rpm 4 facets --> 200 hertz
        # one facet per 1/200 = 5 ms
        self.cam.set_exposure(10_000)
        logger.info("This will open up a window")
        logger.info("Press escape to quit live view")

        self.cam.live_view(0.6)
        self.take_picture()

        def stop_preview():
            host.enable_comp(laser1=False, polygon=False) # noqa: F821
            
        self.esp.exec_func(stop_preview)

    def photo_spot(self):
        """
        Turn on laser.
        User can first preview image. After pressing escape, a final image is taken.
        """
        def start_spot():
            # NOTE: all ND filters and a single channel is used
            host.enable_comp(laser1=True, polygon=False) # noqa: F821

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

    def photo_pattern(self, facet=3, preview=True):
        """
        Line with a given pattern is projected and photo is taken.
        """
        assert facet in (0, 1, 2, 3), "facet must be 0, 1, 2, or 3"

        # Python list is compatible with MicroPython
        fname = f"facet{facet}.jpg"
        local_pattern = [1] * 1 + [0] * 39
        local_lines = 10_000

        def remote_pattern():
            # Variables 'pat', 'reps', 'fct' are injected via kwargs
            # 'host' is assumed available globally on ESP32
            
            # Note: The linter will flag 'pat', 'reps', 'fct' as undefined
            # unless we ignore them or trick the linter. 
            # Since this function is never run locally, ignoring is safe.
            
            bits = host.cfg.laser_timing["scanline_length"] # noqa: F821
            
            # Use 'pat' injected from kwargs
            line = (pat * (bits // len(pat)) + pat[: bits % len(pat)]) # noqa: F821
            
            host.synchronize(True) # noqa: F821
            true_facet = host.remap(fct) # noqa: F821
            
            host.write_line(line, repetitions=reps, facet=true_facet) # noqa: F821

        # Execute using exec_func
        # wait=False because this might take time/run async
        self.esp.exec_func(
            remote_pattern, 
            wait=False, 
            pat=local_pattern, 
            reps=local_lines, 
            fct=facet
        )

        logger.info("Wait for pattern to start...")
        time.sleep(4)

        if preview:
            self.cam.set_exposure(10_000)
            self.cam.live_view(0.6)

        self.take_picture(count=1, name=fname)

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

    def test_full_calibration_cycle(self):
        """
        Automated sequence:
        1. Capture images for all 4 facets.
        2. Run calibration analysis.
        """
        capture = False
        num_facets = 4

        if capture:
            for i in range(num_facets):
                logger.info(f"--- Capturing Calibration Image for Facet {i} ---")

                self.photo_pattern(facet=i, name=f"facet{i}.jpg", preview=False)

                # Reset logic if necessary between shots
                self.esp.exec_wait("host.reset()")
                
                # Re-setup parameters if reset() cleared them
                def reset_current():
                    host.laser_current = 110 # noqa: F821
                self.esp.exec_func(reset_current)

        logger.info("--- Starting Calibration Analysis ---")
        run_full_calibration_analysis(
            image_dir=IMG_DIR,
            num_facets=num_facets,
            filename_pattern="facet{}.jpg",
            debug=False,
        )


if __name__ == "__main__":
    unittest.main()