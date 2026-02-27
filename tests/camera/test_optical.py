import time
from pathlib import Path
import logging
import json

import fire
import cv2 as cv
from tqdm import tqdm

import camera
from hexastorm.log_setup import configure_logging
from hexastorm.interpolator.patterns import camera as cam_patterns
from hexastorm.calibration import (
    calibration,
    get_dots,
    append_history_record,
)
from hexastorm.esp32_controller import ESP32Controller
from hexastorm.config import PlatformConfig

logger = logging.getLogger(__name__)


class StaticTests:
    """Optical test for scanhead

    Does not require facet synchronization
    shutter speed: it is assumed 100 units is ~1 ms
    """

    def __init__(self):
        # Default current matching original parameter
        current_val = 110

        self.cfg = PlatformConfig(test=False)

        self.cam = camera.Cam()
        self.cam.init()

        # Initialize Controller
        self.esp = ESP32Controller(timeout=4.0)

        # Define setup logic locally
        def setup_board(curr):
            global host
            from tools import lh as host

            host.laser_current = curr

        # Execute setup
        # We pass 'curr' as a direct argument to the function call
        self.esp.exec_func(setup_board, curr=current_val)

    def close(self):
        self.cam.close()
        self.esp.exec_wait("host.reset()")
        self.esp.close()

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
        self.take_picture(save=False)

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
            global host
            host.enable_comp(laser1=False, polygon=False)

        self.esp.exec_func(stop_spot)

    def take_picture(self, count=1, name=None, save=True):
        """
        Captures images using the camera class.

        Args:
            count (int): Number of images to capture.
            name (str): Filename to save as.
            save (bool): If True, saves to disk. If False, just returns the image object.
        """
        last_img = None
        for i in range(count):
            img = self.cam.capture()
            if img is None:
                continue

            last_img = img

            if save:
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

                file_path = self.cfg.paths["images"] / file_name
                cv.imwrite(str(file_path), grey)
                logger.info(f"Saved: {file_path.name}")

            if count > 1:
                time.sleep(1)

        return last_img


class DynamicTests(StaticTests):
    def __init__(self):
        super().__init__()

        def get_facet_zero_mapping():
            global host, spoll, sys
            import sys
            import uselect

            host.synchronize(True)
            spoll = uselect.poll()
            spoll.register(sys.stdin, uselect.POLLIN)

            true_facet = host.remap(0)
            return true_facet

        self.esp.exec_func(get_facet_zero_mapping)
        self.facet_zero = 0

        logger.info(f"TestDynamic Setup Complete. Facet 0 maps to: {self.facet_zero}")

    def picture_line(
        self, line, fct=None, preview=True, takepicture=True, name=None, save_image=True
    ):
        """
        Projects a line pattern on a specific facet and takes a picture.
        Uses serial handshaking (P -> R) to ensure synchronization.

        Args:
            save_image (bool): If True, saves the captured image to disk.
                               If False, keeps it in memory only.
        Returns:
            image (numpy array) or None
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
                    if cmd == "P":  # PICTURE REQUEST
                        # Signal PC we are ready/stable
                        sys.stdout.write("R")
                    elif cmd == "Q":  # QUIT REQUEST
                        return

        # Start the remote loop (wait=False is crucial here)
        self.esp.exec_func(remote_pattern, wait=False, line=line, fct=fct)

        self.cam.set_exposure(10_000)

        # 1. Request Picture
        self.esp.serial.reset_input_buffer()
        self.esp.serial.write(b"P")

        # 2. Wait for 'R' response (with timeout)
        start_time = time.time()
        ready = False
        while (time.time() - start_time) < 2.0:  # 3s timeout
            if self.esp.serial.in_waiting:
                if self.esp.serial.read(1) == b"R":
                    ready = True
                    break
            time.sleep(0.01)

        result_img = None
        if ready:
            if preview:
                self.cam.live_view(0.6)
            if takepicture:
                # Pass the save flag down to take_picture
                result_img = self.take_picture(count=1, name=name, save=save_image)
        else:
            raise Exception("Timeout: ESP32 did not respond with 'R'")

        self.esp.serial.write(b"Q")

        # We must confirm the ESP32 has fully exited the previous function
        # and returned to the REPL before we return control to the loop.
        start_drain = time.time()
        buffer = b""
        while (time.time() - start_drain) < 2.0:
            if self.esp.serial.in_waiting:
                chunk = self.esp.serial.read(self.esp.serial.in_waiting)
                buffer += chunk
                if b">>>" in buffer:  # Look for the MicroPython prompt
                    break
            time.sleep(0.01)

        return result_img

    def scan_visibility(self, facet=0, step=10):
        """
        Loops over all pixels in the scanline length.
        Projects a single pixel, takes a picture (no save), and checks visibility.

        Args:
            facet (int): The facet index to scan.
            step (int): The stride/interval between pixels (default=10).
        """
        scan_len = self.cfg.laser_timing["scanline_length"]
        visibility_map = []
        pixels_to_scan = list(range(0, scan_len, step))

        logger.info(
            f"Starting Pixel Visibility Scan. Total Pixels: {scan_len} (Step: {step})"
        )

        try:
            for i in tqdm(pixels_to_scan, desc="Scanning Pixels", unit="px"):
                # 1. Create a line with only the current pixel ON
                line = [0] * scan_len
                line[i] = 1

                # 2. Project and capture (Don't save to disk to save time/space)
                img = self.picture_line(
                    line=line,
                    fct=facet,
                    preview=False,
                    takepicture=True,
                    save_image=False,
                    name=None,
                )

                # 3. Analyze image
                is_visible = False
                if img is not None:
                    # Pass the in-memory image to get_dots
                    dots, _ = get_dots(img, debug=False)
                    if len(dots) > 0:
                        is_visible = True

                visibility_map.append(is_visible)

        except KeyboardInterrupt:
            logger.warning("Scan interrupted by user.")

        # Print final summary
        logger.info("-" * 30)
        logger.info("Visibility Scan Results:")
        logger.info("-" * 30)

        visible_indices = [idx * step for idx, vis in enumerate(visibility_map) if vis]

        # Calculate percentage
        if scan_len > 0:
            percent = (len(visible_indices) * step / scan_len) * 100
        else:
            percent = 0

        logger.info(
            f"Visible Pixels: {len(visible_indices) * step}/{scan_len} ({percent:.1f}%)"
        )
        logger.info(f"Visible Indices: {visible_indices}")

        payload = {
            "facet": facet,
            "step": step,
            "visible_pixels": visible_indices,
        }

        append_history_record("scan_visibility.json", payload)

    def scan_error(self):
        """
        Retrieves the scan by error by projecting a line
        and analyzing the resulting image.
        """
        pix_margin = 20

        history_file = self.cfg.paths["calibration"] / "scan_visibility.json"
        with open(history_file, "r") as f:
            history = json.load(f)
            last_record = history[-1]
            visible_pixels = last_record.get("visible_pixels", [])

        lower_pixel = min(visible_pixels) + pix_margin
        upper_pixel = max(visible_pixels) - pix_margin

        cam_pat = cam_patterns.CameraCalibrationGen(
            lower_pixel=lower_pixel, upper_pixel=upper_pixel, line_thickness_mm=0.150
        )
        cam_pat.generate_vertical_jitter_test()
        pattern_bits = cam_pat.interpolator.readbin()["data"]
        ptrn = pattern_bits.reshape(-1, self.cfg.laser_timing["scanline_length"])
        for facet in range(self.cfg.laser_timing["facets"]):
            line = ptrn[facet].tolist()
            line = line[::-1]  # Reverse the line
            self.picture_line(
                line=line,
                fct=facet,
                name=f"scan_error_facet_{facet}.jpg",
                preview=False,
            )

    def stability_pattern(self, facet=3, preview=True):
        """
        Line with a given pattern is projected and photo is taken.
        """
        assert facet in (0, 1, 2, 3), "facet must be 0, 1, 2, or 3"

        # Python list is compatible with MicroPython
        fname = f"facet{facet}.jpg"
        pat = [1] * 1 + [0] * 39
        bits = self.cfg.laser_timing["scanline_length"]
        # extend pattern to line
        line = pat * (bits // len(pat)) + pat[: bits % len(pat)]

        self.picture_line(line=line, fct=facet, name=fname, preview=preview)

    def full_calibration_cycle(self):
        """
        Automated sequence:
        1. Capture images for all 4 facets.
        2. Run calibration analysis.
        """
        capture = False
        num_facets = self.cfg.laser_timing["facets"]

        if capture:
            for facet in range(num_facets):
                logger.info(f"--- Capturing Calibration Image for Facet {facet} ---")
                self.stability_pattern(facet=facet, preview=False)

        logger.info("--- Starting Calibration Analysis ---")
        calibration(
            image_dir=None,  # Use default from config
            num_facets=num_facets,
            filename_pattern="facet{}.jpg",
            debug=False,
        )


class Launcher:
    """Main Entry Point."""

    def __init__(self):
        self._active_test = None

    @property
    def static(self):
        if self._active_test is None:
            self._active_test = StaticTests()
        return self._active_test

    @property
    def dynamic(self):
        if self._active_test is None:
            self._active_test = DynamicTests()
        return self._active_test

    def close(self):
        if self._active_test:
            self._active_test.close()
            self._active_test = None


def main():
    configure_logging(logging.DEBUG)
    tool = Launcher()
    try:
        fire.Fire(tool)
    finally:
        tool.close()


if __name__ == "__main__":
    main()
