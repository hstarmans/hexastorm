import time
from pathlib import Path
import logging
import json

import fire
import cv2 as cv
from tqdm import tqdm
import numpy as np

import camera
from hexastorm.log_setup import configure_logging
from hexastorm.interpolator.patterns import camera as cam_patterns
from hexastorm.calibration import (
    calibration,
    get_dots,
    append_history_record,
    LaserStackSimulator,
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
            grey = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            if img is None:
                continue

            last_img = grey

            if save:
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

    def _setup_camera_calibration(self):
        """
        Helper method to initialize the camera calibration generator
        and return shared sizing constants to avoid repetition.
        """
        pix_margin = 20
        visible_pixels = self.cfg.get_visible_pixels()

        lower_pixel = min(visible_pixels) + pix_margin
        upper_pixel = max(visible_pixels) - pix_margin

        line_thickness_mm = 0.150
        min_size_mm = 0.050
        max_size_mm = 0.200

        cam_pat = cam_patterns.CameraCalibrationGen(
            lower_pixel=lower_pixel,
            upper_pixel=upper_pixel,
            line_thickness_mm=line_thickness_mm,
        )
        return cam_pat, min_size_mm, max_size_mm

    def pattern_error(self):
        """
        Measures the scan and orthogonal error by projecting a 2D dot grid pattern per facet.

        This method generates an SVG pattern and exposes it onto the camera chip.
        It simulates the physical movement of the stage by stacking multiple virtual
        exposures to form a 2D dot pattern. The calibration process then verifies
        if the corresponding dots align at their expected locations, both with and
        without optical correction.
        """
        cam_pat, min_size_mm, max_size_mm = self._setup_camera_calibration()
        fpattern = "pattern_error_facet_{}_{}.jpg"
        calib_dct = self.cfg.load_latest_calibration()

        def dispatch(correction, storelog=False):
            facets = self.cfg.laser_timing["facets"]
            for facet in range(facets):
                if correction:
                    correc_dct = {}
                    for i in range(facets):
                        correc_dct[str(i)] = {
                            "scan": calib_dct[facet]["scan"],
                            "orth": calib_dct[facet]["orth"],
                        }
                else:
                    correc_dct = False

                cam_pat.set_correction(correction=correc_dct)
                cam_pat.generate_dot_grid_test()
                pattern_bits = cam_pat.interpolator.readbin()["data"]
                ptrn = pattern_bits.reshape(
                    -1, self.cfg.laser_timing["scanline_length"]
                )
                lines = len(ptrn)
                stacker = LaserStackSimulator(expected_total_steps=lines)
                for row in range(lines):
                    line = ptrn[row].tolist()[::-1]  # Reverse the line
                    if max(line) == 0:  # skip empty liines
                        continue
                    img = self.picture_line(
                        line=line, fct=facet, save_image=False, preview=False
                    )
                    stacker.add_exposures(img, num_steps=row)
                res = stacker.get_result()
                file_path = self.cfg.paths["images"] / fpattern.format(
                    facet, bool(correction)
                )
                cv.imwrite(str(file_path), res)
            results = calibration(
                filename_pattern=fpattern.format("{}", bool(correction)),
                min_diameter_mm=min_size_mm,
                max_diameter_mm=max_size_mm,
                debug=False,
                store_log=storelog,
                compute_rotation=False,
            )
            return [
                    [
                        float(results[str(facet)]["Scan_shift_um"]),
                        float(results[str(facet)]["Orth_shift_um"]),
                    ]
                    for facet in range(facets)
                ]

        errors = dispatch(correction=False)
        errors_correct = dispatch(correction=True)

        errors = dispatch(correction=False)
        errors_correct = dispatch(correction=True)

        logger.info(f"Uncorrected 2D Errors (um) [Scan, Orth]: {errors}")
        logger.info(f"Corrected 2D Errors (um) [Scan, Orth]:   {errors_correct}")

        # Convert to numpy arrays (Shape: [num_facets, 2])
        err_before = np.array(errors)
        err_after = np.array(errors_correct)

        abs_before = np.abs(err_before)
        abs_after = np.abs(err_after)

        # Calculate overall worst-case error across BOTH scan and orth axes
        max_err_before = np.max(abs_before)
        max_err_after = np.max(abs_after)

        # Calculate overall Mean Absolute Error (MAE)
        mae_before = np.mean(abs_before)
        mae_after = np.mean(abs_after)

        # Extract axis-specific max errors for detailed logging
        max_scan_after = np.max(abs_after[:, 0])
        max_orth_after = np.max(abs_after[:, 1])

        logger.info("\n--- 2D Calibration Summary ---")
        logger.info(
            f"Overall MAE Before: {mae_before:.2f} um | MAE After: {mae_after:.2f} um"
        )
        logger.info(
            f"Overall Max Error Before: {max_err_before:.2f} um | Max Error After: {max_err_after:.2f} um"
        )
        logger.info(
            f"Post-Correction Max Scan: {max_scan_after:.2f} um | Max Orth: {max_orth_after:.2f} um"
        )

        # 1. Assert the system actually improved overall
        assert max_err_after < max_err_before, (
            "Calibration Failure: Correction increased the maximum 2D error!"
        )

        # 2. Assert the system meets the final engineering tolerance
        TOLERANCE_UM = 30.0
        assert max_err_after <= TOLERANCE_UM, (
            f"Tolerance Failure: Worst facet 2D error ({max_err_after:.2f} um) exceeds {TOLERANCE_UM} um limit."
        )

        logger.info(
            f"SUCCESS: 2D System calibrated to within +/- {TOLERANCE_UM} microns."
        )

        return errors, errors_correct

    def line_error(self):
        """
        Measures the 1D scan error by projecting a vertical line pattern.

        Generates an SVG representing a set of vertical lines. For each facet,
        the first line of this pattern is projected and captured. The center of
        the resulting dots should ideally align across facets. The alignment is
        measured to evaluate the improvement provided by optical correction.
        """
        cam_pat, min_size_mm, max_size_mm = self._setup_camera_calibration()
        fpattern = "scan_error_facet_{}.jpg"

        def dispatch(
            correction,
            storelog=False,
        ):
            cam_pat.set_correction(correction=correction)
            cam_pat.generate_vertical_jitter_test()
            pattern_bits = cam_pat.interpolator.readbin()["data"]
            ptrn = pattern_bits.reshape(-1, self.cfg.laser_timing["scanline_length"])
            facets = self.cfg.laser_timing["facets"]
            for facet in range(facets):
                offset = 16 # orthogonal error can move lines of image
                line = ptrn[facet+offset].tolist()[::-1]
                self.picture_line(
                    line=line, fct=facet, name=fpattern.format(facet), preview=False
                )
            results = calibration(
                filename_pattern=fpattern,
                min_diameter_mm=min_size_mm,
                max_diameter_mm=max_size_mm,
                debug=False,
                store_log=storelog,
            )
            return [float(results[str(facet)]["Scan_shift_um"]) for facet in range(facets)]

        scan_errors = dispatch(correction=False)
        scan_errors_new = dispatch(correction=True)

        logger.info(f"Uncorrected Errors (um): {scan_errors}")
        logger.info(f"Corrected Errors (um):   {scan_errors_new}")

        # --- Mechanical Validation ---
        # Convert to numpy arrays and take the absolute values
        abs_before = np.abs(scan_errors)
        abs_after = np.abs(scan_errors_new)

        # Calculate the worst-case error for both runs
        max_err_before = np.max(abs_before)
        max_err_after = np.max(abs_after)

        # Calculate Mean Absolute Error (MAE) for reporting
        mae_before = np.mean(abs_before)
        mae_after = np.mean(abs_after)

        logger.info("\n--- Calibration Summary ---")
        logger.info(f"MAE Before: {mae_before:.2f} um | MAE After: {mae_after:.2f} um")
        logger.info(
            f"Max Error Before: {max_err_before:.2f} um | Max Error After: {max_err_after:.2f} um"
        )

        # 1. Assert the system actually improved
        assert max_err_after < max_err_before, (
            "Calibration Failure: Correction increased the maximum error!"
        )

        # 2. Assert the system meets the final engineering tolerance
        # (Setting a strict 30-micron limit based on your results)
        TOLERANCE_UM = 30.0
        assert max_err_after <= TOLERANCE_UM, (
            f"Tolerance Failure: Worst facet ({max_err_after:.2f} um) exceeds {TOLERANCE_UM} um limit."
        )

        logger.info(f"SUCCESS: System calibrated to within +/- {TOLERANCE_UM} microns.")

    def direct_pattern(self, facet=3, preview=True):
        """
        Repeating line pattern is exposed on to the camera
        using the full scanline. The pattern is bigger than the camera.

        Args:
            facet (int): The index of the polygon scanner facet to expose.
            preview (bool): If True, displays a preview of the captured image.
        """
        num_facets = self.cfg.laser_timing["facets"]

        assert facet in range(num_facets), (
            f"Facet must be between 0 and {num_facets - 1}"
        )

        fname = f"facet{facet}.jpg"
        pat = [1] * 1 + [0] * 39
        bits = self.cfg.laser_timing["scanline_length"]
        # extend pattern to cover full line
        line = pat * (bits // len(pat)) + pat[: bits % len(pat)]

        self.picture_line(line=line, fct=facet, name=fname, preview=preview)

    def baseline_calibration_cycle(self, capture=True):
        """
        Executes an automated direct calibration sequence for all facets.

        This acts as a baseline hardware test, bypassing the slicer to project
        a raw repeating pattern and analyze the resulting optical alignment.

        Args:
            capture (bool): If True, physical hardware will capture new images
                            for all facets. If False, the analysis will run
                            on the existing images in the directory.

        Returns:
            dict: The calibration results payload.
        """
        num_facets = self.cfg.laser_timing["facets"]

        if capture:
            for facet in range(num_facets):
                logger.info(f"--- Capturing Calibration Image for Facet {facet} ---")
                self.direct_pattern(facet=facet, preview=False)

        logger.info("--- Starting Calibration Analysis ---")
        calibration(
            image_dir=None,  # Use default from config
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
