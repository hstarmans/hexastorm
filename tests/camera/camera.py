import os

# Vertel Qt dat hij de systeemfonts moet gebruiken in plaats van de ontbrekende venv fonts
os.environ["QT_QPA_FONTDIR"] = "/usr/share/fonts"
import cv2 as cv
import subprocess
import logging
import time


logger = logging.getLogger(__name__)


class Cam:
    LINE_TIME_MS = 0.0189
    MAX_EXPECTED_MS = 100.0

    def __init__(self, device_index=0, width=1600, height=1300):

        self.device_index = device_index
        self.width = width
        self.height = height
        self.video_node = f"/dev/video{device_index}"
        self.subdev_node = "/dev/v4l-subdev0"
        self.cap = None
        self._current_exposure_ms = 300  # Standaard waarde

    def _run_hw_cmd(self, cmd):
        """Execute v4l2 commands without crashing on stderr."""
        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as e:
            logger.error(f"V4L2 Error ({cmd[0]}): {e.stderr.strip()}")

    def init(self):
        """Standardized initialization based on verified working script."""
        logger.info(f"Opening OV2311 on /dev/video{self.device_index}...")

        # 1. Open the device first
        self.cap = cv.VideoCapture(self.device_index, cv.CAP_V4L2)

        # 2. Set format using the 'GREY' FOURCC that worked in your script
        # Note: If 'GREY' doesn't work, try 'Y8  ' (with 2 spaces)
        self.cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*"GREY"))
        self.cap.set(cv.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv.CAP_PROP_FRAME_HEIGHT, self.height)

        # 3. Buffer size 1 is key for laser scanning to avoid 'old' frames
        self.cap.set(cv.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open /dev/video{self.device_index}")

        # 4. Flush the first 2-3 frames to clear the driver's internal buffer
        for _ in range(3):
            self.cap.grab()

        logger.info("Camera initialized and buffer flushed.")

    def set_exposure_ms(self, exposure_ms):
        """Sets hardware exposure and adjusts vblank to allow long shutter speeds."""
        self._exposure_ms = exposure_ms
        lines = max(1, int(exposure_ms / self.LINE_TIME_MS))

        # 1. Disable Auto Exposure
        subprocess.run(
            ["v4l2-ctl", "-d", "/dev/v4l-subdev0", "-c", "exposure_auto=1"],
            capture_output=True,
        )

        # 2. Adjust VBLANK if exposure is long.
        # If lines > current frame length, we must increase vblank to prevent the driver from capping exposure.
        # For 1000ms, we need at least 53000 lines total.
        if lines > 1000:
            vblank_value = lines + 500  # Margin
            subprocess.run(
                [
                    "v4l2-ctl",
                    "-d",
                    "/dev/v4l-subdev0",
                    "-c",
                    f"vertical_blanking={vblank_value}",
                ],
                capture_output=True,
            )
            logger.info(f"VBlank increased to {vblank_value} to support long exposure.")

        # 3. Apply the actual exposure lines
        self._run_hw_cmd(
            ["v4l2-ctl", "-d", "/dev/v4l-subdev0", "-c", f"exposure={lines}"]
        )

        logger.info(f"Hardware exposure locked at {exposure_ms}ms ({lines} lines)")

    def capture(self):
        """
        Capture a frame. For long exposures, we manually retry
        to compensate for the lack of a backend timeout.
        """
        # Hoe lang moeten we maximaal wachten? (Exposure + 1 seconde marge)
        max_wait = (self._current_exposure_ms / 1000.0) + 1.0
        start_time = time.time()

        while (time.time() - start_time) < max_wait:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                return frame

            # Geef de CPU wat ademruimte tijdens het wachten
            time.sleep(0.1)

        logger.error(f"Capture timed out after {max_wait:.1f}s")
        return None

    def live_view(self, scale=1.0):
        logger.info(f"Starting live view (Scale: {scale}). Press ESC to stop.")
        window_name = "Hexastorm Live View"

        # WINDOW_NORMAL laat ons het venster handmatig herschalen
        cv.namedWindow(window_name, cv.WINDOW_NORMAL)

        # Bereken de doelgrootte op basis van de sensorresolutie
        target_w = int(self.width * scale)
        target_h = int(self.height * scale)

        # Forceer het venster naar deze grootte op je scherm
        cv.resizeWindow(window_name, target_w, target_h)

        try:
            while True:
                frame = self.capture()
                if frame is not None:
                    # We laten OpenCV de schaling van de pixels afhandelen IN het venster
                    # Dit is vaak sneller en soepeler dan cv.resize() op elke frame-loop
                    cv.imshow(window_name, frame)

                key = cv.waitKey(1) & 0xFF
                if key == 27 or key == ord("q"):
                    break
        finally:
            cv.destroyWindow(window_name)
            for i in range(10):
                cv.waitKey(1)

    def close(self):
        if self.cap:
            self.cap.release()
