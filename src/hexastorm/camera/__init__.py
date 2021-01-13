#NOTE: this code is no longer used, moved to arducam

from . import pyueye_utils
from pyueye import ueye

class Cam:
    def __init__(self):
        self.cam = pyueye_utils.Camera()
        if self.cam.init(): raise Exception("Can't init camera")
        self.cam.alloc()
        self.picnum = 0

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.cam.exit()

    def set_pixelclock(clock=20):
        camera.cam.set_pixelclock(20)

    def set_exposure(ms):
        camera.cam.set_exposure(ms)

    def take_picture(self):
        'returns image as numpy array'
        if self.cam.capture_video():
            raise Exception("Can't start image capture")
        img_buffer = pyueye_utils.ImageBuffer()
        if ueye.is_WaitForNextImage(self.cam.handle(),
                                    1000,
                                    img_buffer.mem_ptr,
                                    img_buffer.mem_id):
            raise Exception('Timeout on capture')
        image_data = pyueye_utils.ImageData(self.cam.handle(), img_buffer)
        img = image_data.as_1d_image()
        image_data.unlock()
        self.cam.stop_video()
        return img