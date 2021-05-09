import unittest
from pathlib import Path
import time
import os

import cv2 as cv
import numpy as np

import camera
from FPGAG.controller import Host, executor
import FPGAG.optical as feature

TEST_DIR = Path(__file__).parents[0].resolve()
IMG_DIR = Path(TEST_DIR, 'images')
TESTIMG_DIR = Path(TEST_DIR, 'testimages')


class OpticalTest(unittest.TestCase):
    '''Tests algorithms upon earlier taken images'''

    def test_laserline(self):
        '''tests laser line detection
        '''
        img = cv.imread(str(Path(TESTIMG_DIR, 'laserline1.jpg')))
        line = [vx, vy, x, y] = feature.detect_line(img)
        res = [0, 0, 1099, 719]
        for idx, val in enumerate(line):
            self.assertEqual(int(val), int(res[idx]))

    def test_laserwidth(self):
        '''tests laser width detection
        '''
        img = cv.imread(str(Path(TESTIMG_DIR, 'laserline.jpg')))
        dct = feature.cross_scan_error(img)
        dct2 = {'max': 86.36, 'min': 21.58, 'mean': 38.04, 'median': 36.0}
        for k, v in dct.items():
            self.assertEqual(int(v), int(dct2[k]))

    def test_laserspot(self):
        '''tests laser spot detection
        '''
        dct = {'laserspot1.jpg': np.array([26, 36]),
               'laserspot2.jpg': np.array([27, 41])}
        for k, v in dct.items():
            img = cv.imread(str(Path(TESTIMG_DIR, k)))
            np.testing.assert_array_equal(
                feature.spotsize(img)['axes'].round(0), v)


class Tests(unittest.TestCase):
    ''' Optical test for scanhead'''
    @classmethod
    def setUpClass(cls, flash=False):
        cls.host = Host()
        if flash:
            cls.host.build()
        else:
            print('Resetting the machine')
            cls.host.reset()
        cls.cam = camera.Cam()
        cls.cam.init()

    @classmethod
    def tearDownClass(cls):
        cls.cam.close()

    @executor
    def alignlaser(self):
        '''align laser with prism

        Laser is aligned without camera
        '''
        yield from self.host.enable_comp(laser1=True)
        print("Press enter to confirm laser is aligned with prism")
        input()
        yield from self.host.enable_comp(laser1=False)

    @executor
    def grabline(self):
        '''turn on laser and motor
        User can first preview image. After pressing escape,
        a final image is taken.
        '''
        yield from self.host.enable_comp(laser1=True, polygon=True)
        self.host.laser_power = 120
        self.cam.set_exposure(36000)
        print("This will open up a window")
        print("Press escape to quit live view")
        self.cam.live_view(0.6)
        img = self.takepicture()
        print(feature.cross_scan_error(img))
        yield from self.host.enable_comp(laser1=False, polygon=False)

    @executor
    def grabspot(self, laserpower=80):
        '''turn on laser
        User can first preview image. After pressing escape,
        a final image is taken.
        '''
        # NOTE: all ND filters and a single channel is used
        self.host.laser_power = laserpower
        self.cam.set_exposure(1499)
        yield from self.host.enable_comp(laser1=True)
        print("Calibrate the camera with live view \
               and press escape to confirm spot in vision")
        self.cam.live_view(scale=0.6)
        img = self.takepicture()
        print(feature.spotsize(img))
        yield from self.host.enable_comp(laser1=False)

    def writepattern(self, pattern):
        '''repeats a pattern so a line is formed and writes to head

        pattern  --  list of bits [0] or [1,0,0]
        '''
        bits = self.host.laser_params['BITSINSCANLINE']
        line = (pattern*(bits//len(pattern))
                + pattern[:bits % len(pattern)])
        yield from self.host.writeline(line)

    @executor
    def searchcamera(self, timeout=3, build=False):
        '''laser is synced with photodiode and a line is projected

        This is done for various line patterns and is
        used to detect the edges of the camera
        '''
        self.host.laser_params['SINGLE_LINE'] = True
        self.host.laser_params['SINGLE_FACET'] = False
        if build:
            self.host.build()
        self.host.laser_power = 120
        self.cam.set_exposure(36000)
        yield from self.writepattern([0]*8+[0]*8)
        self.cam.live_view(scale=0.6)
        # TODO: it doesn't catch the stopline
        yield from self.host.writeline([])
        print(f'Wait for stopline to execute, {timeout} seconds')
        time.sleep(timeout)
        self.host.enable_comp(synchronize=False)

    def takepicture(self):
        'takes picture and store it with timestamp to this folder'
        img = self.cam.capture()
        grey_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        date_string = time.strftime("%Y-%m-%d-%H:%M")
        print(f"Writing to {Path(IMG_DIR, date_string+'.jpg')}")
        if not os.path.exists(IMG_DIR):
            os.makedirs(IMG_DIR)
        cv.imwrite(str(Path(IMG_DIR, date_string+'.jpg')), grey_img)
        return img


if __name__ == '__main__':
    unittest.main()
