import unittest
import math
from pathlib import Path
import time
import os

import cv2 as cv
import numpy as np

import camera
from hexastorm.controller import Machine
from hexastorm.core import Scanhead
import hexastorm.board as board
import hexastorm.optical as feature

TEST_DIR = Path(__file__).parents[0].resolve()
IMG_DIR = Path(TEST_DIR, 'images')
TESTIMG_DIR = Path(TEST_DIR, 'testimages')

class OptMachine(Machine):
    ''' Optical helper adds functions to do optical measurements'''
    def __init__(self, flash=True):
        super().__init__()
        self.single_facet = True
        if flash:
            self.flash(True, True)
        else:
            self.reset()

class OpticalTest(unittest.TestCase):
    '''Tests algorithms upon earlier taken images'''

    def test_laserline(self):
        '''tests laser line detection
        '''
        img = cv.imread(str(Path(TESTIMG_DIR, 'laserline1.jpg')))
        line = [vx,vy,x,y] = feature.detect_line(img)
        res = [0, 0, 1099, 719]
        for idx, val in enumerate(line): self.assertEqual(int(val), int(res[idx]))

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
        dct ={ 'laserspot1.jpg': np.array([26, 36]),
               'laserspot2.jpg': np.array([27, 41])
        }
        for k, v in dct.items():
            img = cv.imread(str(Path(TESTIMG_DIR, k)))
            np.testing.assert_array_equal(feature.spotsize(img)['axes'].round(0),
                                          v)

class Tests(unittest.TestCase):
    ''' Optical test for scanhead'''
    @classmethod
    def setUpClass(cls):
        cls.om = OptMachine(flash=False)
        cls.cam = camera.Cam()
        cls.cam.init()
    
    @classmethod
    def tearDownClass(cls):
        cls.cam.close()

    def alignlaser(self):
        '''align laser with prism
        
        Laser is aligned without camera
        '''
        self.om.test_laser()
        print("Press enter to confirm laser is aligned with prism")
        input()
        self.om.stop()

    def grabline(self):
        '''turn on laser and motor

        User can first preview image. After pressing escape,
        a final image is taken.
        '''
        self.om.test_line()
        self.om.laser_power = 120
        self.cam.set_exposure(36000)
        print("This will open up a window")
        print("Press escape to quitlive view")
        self.cam.live_view(0.6)
        img = self.takepicture()
        print(feature.cross_scan_error(img))
        self.om.stop()

    def grabspot(self, laserpower=80):
        '''turn on laser

        User can first preview image. After pressing escape,
        a final image is taken.
        '''
        self.om.laser_power = laserpower  #NOTE: at the moment all ND filters and a single channel is used
        self.cam.set_exposure(1499)
        self.om.test_laser()
        print("Calibrate the camera with live view an press escape to confirm spot in vision")
        self.cam.live_view(scale=0.6)
        img = self.takepicture()
        print(feature.spotsize(img))
        self.om.stop()

    def searchcamera(self):
        '''laser is synced with photodiode and a line is projected
        
        This is done for various line patterns and used to detect the edges of the camera
        '''
        self.om.single_line = True
        self.om.single_facet = True
        self.om.flash(recompile=True, removebuild=True)
        self.om.writeline([1]*self.om.sh.BITSINSCANLINE)
        self.om.start()
        self.cam.live_view(scale=0.6)
        self.om.stop()

    def takepicture(self):
        'takes picture and store it with timestamp to this folder'
        img = self.cam.capture()
        grey_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        date_string = time.strftime("%Y-%m-%d-%H:%M")
        print(f"Writing to {Path(IMG_DIR, date_string+'.jpg')}")
        if not os.path.exists(IMG_DIR): os.makedirs(IMG_DIR)
        cv.imwrite(str(Path(IMG_DIR, date_string+'.jpg')), grey_img)
        return img

if __name__ == '__main__':
    unittest.main()