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
        line= [vx,vy,x,y] = feature.detect_line(img)
        self.assertListEqual(list(line), [0.09047453,-0.9958988,1106.4971,629.26263])

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

    def aligncamera(self):
        '''align laserline with camera
        '''
        self.om.test_line()
        print("This will open up a window")
        print("Press esc escape live view")
        self.cam.live_view(3000)
        self.takepicture()
        self.om.stop()

    def takepicture(self, capturetime=3000):
        'takes picture and store it with timestamp to this folder'
        img = self.cam.capture(capturetime)
        # TODO: this should not be needed
        # crop image as Arducamdriver has some weird internals
        img = img[:-20,:]
        grey_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        date_string = time.strftime("%Y-%m-%d-%H:%M")
        print(f"Writing to {Path(IMG_DIR, date_string+'.jpg')}")
        if not os.path.exists(IMG_DIR): os.makedirs(IMG_DIR)
        cv.imwrite(str(Path(IMG_DIR, date_string+'.jpg')), grey_img)

if __name__ == '__main__':
    unittest.main()