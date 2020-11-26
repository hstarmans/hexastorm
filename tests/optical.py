import unittest
import math
from pathlib import Path
import time

import cv2
import numpy as np

from hexastorm.camera import Cam
from hexastorm.controller import Machine
from hexastorm.core import Scanhead
import hexastorm.board as board

TEST_DIR = Path(__file__).parents[0].resolve()
IMG_DIR = Path(TEST_DIR, 'images')

def spotsize(img, pixelsize = 4.65, ellipse = -1):
    ''' detects spot
    
    returns dictionary with keys
    centroid  center of detected spot
    position  [x,y] of detected circle or ellipse
    axes      short axis diameter, long axis diameter
              in micrometers

    img: numpy array which should contain the spot
    pixelsize: the size of one pixel in micrometers
    ellipse: 1 calculate position and axes of ellipse
             0 calculate position and radius of circle
            -1 don't calculate position and axes of the spot
    '''
    if img.max() == 255: print("Camera satured, spotsize can be incorrect")
    if img.min()<0 or img.max()>255: print("Image has wrong format, values outside [0-255]")
    # Converts image from one colour space to another.
    # RGB image to gray
    imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Converts gray scale image to binary using a threshold
    ret, thresh = cv2.threshold(imgray, img.max()//2, 255, cv2.THRESH_BINARY)
    # find the external contour
    im2, contours, hierarchy = cv2.findContours\
                    (thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) == 0 or len(contours)>2:
        def createkey(contour):
            '''key function for sort
            
            allowed sorting if multiple ellipses were detected
            using contours.sort(createkey).reverse() 
            
            This is not supported at the moment, make sure image is correct
            '''
            momA = cv2.moments(contour)        
            (xa,ya) = int(momA['m10']/momA['m00']), int(momA['m01']/momA['m00'])
            return xa          
        print("Detected none or multiple spots")
    try:
        if ellipse == 1:
            el = cv2.fitEllipse(contours[0])
        elif ellipse == 0:
            el = cv2.minEnclosingCircle(contours[0])
        else:
            el = [[0,0],[0,0]]
    except:
        print("Spot not detected")
    # image center via centroid
    M = cv2.moments(contours[0]) 
    center = np.array((int(M['m10']/M['m00']), int(M['m01']/M['m00'])))
    dct = {
        'centroid'  : center*pixelsize,
        'position' : list(np.array(el[0])*pixelsize),
        'axes' : np.array(el[1])*pixelsize
    }
    return dct

class OptMachine(Machine):
    ''' Optical helper adds functions to do optical measurements'''
    def __init__(self, flash=True):
        super().__init__()
        self.single_facet = True
        if flash:
            self.flash(True, True)
        else:
            self.reset()
        self.cam = Cam()

class Tests(unittest.TestCase):
    ''' Virtual test for scanhead'''
    def setUp(self):
        self.om = OptMachine(flash=False)

    def takepicture(self):
        'takes picture and store it with timestamp to this folder'
        img = self.om.cam.take_picture()
        grey_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        date_string = time.strftime("%Y-%m-%d-%H:%M")
        print(f"Writing to {Path(IMG_DIR, date_string+'.jpg')}")
        cv2.imwrite(str(Path(IMG_DIR, date_string+'.jpg')), grey_img) 
    
    def aligncamera(self):
        'manually align camera'
        self.om.test_line()
        print("Press enter to confirm camera is aligned")
        input()
        self.takepicture()
        self.om.stop()

if __name__ == '__main__':
    unittest.main()