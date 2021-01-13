import cv2 as cv
import numpy as np

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

def detect_line(img, debug=False, fname='detectlinedebug.jpg'):
    '''detect line and fit rotated rectangle
        
    The idea is that the motor and laser are turned on.
    A line is generated. This method fits a bounding rectangle and detects
    a line in this image. If debug is set to true the result is saved to disk.
    '''
    # convert to gray
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # threshold image
    _, thresh = cv.threshold(gray, 127, 255, 0)
    # TODO: automate search for optimal erosion
    # erode the image, we want to remove the noise around the line
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE,(6,6))
    thresh = cv.erode(thresh,kernel,iterations = 1)
    # find all contours
    cntrs, hierarchy = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    if len(cntrs) != 0:
        # find the biggest countour (c) by the area
        c = max(cntrs, key = cv.contourArea)
        x,y,w,h = cv.boundingRect(c)
        # fit the rotated rectangle
        rect = cv.minAreaRect(c)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        if debug: cv.drawContours(img,[box],0,(0,0,255),2)
        # fit line
        line = [vx,vy,x,y] = cv.fitLine(c, cv.DIST_L2,0,0.01,0.01)
        rows,cols = img.shape[:2]
        lefty = int((-x*vy/vx) + y)
        righty = int(((cols-x)*vy/vx)+y)
        if debug: img = cv.line(img,(cols-1,righty),(0,lefty),(0,255,0),2)
        if debug:
            print("Saving detected contour to disk")
            cv.imwrite('detectedcontour.jpg', img)
        return line
    else:
        print('Did not found a contour')