import cv2 as cv
import numpy as np

def spotsize(img, pixelsize = 3, debug=False, fname='detectsspotdebug.jpg'):
    ''' detects spot
    
    img: numpy array which should contain the spot
    pixelsize: the size of one pixel in micrometers

    returns dictionary with keys
    centroid  center of detected spot
    position  [x,y] of detected circle or ellipse
    axes      short axis diameter, long axis diameter
              in micrometers
    '''
    if img.max() == 255: print("Camera satured, spotsize can be incorrect")
    if img.min()<0 or img.max()>255: print("Image has wrong format, values outside [0-255]")
    # RGB image to gray
    imgray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Converts gray scale image to binary using a threshold
    _, thresh = cv.threshold(imgray, img.max()//2, 255, cv.THRESH_BINARY)
    # find the external contour
    cntrs, hierarchy = cv.findContours\
                    (thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if len(cntrs) == 0:
        print("Did not detect a spot")
        return
    elif len(cntrs)>2:
        def createkey(contour):
            '''key function for sort
            
            allowed sorting if multiple ellipses were detected
            using contours.sort(createkey).reverse() 
            
            This is not supported at the moment, make sure image is correct
            '''
            momA = cv.moments(contour)        
            (xa,ya) = int(momA['m10']/momA['m00']), int(momA['m01']/momA['m00'])
            return xa          
        print("Detected none or multiple spots, using spot with largest surface area")
        c = max(cntrs, key = cv.contourArea)
    else:
        c = cntrs[0]
    el = cv.fitEllipse(c)
    if debug: cv.imwrite(fname, cv.ellipse(img, el, (0,255,0), thickness=2))
    # image center via centroid
    M = cv.moments(c) 
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