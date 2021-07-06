import cv2 as cv
import numpy as np

# NOTE: you should use principal component analysis
#       for cross scan error etc


def find_largestcontour(img, denoise=False):
    '''finds the contours in the image

    img      --  numpy array to find contours in
    denoise  --  larger than 0 image gets denoised for erode
                 and dilate operation

    returns contour with largest area
    '''
    # RGB image to gray
    imgray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Converts gray scale image to binary using a threshold
    _, thresh = cv.threshold(imgray, img.max()//2, 255, cv.THRESH_BINARY)
    if denoise:
        # erode and dilate image
        kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (6, 6))
        thresh = cv.erode(thresh, kernel, iterations=1)
        thresh = cv.dilate(thresh, kernel, iterations=1)
    # find the external contour
    cntrs, _ = cv.findContours(
        thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if len(cntrs) == 0:
        raise Exception("Did not find a contour")
    c = max(cntrs, key=cv.contourArea)
    return c


def spotsize(img, pixelsize=3, debug=False, fname='detectsspotdebug.jpg'):
    '''detects spot

    The idea is that the laser is turned on and the motor is kept off.
    The prism is then pointed so that the laserbundle hits the camera.
    This method can be used to determine the size of the spot
    img: numpy array which should contain the spot
    pixelsize: size of one pixel in micrometers
    debug: if true fit is plotted and stored in fname
    fname: name of file were result of debug is stored
    returns dictionary with keys
        centroid  center of detected spot
        position  [x,y] of detected circle or ellipse
        axes      short axis diameter, long axis diameter
                in micrometers
    '''
    if img.min() < 0 or img.max() > 255:
        print("Image has wrong format, values outside [0-255]")
    c = find_largestcontour(img)
    el = cv.fitEllipse(c)
    if debug:
        cv.imwrite(fname, cv.ellipse(img, el, (0, 255, 0), thickness=2))
    # image center via centroid
    M = cv.moments(c)
    center = np.array((int(M['m10']/M['m00']), int(M['m01']/M['m00'])))
    dct = {
        'centroid': center*pixelsize,
        'position': list(np.array(el[0])*pixelsize),
        'axes': np.array(el[1])*pixelsize
    }
    return dct


def detect_line(img, debug=False, fname='detectlinedebug.jpg'):
    '''detect line and fit rotated rectangle

    The idea is that the motor and laser are turned on.
    A line is generated. This method fits a bounding rectangle and detects
    a line in this image. If debug is set to true the result is saved to disk.
    '''
    c = find_largestcontour(img)
    x, y, w, h = cv.boundingRect(c)
    # fit the rotated rectangle
    rect = cv.minAreaRect(c)
    box = cv.boxPoints(rect)
    box = np.int0(box)
    # fit line
    line = [vx, vy, x, y] = cv.fitLine(c, cv.DIST_L2,
                                       0, 0.01, 0.01)
    rows, cols = img.shape[:2]
    lefty = int((-x*vy/vx) + y)
    righty = int(((cols-x)*vy/vx)+y)
    if debug:
        cv.drawContours(img, [box], 0, (0, 0, 255), 2)
        img = cv.line(img, (cols-1, righty), (0, lefty), (0, 255, 0), 2)
        print("Saving detected contour to disk")
        cv.imwrite('detectedcontour.jpg', img)
    return line


def cross_scan_error(img, pixelsize=3, debug=False,
                     fname='crossscanndebug.jpg'):
    '''detect line and determine cross scan error
    The idea is that the motor and laser are turned on.
    A line is generated. This methods determines the skeleton of this line
    and the distance to edge of this line for this skeleton.
    This can than be used to quantify the cross scan error, via mean etc.
    img       -- numpy array which should contain the line of
                 which the width needs to be determined
    pixelsize -- size of one pixel in micrometers
    debug     -- true fit is plotted and stored in fname
    fname     -- name of file were result of debug is stored
    '''
    # image is denoised to ensure one skeleton is detected
    c = find_largestcontour(img, denoise=8)
    # RGB image to gray
    imgray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Converts gray scale image to binary using a threshold
    _, thresh = cv.threshold(imgray, img.max()//2, 255, cv.THRESH_BINARY)
    # extract contour
    bw = np.zeros(thresh.shape, dtype="uint8")
    extracted = cv.bitwise_and(thresh, (cv.drawContours(bw, [c], contourIdx=0,
                               color=1, thickness=cv.FILLED)))
    # skeletonize
    # NOTE:  opencv also has skeletonize operation called thinning in ximgproc
    from skimage import morphology
    # other approaches result in skeleton with short edges
    skel = morphology.skeletonize(extracted, method='lee')
    if debug:
        cv.imwrite(fname, skel*255)
    # compute the distance of each pixel
    distmap = cv.distanceTransform(extracted, cv.DIST_L2, 5)
    # only use pixels which are on the skeleton
    res = np.multiply(skel, distmap)
    # multiply as the skeleton should be in the center
    res = res[res.nonzero()]*pixelsize*2
    dct = {
        'max': np.max(res),
        'min': np.min(res),
        'mean': np.mean(res),
        'median': np.median(res)
    }
    return dct
