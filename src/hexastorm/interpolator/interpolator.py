import math
import os
from numba import jit, typed, types

import numpy as np
from scipy import ndimage
from PIL import Image
from hexastorm.platforms import Firestarter
from hexastorm.lasers import params as paramsfunc


# numba.jit decorated functions cannot be defined with self
@jit
def displacement(pixel, params):
    '''returns the displacement for a given pixel

    The x-axis is parallel to the scanline if the stage does not move.
    It is assumed, the laser bundle traverses
    in the negative direction if the polygon rotates.

    pixel  --  the pixelnumber, in range [0, self.pixelsfacet]
    '''
    # interiorangle = 180-360/self.n
    # max_angle = 180-90-0.5*interiorangle
    max_angle = 180/params['FACETS']
    pixelsfacet = round(params['LASER_HZ'] /
                        (params['rotationfrequency']*params['FACETS']))
    angle = np.radians(- 2 * max_angle * (pixel/pixelsfacet) + max_angle)
    disp = (params['inradius']*2*np.sin(angle) *
            (1-np.power((1-np.power(np.sin(angle), 2)) /
             (np.power(params['n'], 2)-np.power(np.sin(angle), 2)), 0.5)))
    return disp


@jit
def fxpos(pixel, params, xstart=0):
    '''returns the laserdiode x-position in pixels

    The x-axis is parallel to the scanline if the stage does not move.
    pixel   -- the pixelnumber in the line
    xstart  -- the x-start position [mm], typically
                your xstart is larger than 0 as the displacement
                can be negative
    '''
    line_pixel = params['startpixel'] + pixel % params['BITSINSCANLINE']
    xpos = (np.sin(params['tiltangle']) *
            displacement(line_pixel, params)
            + xstart)
    return xpos/params['samplegridsize']


@jit
def fypos(pixel, params, direction, ystart=0):
    '''
    returns the laserdiode y-position in pixels

    The y-axis is orthogonal to the scanline if the stage does not move,
    and parallel to the stage movement.
    pixel      -- the pixelnumber in the line
    direction  -- True is +, False is -
    ystart     -- the y-start position [mm]
    '''
    line_pixel = params['startpixel'] + pixel % params['BITSINSCANLINE']
    if direction:
        ypos = -np.cos(params['tiltangle'])*displacement(line_pixel,
                                                         params)
        ypos += (line_pixel/params['LASER_HZ']*params['stagespeed']
                 + ystart)
    else:
        ypos = -np.cos(params['tiltangle'])*displacement(line_pixel,
                                                         params)
        ypos -= (line_pixel/params['LASER_HZ']*params['stagespeed'] +
                 ystart)
    return ypos/params['samplegridsize']


class Interpolator:
    '''
    Object to determine binary laser diode information
    for a prism scanner.

    A post script file is converted to a numpy array.
    Laserdiode positions are calculated via the function
    createcoordinates.
    The function patternfiles interpolates the image using the positions.
    These values are either 0 (laser off) or 1 (laser on).
    The files created can also be read and plotted.
    The objects supports different tilt angles which are required by
    the original light engine described in US10114289B2.
    '''
    def __init__(self):
        self.params = self.parameters()
        self.params = self.downsample(self.params)
        currentdir = os.path.dirname(os.path.realpath(__file__))
        self.debug_folder = os.path.join(currentdir, 'debug')

    def parameters(self, stepsperline=1):
        '''
        sets parameters for slicer based on parameters from board

        In principle, the board support both reflective and refractive 
        polygon scanning. The cut is not very clean but in principle
        the prism nature of the system is seperated and only present here.

        stepsperline -- this parameter is not fixed and can be changed
                        without flashing the board
        '''
        # in new version of numba dictionaries are converted automatically
        dct = typed.Dict.empty(key_type=types.string,
                               value_type=types.float64)
        platform = Firestarter()
        var = paramsfunc(platform)
        dct2 = {
            # angle [radians], for a definition see figure 7
            # https://reprap.org/wiki/Transparent_Polygon_Scanning
            'tiltangle': np.radians(90),
            'LASER_HZ': var['LASER_HZ'],   # Hz
            # rotation frequency polygon [Hz]
            'rotationfrequency': var['RPM']/60,
            # number of facets
            'FACETS': var['FACETS'],
            # inradius polygon [mm]
            'inradius': 15,
            # refractive index
            'n': 1.49,
            # platform size scanning direction [mm]
            'pltfxsize': 200,
            # platform size stacking direction [mm]
            'pltfysize': 200,
            # sample size scanning direction [mm]
            'samplexsize': 0,
            # sample size stacking direction [mm]
            'sampleysize': 0,
            # NOTE: sample below Nyquist criterion
            #       you first sample the image,
            #       you then again sample the sampled image
            # height/width of the sample gridth [mm]
            'samplegridsize': 0.015,
            # mm/s
            'stagespeed': ((stepsperline/platform.stepspermm[platform.laser_axis])
                           *(var['RPM']/60)*var['FACETS']),
            # pixel determined via camera
            'startpixel': ((var['BITSINSCANLINE']/(var['END%']-var['START%']))
                           * var['START%']),
            # number of pixels in a line [new 785]
            'BITSINSCANLINE': var['BITSINSCANLINE'],
            # each can be repeated, to speed up interpolation
            # this was used on the beaglebone
            'downsamplefactor': 1
        }
        for k, v in dct2.items():
            dct[k] = v
        return dct

    def downsample(self, params):
        '''to speed up interpolation pixels can be repeated

        This was used on the beaglebone as it was harder to change
        the frequency
        '''
        if params['downsamplefactor'] > 1:
            lst = ['LASER_HZ', 'startpixel', 'BITSINSCANLINE']
            for item in lst:
                params[item] /= params['downsamplefactor']
                params[item] = round(params[item])
        return params

    def pstoarray(self, url):
        '''converts postscript file to an array

        url  --  path to postcript file
        '''
        params = self.params
        # post script pixel in mm
        psppoint = 0.3527777778
        tmp = Image.open(url)
        x_size, y_size = [i*psppoint for i in tmp.size]
        if x_size > params['pltfxsize'] or y_size > params['pltfysize']:
            raise Exception('Object does not fit on platform')
        # NOTE: this is only a crude approximation
        params['samplexsize'] = x_size
        params['sampleysize'] = y_size
        self.params = params
        scale = psppoint/params['samplegridsize']
        tmp.load(scale=scale)
        tmp_array = np.array(tmp.convert('1'))
        if tmp_array.max() == 0:
            raise Exception("Postscript file is empty")
        return tmp_array

    def createcoordinates(self):
        '''returns the x, y position of the laserdiode
        for each pixel, for all lanes

        assumes the line starts at the positive plane
        '''
        params = self.params
        if not params['sampleysize'] or not params['samplexsize']:
            raise Exception('Sampleysize or samplexsize are set to zero.')
        if (fxpos(0, params) < 0 or
                fxpos(params['BITSINSCANLINE']-1, params) > 0):
            raise Exception('Line seems ill positioned')
        # mm
        lanewidth = ((fxpos(0, params) -
                     fxpos(params['BITSINSCANLINE']-1, params))
                     * params['samplegridsize'])
        lanes = math.ceil(params['samplexsize']/lanewidth)
        facets_inlane = math.ceil(params['rotationfrequency']
                                  * params['FACETS'] *
                                  (params['sampleysize']/params['stagespeed']))
        print("The lanewidth is {}".format(lanewidth))
        print("The facets in lane are {}".format(facets_inlane))
        # single facet

        def fxpos2(x, y=0):
            return fxpos(x, params, y)

        def fypos2(x, y=0):
            return fypos(x, params, y)
        vfxpos = np.vectorize(fxpos2, otypes=[np.int16])
        vfypos = np.vectorize(fypos2, otypes=[np.int16])
        xstart = abs(fxpos2(int(params['BITSINSCANLINE'])-1)
                     * params['samplegridsize'])
        xpos_facet = vfxpos(range(0, int(params['BITSINSCANLINE'])), xstart)
        # TODO: you still don't account for ystart
        # (you are moving in the y, so if you start
        #  at the edge you miss something)
        ypos_forwardfacet = vfypos(range(0, int(params['BITSINSCANLINE'])),
                                   True)
        ypos_backwardfacet = vfypos(range(0, int(params['BITSINSCANLINE'])),
                                    False)
        # single lane
        xpos_lane = np.tile(xpos_facet, facets_inlane)
        # TODO: parallel not supported on 32 bit hardware

        @jit(nopython=True, parallel=False)
        def loop0(params):
            forward = np.zeros((facets_inlane, int(params['BITSINSCANLINE'])))
            backward = np.zeros((facets_inlane, int(params['BITSINSCANLINE'])))
            for facet in range(0, facets_inlane):
                ypos_forwardtemp = ypos_forwardfacet + round(
                    (facet*params['stagespeed']) /
                    (params['FACETS']*params['rotationfrequency'] *
                     params['samplegridsize']))
                ypos_backwardtemp = ypos_backwardfacet + round(
                    ((facets_inlane-facet)*params['stagespeed']) /
                    (params['FACETS']*params['rotationfrequency']
                     * params['samplegridsize']))
                forward[facet] = ypos_forwardtemp
                backward[facet] = ypos_backwardtemp
            return forward.flatten(), backward.flatten()

        ypos_forwardlane, ypos_backwardlane = loop0(params)
        # per lane; 5000 (pixels per facet) / 5 (samplefactor)
        #    * (200 mm/0.015 mm (resolution)*2 (16 bit = 2byte)) )/1E6 = 40 MB

        # all lanes, another option would be to slice per lane
        @jit(nopython=True, parallel=False)
        def loop1(params):
            xpos = np.zeros((lanes, len(xpos_lane)), dtype=np.int16)
            ypos = np.zeros((lanes, len(ypos_forwardlane)), dtype=np.int16)
            xwidthlane = (fxpos(0, params) -
                          fxpos(params['BITSINSCANLINE']-1, params))
            for lane in range(0, lanes):
                # TODO: why is this force needed?
                xoffset = int(round(lane * xwidthlane))
                xpos_temp = xpos_lane + xoffset
                if lane % 2 == 1:
                    ypos_temp = ypos_backwardlane
                else:
                    ypos_temp = ypos_forwardlane
                xpos[lane] = xpos_temp
                ypos[lane] = ypos_temp
            return xpos.flatten(), ypos.flatten()

        xpos, ypos = loop1(params)
        # interpolation can be linear, however int are used to save space
        ids = np.concatenate(([xpos], [ypos]))
        return ids

    def patternfile(self, url, test=False):
        '''returns the pattern file as numpy array

        mostly a convenience function which wraps other functions in this class
        url   -- path to postscript file
        test  -- runs a sampling test, whether laser
                 frequency sufficient to provide accurate sample
        '''
        from time import time
        ctime = time()
        layerarr = self.pstoarray(url).astype(np.uint8)
        if test:
            img = Image.fromarray(layerarr.astype(np.uint8)*255)
            img.save(os.path.join(self.debug_folder, 'nyquistcheck.png'))
        if test:
            layerarr = np.ones_like(layerarr)
        print("Retrieved layerarr")
        print(f"Elapsed {time()-ctime:.2f}")
        ids = self.createcoordinates()
        print("Retrieved coordinates")
        print(f"Elapsed {time()-ctime:.2f}")
        if test:
            ptrn = ndimage.map_coordinates(input=layerarr, output=np.uint8,
                                           coordinates=ids, order=1,
                                           mode="constant", cval=1)
        else:
            ptrn = ndimage.map_coordinates(input=layerarr, output=np.uint8,
                                           coordinates=ids, order=1,
                                           mode="constant", cval=0)
        print("Completed interpolation")
        print(f"Elapsed {time()-ctime:.2f}")
        if ptrn.min() < 0 or ptrn.max() > 1:
            raise Exception('This is not a bit list.')
        ptrn = np.logical_not(ptrn)
        ptrn = np.repeat(ptrn, self.params['downsamplefactor'])
        ptrn = np.packbits(ptrn)
        return ptrn

    def plotptrn(self, ptrn, step, filename='plot'):
        '''function can be used to plot a pattern file.
        Result is returned as numpy array and stored
        in script folder under filename.

        ptrnfile --  result of the functions patternfiles
        step     --  pixel step, can be used to lower the number
                     of pixels that are plotted
        filename --  filename to store pattern
        '''
        # TODO: - plot with real laser spots --> convolution?
        #       - your y-step is greater than sample size so you see lines
        #        this will not be there in
        #        reality as the laser spot is larger
        ids = self.createcoordinates()
        ids = np.repeat(ids, self.params['downsamplefactor'], axis=1)
        # repeat adden
        xcor = ids[0, ::step]
        ycor = ids[1, ::step]
        if xcor.min() < 0:
            print('XCOR negative, weird!')
            xcor += abs(xcor.min())
        if ycor.min() < 0:
            print('YCOR negative, weird!')
            ycor += abs(ycor.min())
        arr = np.zeros((xcor.max() + 1, ycor.max() + 1), dtype=np.uint8)
        ptrn = np.unpackbits(ptrn)
        # TODO: this is strange, added as quick fix on may 9 2021
        ptrn = ptrn[:len(xcor)]
        arr[xcor[:], ycor[:]] = ptrn[0: len(ptrn): step]
        arr = arr * 255
        img = Image.fromarray(arr)
        img.save(os.path.join(self.debug_folder, filename + '.png'))
        return img

    def readbin(self, name='test.bin'):
        '''reads a binary file

        name  -- name of binary file with laser information
        '''
        pat = np.fromfile(os.path.join(self.debug_folder, name),
                          dtype=np.uint8)
        return pat

    def writebin(self, pixeldata, filename='test.bin'):
        '''writes pixeldata to a binary file

        pixeldata  -- must have uneven length
        filename   -- name of binary file to write laserinformation to
        '''
        pixeldata = pixeldata.astype(np.uint8)
        if not os.path.exists(self.debug_folder):
            os.makedirs(self.debug_folder)
        pixeldata.tofile(os.path.join(self.debug_folder, filename))


if __name__ == "__main__":
    # https://github.com/hstarmans/ldgraphy
    # numba greatly improves performance
    # install procedure followed is
    # https://github.com/numba/llvmlite/issues/604
    # parallelization could not be used, as I am
    # still on 32 bit (this is easier with
    # camera)
    interpolator = Interpolator()
    dir_path = os.path.dirname(os.path.realpath(__file__))
    url = os.path.join(dir_path, 'test-patterns', 'line-resolution-test.ps')
    ptrn = interpolator.patternfile(url)
    interpolator.writebin(ptrn, "test.bin")
    interpolator.pstoarray(url)
    pat = interpolator.readbin("test.bin")
    print(f"The shape of the pattern is {pat.shape}")
    interpolator.plotptrn(ptrn=pat, step=1)
