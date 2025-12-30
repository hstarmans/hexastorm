import math
import os
from io import BytesIO
import struct
import zlib

import numpy as np
from cairosvg import svg2png
from numba import jit, typed, types
from PIL import Image

from hexastorm.config import PlatformConfig, Spi
from hexastorm.fpga_host.interface import BaseHost


# numba.jit decorated functions cannot be defined with self
@jit(nopython=True)
def displacement(pixel, params):
    """returns the displacement for a given pixel

    The x-axis is parallel to the scanline if the stage does not move.
    It is assumed, the laser bundle traverses
    in the negative direction and the prism motor
    rotates counterclockwise.

    pixel  --  the pixelnumber, in range [0, self.pixelsfacet]
    """
    # interiorangle = 180-360/self.n
    # max_angle = 180-90-0.5*interiorangle
    max_angle = 180 / params["FACETS"]
    pixelsfacet = round(
        params["LASER_HZ"] / (params["rotationfrequency"] * params["FACETS"])
    )
    angle = np.radians(-2 * max_angle * (pixel / pixelsfacet) + max_angle)
    disp = (
        params["inradius"]
        * 2
        * np.sin(angle)
        * (
            1
            - np.power(
                (1 - np.power(np.sin(angle), 2))
                / (np.power(params["n"], 2) - np.power(np.sin(angle), 2)),
                0.5,
            )
        )
    )
    return disp


@jit(nopython=True)
def fxpos(pixel, params, xstart=0):
    """returns the laserdiode x-position in pixels

    The x-axis is parallel to the scanline if the stage does not move.
    pixel   -- the pixelnumber in the line
    xstart  -- the x-start position [mm], typically
                your xstart is larger than 0 as the displacement
                can be negative
    """
    line_pixel = params["startpixel"] + pixel % params["bitsinscanline"]
    xpos = np.sin(params["tiltangle"]) * displacement(line_pixel, params) + xstart
    return xpos / params["samplegridsize"]


@jit(nopython=True)
def fypos(pixel, params, direction, ystart=0):
    """
    returns the laserdiode y-position in pixels

    The y-axis is orthogonal to the scanline if the stage does not move,
    and parallel to the stage movement.
    pixel      -- the pixelnumber in the line
    direction  -- True is +, False is -
    ystart     -- the y-start position [mm]
    """
    line_pixel = params["startpixel"] + pixel % params["bitsinscanline"]
    if direction:
        ypos = -np.cos(params["tiltangle"]) * displacement(line_pixel, params)
        ypos += line_pixel / params["LASER_HZ"] * params["stagespeed"] + ystart
    else:
        ypos = -np.cos(params["tiltangle"]) * displacement(line_pixel, params)
        ypos -= line_pixel / params["LASER_HZ"] * params["stagespeed"] + ystart
    return ypos / params["samplegridsize"]


class Interpolator:
    """
    Object to determine binary laser diode information
    for a prism scanner.

    A post script file is converted to a numpy array.
    Laserdiode positions are calculated via the function
    createcoordinates.
    The function patternfiles interpolates the image using the positions.
    These values are either 0 (laser off) or 1 (laser on).
    The files created can also be read and plotted.
    The objects supports different tilt angles which are required by
    the alternative light engine described in US10114289B2.
    """

    def __init__(self, stepsperline=1):
        self.cfg = PlatformConfig(test=False)
        self.params = self.parameters(stepsperline)
        self.params = self.downsample(self.params)
        currentdir = os.path.dirname(os.path.realpath(__file__))
        self.debug_folder = os.path.join(currentdir, "debug")

    def parameters(self, stepsperline=1):
        """
        sets parameters for slicer based on parameters from board

        In principle, the board support both reflective and refractive
        polygon scanning. The cut is not very clean but in principle
        the prism nature of the system is seperated and only present here.

        stepsperline -- parameter best kept fixed at 1
                        line can be exposed multiple times per line
                        during exposure
        """
        laz_tim = self.cfg.laser_timing
        mtr_cfg = self.cfg.motor_cfg
        # TODO: slicer should account for direction
        # bytes are read using little but finally direction is flipped!
        # as such packing is here in big
        self.bitorder = "big"
        # NOTE:  very special dictionary!!
        # in new version of numba dictionaries are converted automatically
        dct = typed.Dict.empty(key_type=types.string, value_type=types.float64)
        dct2 = {
            # angle [radians], for a definition see figure 7
            # https://reprap.org/wiki/Open_hardware_fast_high_resolution_LASER
            "tiltangle": np.radians(90),
            "LASER_HZ": laz_tim["laser_hz"],  # Hz
            # rotation frequency polygon [Hz]
            "rotationfrequency": laz_tim["rpm"] / 60,
            # number of facets
            "FACETS": laz_tim["facets"],
            # inradius polygon [mm]
            "inradius": 15,
            # refractive index
            "n": 1.49,
            # platform size scanning direction [mm]
            "pltfxsize": 200,
            # platform size stacking direction [mm]
            "pltfysize": 200,
            # sample size scanning direction [mm]
            "samplexsize": 0,
            # sample size stacking direction [mm]
            "sampleysize": 0,
            # NOTE: sample below Nyquist criterion
            #       you first sample the image,
            #       you then again sample the sampled image
            # height/width of the sample gridth [mm]
            "samplegridsize": 0.005,
            "stepsperline": stepsperline,
            "facetsinlane": 0,  # set when file is parsed
            # mm/s
            "stagespeed": (
                (stepsperline / mtr_cfg["steps_mm"][mtr_cfg["orth2lsrline"]])
                * (laz_tim["rpm"] / 60)
                * laz_tim["facets"]
            ),
            # first calculates all bits in scanline and then the start
            "startpixel": (
                (
                    laz_tim["scanline_length"]
                    / (laz_tim["end_frac"] - laz_tim["start_frac"])
                )
                * laz_tim["start_frac"]
            ),
            # number of pixels in a line [new 785]
            "bitsinscanline": laz_tim["scanline_length"],
            # each can be repeated, to speed up interpolation
            # this was used on the beaglebone
            "downsamplefactor": 1,
        }
        dct.update(dct2)
        dct["lanewidth"] = (
            fxpos(0, dct) - fxpos(dct["bitsinscanline"] - 1, dct)
        ) * dct["samplegridsize"]
        return dct

    def downsample(self, params):
        """to speed up interpolation pixels can be repeated

        This was used on the beaglebone as it was harder to change
        the frequency
        """
        if params["downsamplefactor"] > 1:
            lst = ["LASER_HZ", "startpixel", "bitsinscanline"]
            for item in lst:
                params[item] /= params["downsamplefactor"]
                params[item] = round(params[item])
        return params

    def svgtopil(self, svg_filepath):
        """converts SVG snippets to a PIL Image"""
        with open(svg_filepath, "rb") as f:
            svg_data = f.read()
        dpi = 25.4 / self.params["samplegridsize"]
        png_data = svg2png(bytestring=svg_data, dpi=dpi)
        # rotation used to align jitter and crosstest correctly to laser
        img = Image.open(BytesIO(png_data)).rotate(-90, expand=True)
        return img

    def pstopil(self, url, pixelsize=0.3527777778):
        """converts postscript file to a PIL Image

        url       --  path to postcript file
        pixelsize -- pixel size in mm
        """
        img = Image.open(url)
        scale = pixelsize / self.params["samplegridsize"]
        img.load(scale=scale)
        return img

    def imgtopil(self, url, pixelsize):
        """converts image to a PIL image

        url       --  path to PIL image, e.g. PNG or BMP
        """
        img = Image.open(url)
        scale = pixelsize / self.params["samplegridsize"]
        img = img.resize([round(x * scale) for x in img.size])
        return img

    def piltoarray(self, pil):
        """converts PIL Image to numpy array

        Method also changes the settings of on the objects
        and clips to the area of interest

        pil
        """
        img_array = np.array(pil.convert("1"))
        if img_array.max() == 0:
            raise Exception("Image is empty")
        # clip image
        nonzero_col = np.argwhere(img_array.sum(axis=0)).squeeze()
        nonzero_row = np.argwhere(img_array.sum(axis=1)).squeeze()
        img_array = img_array[
            nonzero_row[0] : nonzero_row[-1], nonzero_col[0] : nonzero_col[-1]
        ]
        # update settings
        x_size, y_size = [i * self.params["samplegridsize"] for i in img_array.shape]
        if (x_size > self.params["pltfxsize"]) or (y_size > self.params["pltfysize"]):
            raise Exception("Object does not fit on platform")
        print(f"Sample size is {x_size:.2f} mm by {y_size:.2f} mm")
        # NOTE: this is only a crude approximation
        self.params["samplexsize"] = x_size
        self.params["sampleysize"] = y_size
        return img_array

    def lanewidth(self):
        params = self.params
        lanewidth = (
            fxpos(0, params) - fxpos(params["bitsinscanline"] - 1, params)
        ) * params["samplegridsize"]
        return lanewidth

    def createcoordinates(self):
        """
        returns the x, y position of the laserdiode
        for each pixel, for all lanes.
        """
        params = self.params
        if not params["sampleysize"] or not params["samplexsize"]:
            raise Exception("Sampleysize or samplexsize are set to zero.")

        # Check if line positioning is valid
        if fxpos(0, params) < 0 or fxpos(params["bitsinscanline"] - 1, params) > 0:
            raise Exception("Line seems ill positioned")

        # 1. Basic Geometry Calculation
        lanewidth = self.lanewidth()
        lanes = math.ceil(params["samplexsize"] / lanewidth)
        facets_inlane = math.ceil(
            params["rotationfrequency"]
            * params["FACETS"]
            * (params["sampleysize"] / params["stagespeed"])
        )

        self.params["facetsinlane"] = facets_inlane
        self.params["lanewidth"] = lanewidth

        print(f"The lanewidth is {lanewidth:.2f} mm")
        print(f"The facets in lane are {facets_inlane}")

        # 2. Base Facet Generation (Single scanline)
        # We pass the entire range of pixels to the JIT functions at once
        pixel_indices = np.arange(int(params["bitsinscanline"]))

        # Calculate xstart (mm to pixel conversion)
        xstart_val = abs(
            fxpos(int(params["bitsinscanline"]) - 1, params) * params["samplegridsize"]
        )

        # These calls work on arrays because the Numba functions are simple math
        xpos_facet = fxpos(pixel_indices, params, xstart_val).astype(np.int16)
        ypos_fwd_facet = fypos(pixel_indices, params, True)  # forward
        ypos_bwd_facet = fypos(pixel_indices, params, False)

        # Facet Offsets
        # Create an array of offsets for every facet in a lane
        facet_idx = np.arange(facets_inlane).reshape(-1, 1)  # Shape (facets, 1)

        # Shift factor: movement of stage per facet
        y_shift_per_facet = (params["stagespeed"]) / (
            params["FACETS"] * params["rotationfrequency"] * params["samplegridsize"]
        )

        # Broadcast (facets, 1) + (1, pixels) -> (facets, pixels)
        y_fwd_lane = (ypos_fwd_facet + (facet_idx * y_shift_per_facet)).flatten()
        y_bwd_lane = (
            ypos_bwd_facet + ((facets_inlane - facet_idx) * y_shift_per_facet)
        ).flatten()
        x_lane_base = np.tile(xpos_facet, facets_inlane)

        # 4. Lane Offsets (Broadcasting loop1)
        lane_idx = np.arange(lanes).reshape(-1, 1)  # Shape (lanes, 1)
        x_width_pixel = fxpos(0, params) - fxpos(params["bitsinscanline"] - 1, params)

        x_offsets = (lane_idx * x_width_pixel).astype(np.int16)

        # Generate X: Add lane offsets to the base lane x-positions
        x_final = (x_lane_base + x_offsets).flatten()

        # Generate Y: Alternate between forward and backward lane data based on parity
        # We create a boolean mask for even/odd lanes
        is_forward_lane = (np.arange(lanes) % 2 == 0).reshape(-1, 1)

        # Select the appropriate Y data per lane
        y_final = np.where(is_forward_lane, y_fwd_lane, y_bwd_lane).flatten()

        # We must cast to integers so they can be used as array indices/shapes
        # Using np.int32 or np.intp (platform-dependent integer)
        ids = np.vstack((x_final, y_final)).astype(np.int32)

        return ids

    def patternfile(
        self, url, pixelsize=0.3527777778, test=False, positiveresist=False
    ):
        """returns the pattern file as numpy array

        Converts image at URL to pattern for laser scanner

        url   -- path to file
        positiveresist -- False if negative resist
        test  -- runs a sampling test, whether laser
                 frequency sufficient to provide accurate sample
        """
        from time import time

        ctime = time()
        _, extension = os.path.splitext(url)
        if extension == ".svg":
            pil = self.svgtopil(url)
            pil.save("debug.png")
        elif extension == ".ps":
            pil = self.pstopil(url)
        else:
            pil = self.imgtopil(url, pixelsize)
        layerarr = self.piltoarray(pil).astype(np.uint8)
        if test:
            img = Image.fromarray(layerarr.astype(np.uint8) * 255)
            img.save(os.path.join(self.debug_folder, "nyquistcheck.png"))
        if test:
            layerarr = np.ones_like(layerarr)
        print("Retrieved image")
        print(f"Elapsed {time() - ctime:.2f} seconds")
        ids = self.createcoordinates()
        print("Created coordinates for interpolation")
        print(f"Elapsed {time() - ctime:.2f} seconds")
        ids = self.createcoordinates()
        x_coords = ids[1]
        y_coords = ids[0]

        # 1. Bounds Checking
        # Create a mask for points that actually land on the image
        mask = (
            (x_coords >= 0)
            & (x_coords < layerarr.shape[1])
            & (y_coords >= 0)
            & (y_coords < layerarr.shape[0])
        )

        # 2. Fast Sampling (Advanced Indexing)
        # Initialize with 0 (laser off)
        ptrn = np.zeros(x_coords.shape, dtype=np.uint8)

        # Map coordinates: Note the (y, x) order for NumPy indexing!
        # layerarr[row, col] -> layerarr[y, x]
        ptrn[mask] = layerarr[y_coords[mask], x_coords[mask]]
        print("Completed interpolation")
        print(f"Elapsed {time() - ctime:.2f} seconds")
        if ptrn.min() < 0 or ptrn.max() > 1:
            raise Exception("This is not a bit list")
        if not positiveresist:
            ptrn = np.logical_not(ptrn)
        ptrn = np.repeat(ptrn, self.params["downsamplefactor"])
        ptrn = np.packbits(ptrn, bitorder=self.bitorder)
        return ptrn

    def plotptrn(self, ptrn_df, step, filename="plot"):
        """function can be used to plot a dataframe with laser data

        The settings are retrieved from the dataframe
        and the plotted as an image.
        The result is returned as numpy array and stored
        in script folder under filename.
        The origin is in the lower-left corner,
        the starting point of the exposure.
        The laser line is parallel to the x axis.

        ptrn_df  --  dataframe with laserdata
        step     --  pixel step, can be used to lower the number
                     of pixels that are plotted
        filename --  filename to store pattern
        """
        # TODO: - plot with real laser spots --> convolution?
        #       - your y-step is greater than sample size so you see lines
        #        this will not be there in
        #        reality as the laser spot is larger
        ids = self.createcoordinates()

        # TODO: either use parquet or numpy
        # in case of parquet I store additional information
        try:
            for k in self.params.keys():
                self.params[k] = ptrn_df[k][0]
        except IndexError:
            pass

        ids = np.repeat(ids, self.params["downsamplefactor"], axis=1)
        # repeat adden
        xcor = ids[0, ::step]
        ycor = ids[1, ::step]
        if xcor.min() < 0:
            print("XCOR negative, weird!")
            xcor += abs(xcor.min())
        if ycor.min() < 0:
            print("YCOR negative, weird!")
            ycor += abs(ycor.min())
        arr = np.zeros((xcor.max() + 1, ycor.max() + 1), dtype=np.uint8)
        # TODO: either use parquet or numpy
        try:
            ptrn = np.unpackbits(ptrn_df["data"], bitorder=self.bitorder)
        except IndexError:
            ptrn = np.unpackbits(ptrn_df, bitorder=self.bitorder)
        # TODO: this is strange, added as quick fix on may 9 2021
        ptrn = ptrn[: len(xcor)]
        arr[xcor[:], ycor[:]] = ptrn[0 : len(ptrn) : step]
        arr = arr * 255
        img = Image.fromarray(arr).rotate(90, expand=True)
        img.save(os.path.join(self.debug_folder, filename + ".png"))
        return img

    def readbin(self, filename="test.bin", mode="bytes"):
        """reads a binary data file

        name  -- name of binary file with laser information
        mode  -- parquet, numpy, bytes
        """
        file_path = os.path.join(self.debug_folder, filename)
        bitsinline = int(self.params["bitsinscanline"])
        bytesinline = int(np.ceil(self.params["bitsinscanline"] // 8))
        words_in_line = Spi.words_scanline(self.cfg.laser_timing)
        pixeldata = []

        decompressor = zlib.decompressobj()

        # The global buffer that holds decompressed bytes waiting to be consumed
        decompressed_buffer = b""

        def get_next_decompressed_chunk(f):
            """Reads a chunk of compressed data, decompresses it, and appends the result to the global buffer."""
            nonlocal decompressed_buffer
            # Read a reasonable chunk size, e.g., 4096 bytes of compressed data
            compressed_data = f.read(4096)
            if not compressed_data:
                # End of file: Flush the decompressor to get the last bytes (trailer)
                decompressed_data = decompressor.flush()
            else:
                # Decompress the chunk
                decompressed_data = decompressor.decompress(compressed_data)

            decompressed_buffer += decompressed_data
            return len(decompressed_data)

        def z_read(f, num):
            """
            Reads exactly 'num' decompressed bytes, reading more from the file as needed.
            Returns: The 'num' bytes or raises an EOFError if not enough data is available.
            """
            nonlocal decompressed_buffer
            # Keep reading and decompressing from the file until the buffer has enough bytes
            while len(decompressed_buffer) < num:
                # If we can't get any more decompressed data, it's an EOF
                if (
                    get_next_decompressed_chunk(f) == 0
                    and len(decompressed_buffer) < num
                ):
                    raise EOFError(
                        f"Premature end of file: Requested {num} bytes but only found {len(decompressed_buffer)}."
                    )

            # Extract the requested number of bytes from the front of the buffer
            result = decompressed_buffer[:num]

            # Remove the extracted bytes from the buffer (consume them)
            decompressed_buffer = decompressed_buffer[num:]

            return result

        with open(file_path, "rb") as f:
            # 1. Header
            lanewidth = struct.unpack("<f", z_read(f, 4))[0]
            facetsinlane = struct.unpack("<I", z_read(f, 4))[0]
            lanes = struct.unpack("<I", z_read(f, 4))[0]
            for lane in range(lanes):
                if lane % 2 == 1:
                    direction = 0
                else:
                    direction = 1
                for i in range(facetsinlane):
                    # cmd lst has lenth of 6, there are 9 bytes in a cmd
                    cmdlst = []
                    for _ in range(words_in_line):
                        cmdlst.append(z_read(f, 9))
                    # let's unpack the cmds, all 6 commands start with write
                    # first command has the scanline command
                    # contains direction and steps per line
                    cmd0 = cmdlst[0]
                    assert cmd0[0] == 1  # cmd always starts with write
                    move_word = list(cmd0[1:])[::-1]
                    assert move_word[0] == 3  # scanline
                    move_header = move_word[1:]
                    bits = np.unpackbits(
                        np.array(move_header, dtype=np.uint8), bitorder="litle"
                    )
                    assert bits[0] == direction
                    # convert to binary, reverse, map to string, convert binary string to int
                    half_periodbits = int("".join(map(str, bits[1:][::-1])), 2)
                    stepsperline = int((bitsinline - 1) // half_periodbits) / 2
                    assert stepsperline == self.params["stepsperline"]
                    # other command contain the linedata
                    bitlst = []
                    for cmd in cmdlst[1:]:
                        assert cmd[0] == 1
                        bits = np.unpackbits(
                            np.array(list(cmd[1:])[::-1], dtype=np.uint8),
                            bitorder="litle",
                        )
                        bitlst.extend(bits)
                    bitlst = bitlst[: bytesinline * 8][::-1]
                    pixeldata.extend(bitlst)
        return facetsinlane, lanes, lanewidth, np.packbits(pixeldata)

    def writebin(self, pixeldata, filename="test.bin", compression_level=9):
        """writes pixeldata with parameters to parquet file

        header contains stepsperline, facetsinlane and number of lane
        rest is the pixeldata


        pixeldata  -- must have uneven length
        filename   -- name of binary file to write laserinformation to
        compression_level      -- compression level
        """
        if not os.path.exists(self.debug_folder):
            os.makedirs(self.debug_folder)
        # parquet is more efficient, not supported by micropython
        # micropython ulab has numpy load and save but cannot
        # load object partially, as such default numpy save not used
        lanes = int(np.ceil(self.params["samplexsize"] / self.params["lanewidth"]))
        facetsinlane = int(self.params["facetsinlane"])
        pixeldata = pixeldata.astype(np.uint8)
        host = BaseHost(test=False)
        reconstructed = []
        compressor = zlib.compressobj(level=compression_level)

        with open(os.path.join(self.debug_folder, filename), "wb") as f:

            def z_write(data):
                compressed = compressor.compress(data)
                f.write(compressed)

            # 1. Header:
            z_write(struct.pack("<f", self.params["lanewidth"]))
            z_write(
                struct.pack("<I", int(self.params["facetsinlane"]))
            )  # unsigned int (4 bytes)
            z_write(struct.pack("<I", lanes))  # unsigned int (4 bytes)
            # Overtime packing changed significantly
            bitsinline = int(self.params["bitsinscanline"])
            bytesinline = int(np.ceil(self.params["bitsinscanline"] // 8))
            assert len(pixeldata) == round(facetsinlane * lanes * bytesinline)
            for lane in range(lanes):
                if lane % 2 == 1:
                    direction = 0
                else:
                    direction = 1
                for i in range(facetsinlane):
                    offset = lane * facetsinlane * bytesinline
                    start = i * bytesinline + offset
                    end = (i + 1) * bytesinline + offset
                    linedata = pixeldata[start:end]
                    reconstructed.extend(linedata)
                    bits = np.unpackbits(linedata)[:bitsinline]
                    # reverse, clockwise exposure
                    bits = bits[::-1]
                    bytelst = host.bit_to_byte_list(
                        bits, self.params["stepsperline"], direction
                    )
                    cmdlst = host.byte_to_cmd_list(bytelst)
                    # cmd lst has lenth of 6, there are 9 bytes in a cmd
                    for cmd in cmdlst:
                        z_write(cmd)
            f.write(compressor.flush())


if __name__ == "__main__":
    # PCB / photopaper stepsperline single channel, current 130, 2x per line
    fname = "jittertest"
    interpolator = Interpolator()
    dir_path = os.path.dirname(os.path.realpath(__file__))
    # postscript resolution test
    url = os.path.join(dir_path, "patterns", f"{fname}.svg")
    ptrn = interpolator.patternfile(url)
    # hexastorm.png pixelsize 0.035
    # url = os.path.join(dir_path, "test-patterns", "hexastorm.png")
    # ptrn = interpolator.patternfile(url, pixelsize=0.035)
    print("This can take up to 30 seconds")
    # TODO: zlib can compress the data with a factor over 20
    interpolator.writebin(ptrn, f"{fname}.bin")
    facetsinlane, lanes, lanewidth, arr = interpolator.readbin(f"{fname}.bin")
    assert np.allclose(interpolator.params["lanewidth"], lanewidth, 1e-3)
    assert np.allclose(interpolator.params["facetsinlane"], facetsinlane, 1e-3)
    assert len(arr) == round(
        facetsinlane * lanes * np.ceil(interpolator.params["bitsinscanline"] // 8)
    )
    # TODO: step must be an integer!!
    interpolator.plotptrn(arr, step=1)
