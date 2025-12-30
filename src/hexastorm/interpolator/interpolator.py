import math
from pathlib import Path
from io import BytesIO
import struct
import zlib
from time import time

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

    def __init__(self, stepsperline: int = 1):
        self.cfg = PlatformConfig(test=False)
        self.params = self._init_parameters(stepsperline)
        self.params = self.downsample(self.params)

        self.current_dir = Path(__file__).parent.resolve()
        self.debug_folder = self.current_dir / "debug"
        self.debug_folder.mkdir(exist_ok=True)  # Creates dir if missing

    def _init_parameters(self, stepsperline: int = 1):
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

        # Create Numba-compatible dictionary
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

    def svgtopil(self, svg_filepath: Path) -> Image.Image:
        """converts SVG snippets to a PIL Image"""
        with open(svg_filepath, "rb") as f:
            svg_data = f.read()
        dpi = 25.4 / self.params["samplegridsize"]
        png_data = svg2png(bytestring=svg_data, dpi=dpi)
        # rotation used to align jitter and crosstest correctly to laser
        return Image.open(BytesIO(png_data)).rotate(-90, expand=True)

    def pstopil(self, filepath: Path, pixelsize: float = 0.3527777778) -> Image.Image:
        """converts postscript file to a PIL Image

        filepath  --  path to postcript file
        pixelsize -- pixel size in mm
        """
        img = Image.open(filepath)
        scale = pixelsize / self.params["samplegridsize"]
        img.load(scale=scale)
        return img

    def imgtopil(self, filepath: Path, pixelsize: float) -> Image.Image:
        """converts image to a PIL image

        filepath  --  path to PIL image, e.g. PNG or BMP
        """
        img = Image.open(filepath)
        scale = pixelsize / self.params["samplegridsize"]
        img = img.resize([round(x * scale) for x in img.size])
        return img

    def piltoarray(self, img: Image.Image) -> np.ndarray:
        """converts PIL Image to numpy array

        Method also changes the settings of on the objects
        and clips to the area of interest
        pil  --  input image as
        """
        img_array = np.array(img.convert("1"))
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
        # Helper to keep code DRY (Don't Repeat Yourself)
        return (
            fxpos(0, self.params)
            - fxpos(self.params["bitsinscanline"] - 1, self.params)
        ) * self.params["samplegridsize"]

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
        ctime = time()
        file_path = Path(url)

        if file_path.suffix == ".svg":
            pil = self.svgtopil(file_path)
            pil.save(self.debug_folder / "debug.png")
        elif file_path.suffix == ".ps":
            pil = self.pstopil(file_path)
        else:
            pil = self.imgtopil(file_path, pixelsize)
        layerarr = self.piltoarray(pil).astype(np.uint8)

        if test:
            img = Image.fromarray(layerarr.astype(np.uint8) * 255)
            img.save(self.debug_folder / "nyquistcheck.png")
            layerarr = np.ones_like(layerarr)
        print("Retrieved image")
        print(f"Elapsed {time() - ctime:.2f} seconds")
        ids = self.createcoordinates()
        print("Created coordinates for interpolation")
        print(f"Elapsed {time() - ctime:.2f} seconds")

        x_coords = ids[1]
        y_coords = ids[0]

        # Bounds Checking
        # Create a mask for points that actually land on the image
        mask = (
            (x_coords >= 0)
            & (x_coords < layerarr.shape[1])
            & (y_coords >= 0)
            & (y_coords < layerarr.shape[0])
        )

        # Fast Sampling (Advanced Indexing)
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

        save_path = self.debug_folder / f"{filename}.png"
        img.save(save_path)

        return img

    def readbin(self, filename="test.bin", mode="bytes"):
        """reads a binary data file

        name  -- name of binary file with laser information
        mode  -- parquet, numpy, bytes
        """
        file_path = self.debug_folder / filename
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
                        np.array(move_header, dtype=np.uint8), bitorder="little"
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
        """
        Writes pixeldata to binary file with massive performance improvements.
        Inlines SPI chunking and vectorizes data processing.
        """
        self.debug_folder.mkdir(parents=True, exist_ok=True)

        # 1. Setup Parameters
        params = self.params
        lanes = int(np.ceil(params["samplexsize"] / params["lanewidth"]))
        facets = int(params["facetsinlane"])
        bytes_in_line = int(np.ceil(params["bitsinscanline"] / 8))

        # 2. Reshape & Global Reversal (Vectorized)
        # Reshape to (Lanes, Facets, BytesPerLine) for easy processing
        try:
            expected_size = lanes * facets * bytes_in_line
            if pixeldata.size != expected_size:
                print(f"Resizing data: {pixeldata.size} -> {expected_size}")
                pixeldata.resize(expected_size, refcheck=False)

            grid = pixeldata.reshape(lanes, facets, bytes_in_line)
        except ValueError as e:
            raise ValueError(f"Data shape mismatch: {e}")

        # Reverse the image data bytes (Corresponds to bits[::-1] in original code)
        grid = grid[:, :, ::-1]

        # 3. Pre-calculate Headers
        # Calculate the 7-byte configuration header once
        def make_header(direction):
            steps = params["stepsperline"]
            scan_len = params["bitsinscanline"]
            half_period = int((scan_len - 1) // (steps * 2))

            # (HalfPeriod << 1) | Direction
            payload = (half_period << 1) | direction
            return payload.to_bytes(7, "little")

        # Create full headers: [CMD_SCANLINE (0x03 usually)] + [Config Bytes]
        # Note: Your original code used Spi.Instructions.scanline.
        # Ensure this matches the byte value (e.g., b'\x03')
        scanline_cmd = Spi.Instructions.scanline.to_bytes(1, "big")
        header_bwd = scanline_cmd + make_header(0)
        header_fwd = scanline_cmd + make_header(1)

        # Calculate Padding
        total_len = len(header_fwd) + bytes_in_line
        pad_len = (8 - (total_len % 8)) % 8
        padding = b"\x00" * pad_len

        # Pre-calculate the Write Command Byte for SPI
        # Your code: Spi.Commands.write.to_bytes(1, "big")
        spi_write_cmd = Spi.Commands.write.to_bytes(1, "big")

        # 4. Write Loop
        compressor = zlib.compressobj(level=compression_level)
        out_path = self.debug_folder / filename

        # 1MB Buffer to minimize disk/compressor overhead
        IO_BUFFER_SIZE = 1024 * 1024
        write_buffer = bytearray()

        with open(out_path, "wb") as f:
            # File Header
            f.write(compressor.compress(struct.pack("<f", params["lanewidth"])))
            f.write(compressor.compress(struct.pack("<I", facets)))
            f.write(compressor.compress(struct.pack("<I", lanes)))

            for lane_idx in range(lanes):
                # Select Header
                is_forward = lane_idx % 2 == 0
                header = header_fwd if is_forward else header_bwd

                # Get all facets for this lane
                lane_data = grid[lane_idx]

                for facet_idx in range(facets):
                    # Assemble the full raw payload for this line
                    # [Header] + [Image Data (reversed)] + [Padding]
                    full_payload = header + lane_data[facet_idx].tobytes() + padding

                    # INLINED SPI CHUNKING (Replaces byte_to_cmd_list)
                    # Loop over payload in 8-byte chunks
                    for i in range(0, len(full_payload), 8):
                        chunk = full_payload[i : i + 8]

                        # CRITICAL: Reverse the chunk for SPI endianness
                        # This matches list(reversed(chunk)) in your reference code
                        chunk_reversed = chunk[::-1]

                        # Append [Write Byte] + [Reversed Chunk]
                        write_buffer.extend(spi_write_cmd)
                        write_buffer.extend(chunk_reversed)

                    # Flush buffer to compressor periodically
                    if len(write_buffer) >= IO_BUFFER_SIZE:
                        f.write(compressor.compress(write_buffer))
                        write_buffer = bytearray()

            # Final Cleanup
            if write_buffer:
                f.write(compressor.compress(write_buffer))
            f.write(compressor.flush())


if __name__ == "__main__":
    # PCB / photopaper stepsperline single channel, current 130, 2x per line
    fname = "jittertest"
    interpolator = Interpolator()
    dir_path = Path(__file__).parent.resolve()
    # postscript resolution test

    url = dir_path / "patterns" / f"{fname}.svg"
    ptrn = interpolator.patternfile(url)
    # hexastorm.png pixelsize 0.035
    # url = dir_path / "test-patterns" / "hexastorm.png"
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
