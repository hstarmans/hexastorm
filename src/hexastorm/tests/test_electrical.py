import os
import unittest
import sys
from time import sleep, time

from ..constants import COMMANDS, MOVE_TICKS, WORD_BYTES, wordsinmove
from ..controller import Host, Memfull, executor

upython = False
if sys.implementation.name == "cpython":
    import numpy as np
    import pandas as pd
    import plotext as plt
    from numpy.testing import assert_array_almost_equal
    from gpiozero import LED

    from .. import interpolator
    from ..platforms import Firestarter
else:
    from ulab import numpy as np
    from ..ulabext import assert_array_almost_equal

    upython = True


class Base(unittest.TestCase):
    @classmethod
    def setUpClass(cls, flash=False, mod="all"):
        """builds the FPGA code using Amaranth HDL, Yosys, Nextpnr and icepack

        flash  -- flash FPGA and build
        mod    -- module to build; all or motor
        """
        cls.host = Host()
        cls.mod = mod
        if flash:
            cls.host.build(mod=mod)
        else:
            print("Resetting the machine")
            cls.host.reset()

    @classmethod
    def tearDownClass(cls):
        if not upython:
            import subprocess

            cls.host.enable.close()

            subprocess.run(
                [
                    "raspi-gpio",
                    "set",
                    str(cls.host.platform.enable_pin),
                    "op",
                    "dh",
                ]
            )


class StaticTest(Base):
    @executor
    def test_memfull(self):
        """test if memory can be filled and emptied

        Yosys does not raise error for memories which cannot be synthesized
        You fill the entire memory and see if it is properly emptied
        by ensuring that the position changes as excepted.
        """
        host = self.host
        yield from host.set_parsing(False)
        host.enable_steppers = False
        host.maxtrials = 10
        for _ in range(host.platform.memdepth):
            coeff = [3] * host.platform.motors
            try:
                yield from host.spline_move(MOVE_TICKS, coeff)
            except Memfull:
                pass
        host.maxtrials = 1e5
        self.assertEqual((yield from host.get_state())["mem_full"], True)
        yield from host.set_parsing(True)
        self.assertEqual((yield from host.get_state())["mem_full"], False)
        self.assertEqual((yield from host.get_state())["error"], False)
        self.host.reset()

    @executor
    def test_invalidinstruction(self):
        """write invalid instruction and verify it passes dispatcher"""
        host = self.host
        command = [COMMANDS.WRITE] + [0] * WORD_BYTES
        for _ in range(wordsinmove(host.platform)):
            yield from host.send_command(command)
        sleep(3)
        state = yield from host.get_state()
        self.assertEqual(state["mem_full"], False)
        self.assertEqual(state["error"], True)
        self.host.reset()


class LaserheadTest(Base):
    def test_setlaserpower(self, power=130):
        """test if laser power can be set

        sets value on digipot via I2C
        and verifies this is equal to readed value of digipot
        """
        self.host.laser_current = power
        assert self.host.laser_current == power
        # if this doesnt work check i2cdetect -y 1
        # it should produce a matrix with -- and one value with 28.

    @executor
    def spinprism(self, timeout=15):
        "spin for timeout seconds"
        host = self.host
        yield from host.enable_comp(polygon=True)
        print(f"Spinning prism for {timeout} seconds")
        sleep(timeout)
        yield from host.enable_comp(polygon=False)
        self.assertEqual((yield from host.get_state())["error"], False)

    @executor
    def lasertest(self):
        "enable and disable laser by pressing enter"
        host = self.host
        print("Press enter to turn laser on")
        input()
        yield from host.enable_comp(laser1=True, laser0=False)
        print("Press enter to turn laser off")
        input()
        yield from host.enable_comp(laser1=False)
        self.assertEqual((yield from host.get_state())["error"], False)

    @executor
    def test_diode(self, timeout=3):
        "enable motor, laser and verify photodiode is triggered"
        host = self.host
        res = (yield from host.get_state())["photodiode_trigger"]
        self.assertEqual(res, 0)
        yield from host.enable_comp(laser1=True, polygon=True)
        print(f"Wait for diode trigger, {timeout} seconds")
        sleep(timeout)
        yield from host.enable_comp(laser1=False, polygon=False)
        res = (yield from host.get_state())["photodiode_trigger"]
        self.assertEqual(res, 1)
        self.assertEqual((yield from host.get_state())["error"], False)

    @executor
    def test_stable(self, timeout=3):
        host = self.host
        yield from host.enable_comp(synchronize=True)
        print(f"Wait for synchronization, {timeout} seconds")
        sleep(timeout)
        res = (yield from host.get_state())["photodiode_trigger"]
        self.assertEqual(res, 1)
        yield from host.enable_comp(synchronize=False)
        self.assertEqual((yield from host.get_state())["error"], False)

    @executor
    def test_move(self, dist=10, stepsperline=1, timeout=3):
        """verifies movement during a forward and backward
           scanning move

        dist    -- distance in mm
        timeout -- wait
        """
        host = self.host
        laser_params = host.laser_params
        numblines = round(
            dist
            * host.platform.stepspermm[host.platform.laser_axis]
            * stepsperline
        )
        yield from host.enable_comp(synchronize=True)
        startpos = (yield from host.position).copy()
        host.enable_steppers = True
        print(f"Wait for synchronization, {timeout} seconds")
        sleep(timeout)
        line = [0] * laser_params["BITSINSCANLINE"]
        for direction in [0, 1]:
            for _ in range(numblines):
                yield from host.writeline(line, stepsperline, direction)
            print(f"Wait for move to complete, {timeout} seconds")
            sleep(timeout)
            indx = list(host.platform.stepspermm.keys())
            indx = indx.index(host.platform.laser_axis)
            # assume y axis is 1
            startpos[indx] += dist if direction else -dist
            assert_array_almost_equal(
                (yield from host.position), startpos, decimal=1
            )
        yield from host.writeline([])
        print(f"Wait for stopline to execute, {timeout} seconds")
        sleep(timeout)
        self.assertEqual((yield from host.get_state())["error"], False)
        yield from host.enable_comp(synchronize=False)
        self.host.enable_steppers = False

    @executor
    def test_scanline(self, timeout=3, numblines=1000):
        host = self.host
        line = [1] * host.laser_params["BITSINSCANLINE"]
        for _ in range(numblines):
            yield from host.writeline(line)
        yield from host.writeline([])
        print(f"Wait for stopline to execute, {timeout} seconds")
        sleep(timeout)
        self.assertEqual((yield from host.get_state())["error"], False)
        yield from host.enable_comp(synchronize=False)


class MotorTest(Base):
    """Test BLDC motor

    There are no virtual tests for the prism motor.
    The device is debugged by communicating a debug word via SPI.
    The debug mode is set via MOTORDEBUG, in platforms.py.
    This program can then be used to analyze the results.
    """

    def finish(self, output):
        print(
            "Measurement finished in mode "
            + f"{self.host.laser_params['MOTORDEBUG']}"
        )
        output.to_csv("measurement.csv")
        mode = self.host.laser_params["MOTORDEBUG"]
        if mode == "hallstate":
            output = output[output["word_0"] != 0]
            output = output.replace(
                {"word_0": {1: 1, 2: 3, 3: 2, 4: 5, 5: 6, 6: 4}}
            )
            # compute fraction
            # ideally should be 1/6
            print(
                (
                    output.rename(columns={"time": "fraction"})
                    .groupby(["word_0"])
                    .count()
                    .transform(lambda x: x / x.sum())
                )
            )
        elif mode == "PIcontrol":
            print(output[["word_0"]].describe())
        elif mode == "ticksinfacet":
            print(output[["word_0", "word_1"]].describe())

    @executor
    def test_main(self, debug=False):
        """turns on the motor board and retrieves the rotor frequency

        Method runs for ever, can be interrupted with keyboard interrupt.
        """
        host = self.host
        if self.mod == "motor":
            blocking = False
        else:
            blocking = True

        mode = host.laser_params["MOTORDEBUG"]
        start = time()
        if mode == "hallstate":
            starttime = 5
            measurementtime = 15
            totaltime = 120
        elif mode == "PIcontrol":
            starttime = 5
            measurementtime = 15
            totaltime = 60
        elif mode == "ticksinfacet":
            starttime = 5
            measurementtime = 15
            totaltime = 60
        else:
            starttime = 15
            measurementtime = 15
            totaltime = 60
        output = pd.DataFrame(columns=["time"])
        measurement = pd.DataFrame(columns=["time"])
        print(f"Waiting {starttime} seconds to start measurement.")
        if self.mod == "all":
            if mode == "ticksinfacet":
                yield from host.enable_comp(synchronize=True)
            else:
                yield from host.enable_comp(polygon=True)

        print("Starting measurement")
        plt.title(f"Streaming Data in {mode}")
        # plt.clc()
        try:
            sleep(starttime)
            while True:
                if (time() - start) >= totaltime:
                    self.finish(measurement)
                    break
                # TODO: why does true not work
                words = yield from host.get_motordebug(blocking=False)
                try:
                    dct = {"time": [time() - start]}
                    for idx, word in enumerate(words):
                        dct[f"word_{idx}"] = [word]
                    if debug:
                        print(dct)
                        sleep(0.1)
                    frame1 = pd.DataFrame(dct)
                    output = pd.concat([output, frame1], ignore_index=True)
                    if time() - start > measurementtime:
                        measurement = pd.concat(
                            [measurement, frame1], ignore_index=True
                        )
                    if mode in [
                        "cycletime",
                        "ticksinfacet",
                        "PIcontrol",
                    ]:
                        plt.clf()
                        plt.clt()  # to clear the terminal
                        plt.cld()  # to clear the data only
                        plt.xlim(0, totaltime)
                        if mode == "cycletime":
                            plt.ylim(0, 4000)
                            plt.title("Speed in RPM")
                            plt.xlabel("Time [seconds]")
                            plt.ylabel("Speed [RPM]")
                            plt.scatter(
                                output["time"], output["word_0"], label="speed"
                            )
                        elif mode == "PIcontrol":
                            plt.ylim(0, 4000)
                            plt.title("PI controller")
                            plt.xlabel("Time [seconds]")
                            plt.ylabel("Counter")
                            plt.scatter(
                                output["time"], output["word_0"], label="speed"
                            )
                            plt.scatter(
                                output["time"],
                                output["word_1"],
                                label="control",
                            )
                            # A PRINT COMMAND will make the plot fail
                        elif mode == "ticksinfacet":
                            plt.ylim(0, 4000)
                            plt.title("Ticksinfacet")
                            plt.xlabel("Time [seconds]")
                            plt.ylabel("Counter")
                            plt.scatter(
                                output["time"],
                                output["word_0"],
                                label="speed hall",
                            )
                            plt.scatter(
                                output["time"],
                                output["word_1"],
                                label="speed diode",
                            )
                        plt.sleep(0.1)
                        # open issue; only happens after reboot
                        #             if script is run succesful once
                        #             it is fixed
                        # https://github.com/piccolomo/plotext/issues/185
                        try:
                            plt.show()
                        except IndexError:
                            print(
                                "Aborting due to strange plotext bug, try restart."
                            )
                            # break

                except ValueError as e:
                    print(e)
        except KeyboardInterrupt:
            self.finish(measurement)
            pass
        finally:
            if self.mod == "motor":
                reset_pin = LED(self.host.platform.reset_pin)
                reset_pin.off()
                reset_pin.close()
                # gpiozero cleans up pins
                # this ensures pin is kept off
                subprocess.run(
                    [
                        "raspi-gpio",
                        "set",
                        str(self.host.platform.reset_pin),
                        "op",
                        "dl",
                    ]
                )
            else:
                yield from host.enable_comp(polygon=False)
            print("Interrupted, exiting")


class MoveTest(Base):
    """Test movement core"""

    @executor
    def readpin(self):
        """test if you can detect triggers of the limit switches

        This is typically executed manually by placing a sheet of paper
        in the cavity of the optical switch.
        """
        self.host.enable_steppers = False
        try:
            while True:
                dct = yield from self.host.get_state()
                print(
                    f"[x, y, z] is [{dct['x']}, " + f"{dct['y']}, {dct['z']}]"
                )
                sleep(1)
        except KeyboardInterrupt:
            pass

    @executor
    def motorenable(self):
        """test if motors are enabled and execution is enabled/disabled
        via communication with FPGA"""
        self.host.enable_steppers = True
        print("Check manually if axes are blocked and require force to move.")
        input()
        self.host.enable_steppers = False
        # needs to be iteratable
        yield

    @executor
    def multiplemove(self, decimals=1):
        """test if motors move

        decimals -- number of decimals
        """
        motors = self.host.platform.motors
        dist = np.array([1, 1, 1])
        startpos = (yield from self.host.position).copy()
        for direction in [-1, 1]:
            self.assertEqual(
                (yield from self.host.get_state())["error"], False
            )
            self.host.enable_steppers = True
            current = (yield from self.host.position).copy()
            disp = dist * direction
            yield from self.host.gotopoint(
                position=disp, speed=[1] * motors, absolute=False
            )
            sleep(3)
            assert_array_almost_equal(
                (yield from self.host.position),
                current + disp,
                decimal=decimals,
            )
        assert_array_almost_equal(
            (yield from self.host.position), startpos, decimal=decimals
        )
        self.host.enable_steppers = False


class PrintTest(Base):
    @executor
    def test_dose(
        self,
        lines=10,
        thickness=100,
        stepsperline=None,
        orthogonal=False,
    ):
        """prints lines with thickness in microns for a range of stagespeeds
           in steps per line

        For stage speed in mm/s, lines are made of thickness micrometers
        wide.

        number of lines -- number of lines made per dosage
        thickness       -- line thickness in number of scanlines
        stepsperline    -- stagespeed of optical head in steps per line
        orthogonal      -- orthogonal
        """
        if stepsperline is None:
            stepsperline = [0.25, 0.5, 1]

        host = self.host

        laser_var = host.platform.laser_var

        lines_per_second = laser_var["RPM"] / 60 * laser_var["FACETS"]
        stepspermm = host.platform.stepspermm[host.platform.laser_axis]

        stagespeeds = lines_per_second / (stepspermm * np.array(stepsperline))

        print(f"Stagepeeds are {stagespeeds.round(2)} mm/s")

        # laser diameter estimated to be 60 microns
        if max(stepsperline) * (1 / stepspermm) > 0.03:
            # checks Nyquist criterion
            raise Exception("Lines too far apart, exposure is not uniform.")
        LANEWIDTH = 5.4
        self.host.enable_steppers = True
        print("Homing X and Y axis")
        yield from host.home_axes([1, 1, 0])
        print("Move to start")
        yield from host.gotopoint([70, 8, 0], absolute=False)
        bitsinline = host.laser_params["BITSINSCANLINE"]

        if orthogonal:
            laser_on = [1] * bitsinline
            laser_off = [0] * bitsinline
        else:
            times = 6
            print(f"splitting lines {times}")
            remainder = bitsinline % times
            bitlst = []
            for i in range(times):
                if i % 2:
                    bitlst += [1] * (bitsinline // times)
                else:
                    bitlst += [0] * (bitsinline // times)
            bitlst += remainder * [0]
            laser_on = laser_off = bitlst

        for lane, steps in enumerate(stepsperline):
            if lane > 0:
                print("Moving in x-direction for next lane")
                yield from host.gotopoint([LANEWIDTH, 0, 0], absolute=False)
            if lane % 2 == 1:
                direction = 0
                print("Start exposing forward lane")
            else:
                direction = 1
                print("Start exposing back lane")
            for _ in range(lines):
                scanlines = round(thickness / steps)
                if direction:
                    bitlst = laser_on
                else:
                    bitlst = laser_off
                for _ in range(scanlines):
                    yield from host.writeline(
                        bitlst=bitlst, stepsperline=steps, direction=direction
                    )
                if direction:
                    bitlst = laser_off
                else:
                    bitlst = laser_on
                for _ in range(scanlines):
                    yield from host.writeline(
                        bitlst=bitlst, stepsperline=steps, direction=direction
                    )
            # send stopline
            yield from host.writeline([])
        # TODO: this is needed as otherwise synchronize is ignored
        sleep(3)
        print("Waiting for stopline to execute")
        yield from self.host.enable_comp(synchronize=False)
        self.host.enable_steppers = False
        print("Finished exposure")

    @executor
    def test_moveoffset(self, offsets=None, stepsperline=1, thickness=300):
        """prints lanes with different parallel offset

        offsets -- offset parallel to scanline between lanes
        stepsperline -- number of steps between scanlines
        thickness -- thickness in number of scanines
        """
        # TODO: this was introduced to meet pyflake, probably should be
        # pased as tuple
        if offsets is None:
            offsets = [5.3, 5.3, 5.4, 5.4]

        host = self.host
        host.enable_steppers = True
        print("Homing X and Y axis")
        yield from host.home_axes([1, 1, 0])
        print("Move to start")
        yield from host.gotopoint([70, 0, 0], absolute=False)
        bitsinline = host.laser_params["BITSINSCANLINE"]
        laser_on = [1] * bitsinline
        # you don't move first lane so offset is 0
        offsets = [0] + offsets
        for lane, offset in enumerate(offsets):
            if lane > 0:
                print("Moving in x-direction for next lane")
                yield from host.gotopoint([offset, 0, 0], absolute=False)
            if lane % 2 == 1:
                direction = 0
                print("Start exposing forward lane")
            else:
                direction = 1
                print("Start exposing back lane")
            for _ in range(thickness):
                yield from host.writeline(
                    bitlst=laser_on,
                    stepsperline=stepsperline,
                    direction=direction,
                )
            # send stopline
            yield from host.writeline([])
        # disable scanhead
        sleep(3)
        print("Waiting for stopline to execute")
        yield from self.host.enable_comp(synchronize=False)
        self.host.enable_steppers = False
        print("Finished exposure")

    # TODO: code clone of the above!!
    @executor
    def test_lineoffset(
        self,
        offset_lines=None,
        stepsperline=1,
        thickness=600,
        orthogonal=True,
    ):
        """prints lanes with different offset in lines

        it is assumed lanes do not overlap due to backlash
        see https://en.wikipedia.org/wiki/Backlash_(engineering)

        stepsperline -- number of steps between scanlines
        thickness -- thickness in number of scanines
        """
        if offset_lines is None:
            offset_lines = [1, 1, 1, 1, 1]
        host = self.host
        host.enable_steppers = True
        print("Homing X and Y axis")
        yield from host.home_axes([1, 1, 0])
        print("Move to start")
        yield from host.gotopoint([70, 0, 0], absolute=False)
        bitsinline = host.laser_params["BITSINSCANLINE"]
        laser_on = [1] * bitsinline
        laser_off = [0] * bitsinline

        # first offset zero by definition
        offset_lines += [0] + offset_lines
        offset = 5.4
        for lane, offset_line in enumerate(offset_lines):
            if lane > 0:
                print("Moving in x-direction for next lane")
                yield from host.gotopoint([offset, 0, 0], absolute=False)
            if lane % 2 == 1:
                direction = 0
                print("Start exposing forward lane")
            else:
                direction = 1
                print("Start exposing back lane")
            if lane > 0:
                for _ in range(offset_line):
                    yield from host.writeline(
                        bitlst=laser_off,
                        stepsperline=stepsperline,
                        direction=direction,
                    )
            for _ in range(thickness):
                yield from host.writeline(
                    bitlst=laser_on,
                    stepsperline=stepsperline,
                    direction=direction,
                )
            # send stopline
            yield from host.writeline([])
        # disable scanhead
        sleep(3)
        print("Waiting for stopline to execute")
        yield from self.host.enable_comp(synchronize=False)
        self.host.enable_steppers = False
        print("Finished exposure")

    @executor
    def test_print(self):
        """the LDgraphy test pattern is printed"""
        host = self.host
        host.enable_steppers = True
        if not upython:
            from pathlib import Path

            dir_path = os.path.dirname(interpolator.__file__)
            FILENAME = Path(dir_path, "debug", "test.bin")
        else:
            FILENAME = "sd/fpga/job.bin"
        # it assumed the binary is already created and
        # in the interpolator folder
        if not os.path.isfile(FILENAME):
            raise Exception("File not found")
        FACETS_IN_LANE = 5377
        LANEWIDTH = 5.45
        bytesinline = int(host.laser_params["BITSINSCANLINE"] / 8)
        stepsperline = 1
        # z is not homed as it should be already in
        # position so laser is in focus
        self.host.enable_steppers = True
        self.host.laser_current = 130  # assuming 1 channel
        print("Homing X and Y axis")
        yield from host.home_axes([1, 1, 0])
        print("Move to start")
        # scaning direction offset is needed to prevent lock with home
        yield from host.gotopoint([70, 5, 0], absolute=False)
        print("Reading binary")
        # enable scanhead
        yield from self.host.enable_comp(synchronize=True)
        with open(FILENAME, "rb") as f:
            lane = 0
            line_data = np.frombuffer(f.read(bytesinline), dtype=np.uint8)
            while True:
                if not line_data:
                    break
                print(f"Exposing lane {lane}")
                if lane > 0:
                    print("Moving in x-direction for next lane")
                    yield from host.gotopoint(
                        [LANEWIDTH, 0, 0], absolute=False
                    )
                if lane % 2 == 1:
                    direction = 0
                    print("Start exposing forward lane")
                else:
                    direction = 1
                    print("Start exposing back lane")
                for line in range(FACETS_IN_LANE):
                    # reverse, as exposure is inversed
                    # line_data = line_data[::-1]
                    yield from host.writeline(
                        bitlst=line_data,
                        stepsperline=stepsperline,
                        direction=direction,
                    )
                    line_data = np.frombuffer(
                        f.read(bytesinline), dtype=np.uint8
                    )
                    if not line_data:
                        break
                lane += 1
            # send stopline
            yield from host.writeline([])
        # disable scanhead
        sleep(3)
        print("Waiting for stopline to execute")
        yield from self.host.enable_comp(synchronize=False)
        self.host.enable_steppers = False
        print("Finished exposure")


if __name__ == "__main__":
    unittest.main()
