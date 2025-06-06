import sys

if sys.implementation.name == "cpython":
    raise Exception("Module should be called from micropython")


import unittest
from time import sleep, ticks_ms
from math import isclose


from ulab import numpy as np

from ..config import COMMANDS, MOVE_TICKS, WORD_BYTES, wordsinmove
from ..controller import Host, Memfull, executor
from ..ulabext import assert_array_almost_equal


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
        pass


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
    def lasertest(self, laser1=True):
        "enable and disable laser by pressing enter"
        host = self.host
        print(f"Press enter to turn laser {1 if laser1 else 0} on")
        input()
        yield from host.enable_comp(laser1=laser1, laser0=(not laser1))
        print("Press enter to turn laser off")
        input()
        yield from host.enable_comp(laser1=False, laser0=False)
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
            dist * host.platform.stepspermm[host.platform.laser_axis] * stepsperline
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
            assert_array_almost_equal((yield from host.position), startpos, decimal=1)
        yield from host.writeline([])
        print(f"Wait for stopline to execute, {timeout} seconds")
        sleep(timeout)
        self.assertEqual((yield from host.get_state())["error"], False)
        yield from host.enable_comp(synchronize=False)
        self.host.enable_steppers = False
        # TODO: parsing is left open
        # (yield from self.host.set_parsing(False))

    @executor
    def test_scanline(self, numblines=1_000, repeat=True, singlefacet=False):
        host = self.host
        line = [1] * host.laser_params["BITSINSCANLINE"]
        yield from host.enable_comp(singlefacet=singlefacet)
        for _ in range(8):
            yield from host.writeline(line)
        print(
            f"waiting for polygon to stabilize and laserhead to process {numblines} lines"
        )
        sleep(3)
        starttime = ticks_ms()  # milliseconds
        if repeat:
            yield from host.writeline(line, repetitions=numblines)
        else:
            for _ in range(numblines):
                yield from host.writeline(line)
        elapsed = (ticks_ms() - starttime) / 1000  # seconds
        measured_freq = numblines / elapsed  # hertz
        yield from host.writeline([])
        # 3000 RPM
        #    100 khz --> pass
        #    400 Khz --> pass, fail for repeat is False
        expected_freq = host.platform.laser_timing["RPM"] / 60
        if not singlefacet:
            expected_freq *= host.platform.laser_timing["FACETS"]
        print(
            f"line rate measured: {measured_freq:.2f},  expected {expected_freq:.2f} in Hertz"
        )
        # measured freq is higher as it still need to clean the buffer
        self.assertEqual(isclose(measured_freq, expected_freq, rel_tol=0.1), True)
        self.assertEqual((yield from host.get_state())["error"], False)
        yield from host.enable_comp(synchronize=False)


# port to micropython
# import numpy as np
# import pandas as pd
# import plotext as plt
# from numpy.testing import assert_array_almost_equal
# from gpiozero import LED

# from .. import interpolator
# from ..platforms import Firestarter


# class MotorTest(Base):
#     """Test BLDC motor

#     There are no virtual tests for the prism motor.
#     The device is debugged by communicating a debug word via SPI.
#     The debug mode is set via MOTORDEBUG, in platforms.py.
#     This program can then be used to analyze the results.
#     """

#     def finish(self, output):
#         print(
#             "Measurement finished in mode "
#             + f"{self.host.laser_params['MOTORDEBUG']}"
#         )
#         output.to_csv("measurement.csv")
#         mode = self.host.laser_params["MOTORDEBUG"]
#         if mode == "hallstate":
#             output = output[output["word_0"] != 0]
#             output = output.replace(
#                 {"word_0": {1: 1, 2: 3, 3: 2, 4: 5, 5: 6, 6: 4}}
#             )
#             # compute fraction
#             # ideally should be 1/6
#             print(
#                 (
#                     output.rename(columns={"time": "fraction"})
#                     .groupby(["word_0"])
#                     .count()
#                     .transform(lambda x: x / x.sum())
#                 )
#             )
#         elif mode == "PIcontrol":
#             print(output[["word_0"]].describe())
#         elif mode == "ticksinfacet":
#             print(output[["word_0", "word_1"]].describe())

#     @executor
#     def test_main(self, debug=False):
#         """turns on the motor board and retrieves the rotor frequency

#         Method runs for ever, can be interrupted with keyboard interrupt.
#         """
#         host = self.host
#         if self.mod == "motor":
#             blocking = False
#         else:
#             blocking = True

#         mode = host.laser_params["MOTORDEBUG"]
#         start = time()
#         if mode == "hallstate":
#             starttime = 5
#             measurementtime = 15
#             totaltime = 120
#         elif mode == "PIcontrol":
#             starttime = 5
#             measurementtime = 15
#             totaltime = 60
#         elif mode == "ticksinfacet":
#             starttime = 5
#             measurementtime = 15
#             totaltime = 60
#         else:
#             starttime = 15
#             measurementtime = 15
#             totaltime = 60
#         output = pd.DataFrame(columns=["time"])
#         measurement = pd.DataFrame(columns=["time"])
#         print(f"Waiting {starttime} seconds to start measurement.")
#         if self.mod == "all":
#             if mode == "ticksinfacet":
#                 yield from host.enable_comp(synchronize=True)
#             else:
#                 yield from host.enable_comp(polygon=True)

#         print("Starting measurement")
#         plt.title(f"Streaming Data in {mode}")
#         # plt.clc()
#         try:
#             sleep(starttime)
#             while True:
#                 if (time() - start) >= totaltime:
#                     self.finish(measurement)
#                     break
#                 # TODO: why does true not work
#                 words = yield from host.get_motordebug(blocking=False)
#                 try:
#                     dct = {"time": [time() - start]}
#                     for idx, word in enumerate(words):
#                         dct[f"word_{idx}"] = [word]
#                     if debug:
#                         print(dct)
#                         sleep(0.1)
#                     frame1 = pd.DataFrame(dct)
#                     output = pd.concat([output, frame1], ignore_index=True)
#                     if time() - start > measurementtime:
#                         measurement = pd.concat(
#                             [measurement, frame1], ignore_index=True
#                         )
#                     if mode in [
#                         "cycletime",
#                         "ticksinfacet",
#                         "PIcontrol",
#                     ]:
#                         plt.clf()
#                         plt.clt()  # to clear the terminal
#                         plt.cld()  # to clear the data only
#                         plt.xlim(0, totaltime)
#                         if mode == "cycletime":
#                             plt.ylim(0, 4000)
#                             plt.title("Speed in RPM")
#                             plt.xlabel("Time [seconds]")
#                             plt.ylabel("Speed [RPM]")
#                             plt.scatter(
#                                 output["time"], output["word_0"], label="speed"
#                             )
#                         elif mode == "PIcontrol":
#                             plt.ylim(0, 4000)
#                             plt.title("PI controller")
#                             plt.xlabel("Time [seconds]")
#                             plt.ylabel("Counter")
#                             plt.scatter(
#                                 output["time"], output["word_0"], label="speed"
#                             )
#                             plt.scatter(
#                                 output["time"],
#                                 output["word_1"],
#                                 label="control",
#                             )
#                             # A PRINT COMMAND will make the plot fail
#                         elif mode == "ticksinfacet":
#                             plt.ylim(0, 4000)
#                             plt.title("Ticksinfacet")
#                             plt.xlabel("Time [seconds]")
#                             plt.ylabel("Counter")
#                             plt.scatter(
#                                 output["time"],
#                                 output["word_0"],
#                                 label="speed hall",
#                             )
#                             plt.scatter(
#                                 output["time"],
#                                 output["word_1"],
#                                 label="speed diode",
#                             )
#                         plt.sleep(0.1)
#                         # open issue; only happens after reboot
#                         #             if script is run succesful once
#                         #             it is fixed
#                         # https://github.com/piccolomo/plotext/issues/185
#                         try:
#                             plt.show()
#                         except IndexError:
#                             print(
#                                 "Aborting due to strange plotext bug, try restart."
#                             )
#                             # break

#                 except ValueError as e:
#                     print(e)
#         except KeyboardInterrupt:
#             self.finish(measurement)
#             pass
#         finally:
#             if self.mod == "motor":
#                 reset_pin = LED(self.host.platform.reset_pin)
#                 reset_pin.off()
#                 reset_pin.close()
#                 # gpiozero cleans up pins
#                 # this ensures pin is kept off
#                 subprocess.run(
#                     [
#                         "raspi-gpio",
#                         "set",
#                         str(self.host.platform.reset_pin),
#                         "op",
#                         "dl",
#                     ]
#                 )
#             else:
#                 yield from host.enable_comp(polygon=False)
#             print("Interrupted, exiting")


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
                print(f"[x, y, z] is [{dct['x']}, " + f"{dct['y']}, {dct['z']}]")
                sleep(1)
        except KeyboardInterrupt:
            pass

    def motorenable(self):
        """test if motors are enabled and execution is enabled/disabled
        via communication with FPGA"""
        self.host.enable_steppers = True
        print("Check manually axes are blocked and require force to move, press enter.")
        input()
        self.host.enable_steppers = False

    @executor
    def multiplemove(self, decimals=1):
        """test if motors move

        decimals -- number of decimals
        """
        motors = self.host.platform.motors
        dist = np.array([10, 10, 0])
        startpos = (yield from self.host.position).copy()
        # does not work with -1, 1
        for direction in [1, -1]:
            self.assertEqual((yield from self.host.get_state())["error"], False)
            self.host.enable_steppers = True
            current = (yield from self.host.position).copy()
            disp = dist * direction
            yield from self.host.gotopoint(
                position=disp, speed=[1] * motors, absolute=False
            )
            sleep(3)  # wait for execution of all command in ringbuffer
            assert_array_almost_equal(
                (yield from self.host.position),
                current + disp,
                decimal=decimals,
            )
        assert_array_almost_equal(
            (yield from self.host.position), startpos, decimal=decimals
        )
        self.host.enable_steppers = False


if __name__ == "__main__":
    unittest.main()
