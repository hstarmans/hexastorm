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
