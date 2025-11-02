import machine
from machine import Pin, PWM
import time
import random
from hexastorm.fpga_host.micropython import ESP32Host

# Configuration
TEST_DURATION_SECONDS = 15 * 60  # 15 minutes
PROGRESS_UPDATE_INTERVAL_SECONDS = 30
SPI_BUS = 2
# Nyquist rate sampling says max is half the clock rate
# In practice, 1/4 clock rate is more reliable
# If you use single stage synchronization and the oscillator from the ESP32S3
# maxed out you can go up to 12MHz.
# Currently, the biggest speed blocker is that data is sent in chunks not continuous.
# 12 MHz onboard / with 2.9 MHz baudrate,
# 24 MHz onboard with 5 MHz baudrate (does not work with 2.9 MHz)
SPI_BAUDRATE = int(5e6)
SPI_SCK_PIN = 12
SPI_MOSI_PIN = 13
SPI_MISO_PIN = 11
FPGA_SELECT_PIN = 9
DATA_LENGTH = (
    12  # Number of bytes to send.  Important: changed from len(bts) to a constant
)


def run_spi_test(
    duration_seconds=TEST_DURATION_SECONDS,
    update_interval_seconds=PROGRESS_UPDATE_INTERVAL_SECONDS,
):
    """
    Runs an SPI test for a specified duration, sending random data and checking the response.

    Args:
        duration_seconds: The duration of the test in seconds.
        update_interval_seconds: The interval for printing progress updates in seconds.
    """
    hst = ESP32Host()
    hst.reset()

    # Initialize SPI
    spi = machine.SPI(
        SPI_BUS,
        baudrate=SPI_BAUDRATE,
        polarity=0,
        phase=1,
        sck=machine.Pin(SPI_SCK_PIN),
        mosi=machine.Pin(SPI_MOSI_PIN),
        miso=machine.Pin(SPI_MISO_PIN),
    )

    fpga_select = machine.Pin(FPGA_SELECT_PIN, machine.Pin.OUT)
    fpga_select.value(1)  # Start with FPGA deselected

    start_time = time.time()
    next_update_time = start_time + update_interval_seconds
    iteration_count = 0
    error_count = 0

    print(f"Starting SPI test for {duration_seconds} seconds...")
    print(f"Progress updates every {update_interval_seconds} seconds.")
    print("Max speed microdot can handle is typically lower!")
    while time.time() - start_time < duration_seconds:
        iteration_count += 1
        # Generate random data for each test iteration
        data = bytearray(random.randint(0, 255) for _ in range(DATA_LENGTH))
        response = bytearray([0] * DATA_LENGTH)

        fpga_select.value(0)  # Select FPGA
        try:
            spi.write_readinto(data, response)
        except Exception as e:
            print(f"Error during SPI communication: {e}")
            error_count += 1
            fpga_select.value(1)
            continue
        fpga_select.value(1)  # Deselect FPGA

        # Check the response.  Only check the part of the response that contains the sent data
        try:
            assert data[:-2] == response[2:]
        except AssertionError:
            print(f"Test failed in iteration {iteration_count}")
            print(f"Sent:     {data}")
            print(f"Received: {response}")
            error_count += 1

        # Print progress update
        if time.time() >= next_update_time:
            elapsed_time = time.time() - start_time
            remaining_time = duration_seconds - elapsed_time
            print(
                f"Elapsed: {elapsed_time:.0f}s, Remaining: {remaining_time:.0f}s, Iterations: {iteration_count}, Errors: {error_count}"
            )
            next_update_time = time.time() + update_interval_seconds

    # Test finished. Print summary.
    elapsed_time = time.time() - start_time
    print(f"\nSPI test finished after {elapsed_time:.0f} seconds.")
    print(f"Total iterations: {iteration_count}")
    print(f"Total errors: {error_count}")
    spi.deinit()  # Clean up the SPI bus.
