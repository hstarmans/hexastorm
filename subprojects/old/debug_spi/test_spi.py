import machine
import time
import random
from hexastorm.controller import Host 

# Configuration
TEST_DURATION_SECONDS = 15 * 60  # 15 minutes
PROGRESS_UPDATE_INTERVAL_SECONDS = 30
SPI_BUS = 2
SPI_BAUDRATE = int(3e6)
SPI_SCK_PIN = 12
SPI_MOSI_PIN = 13
SPI_MISO_PIN = 11
FPGA_SELECT_PIN = 9
DATA_LENGTH = 12  # Number of bytes to send.  Important: changed from len(bts) to a constant

def run_spi_test(duration_seconds=TEST_DURATION_SECONDS, update_interval_seconds=PROGRESS_UPDATE_INTERVAL_SECONDS):
    """
    Runs an SPI test for a specified duration, sending random data and checking the response.

    Args:
        duration_seconds: The duration of the test in seconds.
        update_interval_seconds: The interval for printing progress updates in seconds.
    """
    hst = Host(micropython=True) 
    hst.reset() 
    # Initialize SPI
    spi = machine.SPI(SPI_BUS,
                    baudrate=SPI_BAUDRATE,
                    polarity=0,
                    phase=1,
                    sck=machine.Pin(SPI_SCK_PIN),
                    mosi=machine.Pin(SPI_MOSI_PIN),
                    miso=machine.Pin(SPI_MISO_PIN))

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
            print(f"Elapsed: {elapsed_time:.0f}s, Remaining: {remaining_time:.0f}s, Iterations: {iteration_count}, Errors: {error_count}")
            next_update_time = time.time() + update_interval_seconds

    # Test finished. Print summary.
    elapsed_time = time.time() - start_time
    print(f"\nSPI test finished after {elapsed_time:.0f} seconds.")
    print(f"Total iterations: {iteration_count}")
    print(f"Total errors: {error_count}")
    spi.deinit() # Clean up the SPI bus.


