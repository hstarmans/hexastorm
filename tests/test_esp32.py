import logging
import unittest
import time

from hexastorm.esp32_controller import ESP32Controller

logger = logging.getLogger(__name__)


class Tests(unittest.TestCase):
    # Test that we can send commands, get output, and handle interrupts properly via ESP32Controller.
    @classmethod
    def setUpClass(cls):
        cls.esp = ESP32Controller(port='/dev/ttyACM0')
    
    def test_invalid_command(self):
        """
        Tests that an invalid command raises a RuntimeError.
        """
        logger.info("Running Invalid Command Test...")
        
        # Define the invalid code locally
        def remote_fail():
            import host # noqa: F401 (silence linter warning if host isn't local)
            host.superfunctie()

        with self.assertRaises(RuntimeError) as cm:
            self.esp.exec_func(remote_fail)
        
        logger.info(f"Caught expected error: {cm.exception}")
        time.sleep(1) 

    def test_valid_command(self):
        """
        Tests that a valid command runs without error and returns output.
        """
        logger.info("Running Valid Command Test...")

        # We pass operands via kwargs to show injection
        def remote_calc():
            print(a + b)

        result = self.esp.exec_func(remote_calc, a=10, b=5)
        
        self.assertEqual(result, "15")
        logger.info(f"Valid command output verified: {result}")

    def test_blink(self):
        logger.info("Running Blink Test...")
        
        # The logic is now standard Python code, fully lintable
        def remote_blink():
            import machine, time
            led = machine.Pin(8, machine.Pin.OUT)
            led.off()
            time.sleep(dur) # 'dur' is injected via kwargs
            led.on()

        # Fire and forget
        self.esp.exec_func(remote_blink, wait=False, dur=2)
        
        # Wait for python to catch up + buffer
        time.sleep(2.5)
        
        # Verify we can talk again
        res = self.esp.exec_wait("print('done')")
        self.assertEqual(res, "done")

    def test_interrupt_state_retention(self):
        """
        Tests that variables retain their value after a Ctrl+C interrupt.
        """
        logger.info("Running Interrupt State Retention Test...")

        # 1. Initialize global variable x
        self.esp.exec_wait("x = 0")

        # 2. Define the infinite loop function locally
        # Note: We don't define a function *on* the ESP32, we just send
        # a script that runs an infinite loop.
        def remote_loop():
            import time
            global x
            x += 1           # Step 1: Increase
            while True:      # Step 2: Sleep forever
                time.sleep(0.1)

        # 3. Start execution in background (wait=False)
        logger.info("Starting infinite loop function...")
        self.esp.exec_func(remote_loop, wait=False)

        # 4. Wait briefly for x += 1 to happen
        time.sleep(0.5)

        # 5. Interrupt!
        logger.info("Sending Ctrl+C to stop execution...")
        self.esp.stop()

        # 6. Verify State
        val = self.esp.exec_wait("print(x)")
        logger.info(f"Value of x after interrupt: {val}")
        self.assertEqual(val, "1", "Global variable x should be 1 after interrupt")

if __name__ == "__main__":
    unittest.main()