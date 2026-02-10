import logging
import unittest
import time

from hexastorm.esp32_controller import ESP32Controller

logger = logging.getLogger(__name__)

class Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.esp = ESP32Controller(port='/dev/ttyACM0')
    
    def test_invalid_command(self):
        """
        Tests that an invalid command raises a RuntimeError.
        """
        logger.info("Running Invalid Command Test...")
        
        def remote_fail():
            import host  # noqa: F401 (silence linter warning if host isn't local)
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

        def remote_calc(a, b):
            print(a + b)

        result = self.esp.exec_func(remote_calc, a=10, b=5)
        
        self.assertEqual(result, "15")
        logger.info(f"Valid command output verified: {result}")

    def test_blink(self):
        logger.info("Running Blink Test...")
        
        def remote_blink(dur):
            import machine, time
            led = machine.Pin(8, machine.Pin.OUT)
            led.off()
            time.sleep(dur) 
            led.on()

        self.esp.exec_func(remote_blink, wait=False, dur=2)
        
        # Wait for duration + buffer
        time.sleep(2.5)
        
        # Verify we can talk again
        res = self.esp.exec_wait("print('done')")
        self.assertEqual(res, "done")

    def test_interrupt_state_retention(self):
        """
        Tests that variables retain their value after a Ctrl+C interrupt.
        """
        logger.info("Running Interrupt State Retention Test...")

        self.esp.exec_wait("x = 0")

        # This still relies on global x because x is state *outside* the function.
        # However, we must explicitly declare global x because we are now 
        # inside a real function scope on the ESP32.
        def remote_loop():
            import time
            global x  # Necessary now because we are inside a 'def' scope on ESP32
            x += 1          
            while True:      
                time.sleep(0.1)

        self.esp.exec_func(remote_loop, wait=False)

        time.sleep(0.5)
        logger.info("Sending Ctrl+C...")
        self.esp.stop()

        val = self.esp.exec_wait("print(x)")
        logger.info(f"Value of x after interrupt: {val}")
        self.assertEqual(val, "1", "Global variable x should be 1 after interrupt")

if __name__ == "__main__":
    unittest.main()