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

    def test_soft_interrupt(self):
        """
        Tests breaking a loop using 'uselect' and a custom character ('q').
        """
        logger.info("Running Soft Interrupt (uselect) Test...")
        
        def remote_poll_loop():
            import sys, uselect, time
            
            # 1. Register Standard Input (Serial) for polling
            spoll = uselect.poll()
            spoll.register(sys.stdin, uselect.POLLIN)
            
            counter = 0
            while True:
                counter += 1
                
                # 2. Check for data immediately (timeout=0)
                if spoll.poll(0):
                   char = sys.stdin.read(1)
                   if char == 'q':
                       print("Quit received!") # <--- We search for this string
                       break
                
                time.sleep(0.1)
            
            return counter

        # 1. Start the loop asynchronously
        self.esp.exec_func(remote_poll_loop, wait=False)
        
        # 2. Let it run briefly
        time.sleep(1.0)
        
        # 3. Send 'q' to break the loop softly
        logger.info("Sending 'q' command to break loop...")
        self.esp.serial.write(b'q')
        
        # 4. VERIFY EXIT (The Critical Step)
        # Give ESP32 time to process 'q' and print the message
        time.sleep(0.5)
        
        # Read whatever is in the serial buffer
        if self.esp.serial.in_waiting:
            output = self.esp.serial.read(self.esp.serial.in_waiting).decode('utf-8', errors='replace')
        else:
            output = ""
            
        # If your 'if spoll.poll(0):' code is commented out, this will FAIL 
        # because the loop never prints "Quit received!"
        self.assertIn("Quit received!", output, "Loop did not exit via soft interrupt!")

        # 5. Now it is safe to confirm the board is free
        res = self.esp.exec_wait("print('I am free')")
        self.assertEqual(res, "I am free")
        logger.info("Soft interrupt successful.")

if __name__ == "__main__":
    unittest.main()