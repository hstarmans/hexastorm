import serial
import time
import textwrap
import inspect

class ESP32Controller:
    def __init__(self, port='/dev/ttyACM0', baud=115200, timeout=2.0):
        self.serial = serial.Serial(port, baud)
        self._ensure_raw_repl()

    def close(self):
        if self.serial.is_open:
            self.serial.close()

    def stop(self):
        """
        Sends Ctrl-C to interrupt the running program.
        """
        self.serial.write(b'\r\x03') 
        time.sleep(0.1)

    def _ensure_raw_repl(self):
        self.serial.reset_input_buffer()
        for _ in range(3):
            self.serial.write(b'\r\x03') # Ctrl-C
            time.sleep(0.1)
            self.serial.write(b'\x01')   # Ctrl-A
            
            end_time = time.time() + 1.0
            while time.time() < end_time:
                if self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting)
                    if b'raw REPL' in data and data.strip().endswith(b'>'):
                        return
                time.sleep(0.05)
        raise RuntimeError("Could not connect to ESP32 REPL.")

    def exec_wait(self, command):
        """
        Executes command string and returns output.
        """
        self._ensure_raw_repl()
        
        cmd_bytes = textwrap.dedent(command).encode('utf-8')
        self.serial.write(cmd_bytes)
        self.serial.write(b'\x04') 

        ret = self.serial.read_until(b'OK')
        if b'OK' not in ret:
            if b'Traceback' in ret:
                raise RuntimeError(f"MicroPython Compile Error:\n{ret.decode(errors='replace')}")
            raise RuntimeError(f"ESP32 stuck or timed out. Got: {ret}")

        output = self.serial.read_until(b'\x04')
        if not output.endswith(b'\x04'):
             raise RuntimeError(f"Timeout waiting for output. partial: {output}")
        output = output[:-1]

        err = self.serial.read_until(b'\x04')
        if not err.endswith(b'\x04'):
             raise RuntimeError("Timeout waiting for error code.")
        err = err[:-1]

        if err.strip():
            raise RuntimeError(f"ESP32 Exception:\n{err.decode(errors='replace')}")

        return output.decode(errors='replace').strip()

    def exec_no_wait(self, command):
        """
        Fire and forget execution of command string.
        """
        self._ensure_raw_repl()
        cmd_bytes = textwrap.dedent(command).encode('utf-8')
        self.serial.write(cmd_bytes)
        self.serial.write(b'\x04')

        ret = self.serial.read_until(b'OK')
        if b'OK' not in ret:
             raise RuntimeError("ESP32 did not acknowledge start of command.")

    def exec_func(self, func, wait=True, **kwargs):
        """
        Sends the function definition to the ESP32 and calls it with provided kwargs.
        
        :param func: The local function object.
        :param wait: If True, uses exec_wait; else exec_no_wait.
        :param kwargs: Arguments to pass to the function call.
        """
        # 1. Get the full source code of the function (including 'def ...')
        source = inspect.getsource(func)
        source = textwrap.dedent(source)
        
        # 2. Get the function name so we can call it
        func_name = func.__name__
        
        # 3. Format the arguments (e.g., "a=10, b=5")
        args_str = ", ".join(f"{k}={v!r}" for k, v in kwargs.items())
        
        # 4. Construct the full payload: Define it, then call it.
        full_command = f"{source}\n{func_name}({args_str})"
        
        if wait:
            return self.exec_wait(full_command)
        else:
            return self.exec_no_wait(full_command)
