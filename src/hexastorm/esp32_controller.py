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
        Extracts the body of a local Python function and runs it on the ESP32.
        
        :param func: The local function object.
        :param wait: If True, uses exec_wait; else exec_no_wait.
        :param kwargs: Variables to inject into the global scope before running.
        """
        # 1. Get source and dedent
        source = inspect.getsource(func)
        source = textwrap.dedent(source)
        
        # 2. Extract body (naive approach: everything after first line)
        # This assumes the function definition "def name(...):" is on one line.
        lines = source.splitlines()
        body = "\n".join(lines[1:])
        body = textwrap.dedent(body)
        
        # 3. Create header for variable injection
        header = ""
        for key, value in kwargs.items():
            header += f"{key} = {value!r}\n"
            
        full_command = header + body
        
        if wait:
            return self.exec_wait(full_command)
        else:
            return self.exec_no_wait(full_command)
