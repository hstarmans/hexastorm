from smbus2 import SMBus
# connect to I2C bus on device 1
device_nr = 1
address = 0x28

bus = SMBus(device_nr)

val = bus.read_byte_data(address,0)
print(f"Current value in memory {val}")
set_val = 70
bus.write_byte_data(address,0, set_val)

assert set_val == bus.read_byte_data(address,0)
