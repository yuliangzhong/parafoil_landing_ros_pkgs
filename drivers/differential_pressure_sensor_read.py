import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1)

data = bus.read_i2c_block_data(0x28, 0x00, 4) # 32 bytes, 27 + 5

print(data)
print(type(data[0]))

status = (data[0] & 0b11000000) >> 6
if (status == 0):
    print("Normal operation. Good data packet")
elif (status == 1):
    print("Reserved")
elif (status == 2):
    print("Stale data. Data has been fetched since last measurement cycle")
elif (status == 3):
    print("Fault detected")
else:
    print("Unknown status!! Status is: ", status)

pressure_raw = ((0b00111111 & data[0]) << 8) + data[1]
temp_raw = ((0b11100000 & data[3]) >> 5) + (data[2] << 3)
print(pressure_raw, temp_raw)

temperature = temp_raw * 200.0 / 2047 - 50.0
print(temperature)

P_MAX = 1
P_MIN = -P_MAX

pressure = (pressure_raw - 0.1*16383) / (0.8*16383/(P_MAX - P_MIN)) + P_MIN

print(pressure)
