from smbus2 import SMBus, i2c_msg
import time

RESET = 0b00000110
START_SYNC = 0b00001000
WREG = 0b01000000
RDATA = 0b00010000
CONFIG = 0b10000010

SENSOR_ADDR = 0x45

bus = SMBus(1, True)

# reset sensor
bus.write_byte(SENSOR_ADDR, RESET)
time.sleep(0.0001)


# start/restart conversions
bus.write_byte(SENSOR_ADDR, START_SYNC)
time.sleep(0.0001)


# write configuration register
# bus.write_byte(SENSOR_ADDR, 0b01000000)
# time.sleep(0.0001)
bus.write_byte_data(SENSOR_ADDR, WREG , CONFIG)
time.sleep(0.0001)

# start/restart conversions
bus.write_byte(SENSOR_ADDR, START_SYNC)
time.sleep(0.1)


# write = i2c_msg.write(SENSOR_ADDR, [CONFIG, RDATA])
write = i2c_msg.write(SENSOR_ADDR, [RDATA])
read = i2c_msg.read(SENSOR_ADDR, 3)
bus.i2c_rdwr(write, read)
data = list(read)

print(data)

num = (data[0]<<16)+(data[1]<<8)+data[2]
print(num)
print(type(num))

volt = num*2.048/(2**23)
print(volt)

R1 = 2*499
R2 = 71.5
Vs = volt*(R1+R2)/R2
print("Battary voltage: ", Vs)


# data = bus.read_i2c_block_data(SENSOR_ADDR,RDATA, 3)
# # bus.write_byte(SENSOR_ADDR, RDATA)
# # time.sleep(0.0001)
# # data= bus.read_block_data(SENSOR_ADDR, RDATA)

# [0,0,0]

# print(data)

# if __name__ == '__main__':