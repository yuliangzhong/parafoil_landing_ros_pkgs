# source code from 
# http://www.pibits.net/code/raspberry-pi-and-ms5611-barometric-pressure-sensor-example.php

import smbus
import time

# Get I2C bus
bus = smbus.SMBus(1)

# # MS5611_01BXXX address, 0x77(119)
# # 0x1E(30) Reset command
bus.write_byte(0x77, 0x1E)
time.sleep(0.5)
# Read 12 bytes of calibration data
# Read pressure sensitivity
data = bus.read_i2c_block_data(0x77, 0xA2, 2)
C1 = data[0] * 256 + data[1]
# Read pressure offset
data = bus.read_i2c_block_data(0x77, 0xA4, 2)
C2 = data[0] * 256 + data[1]
# Read temperature coefficient of pressure sensitivity
data = bus.read_i2c_block_data(0x77, 0xA6, 2)
C3 = data[0] * 256 + data[1]
# Read temperature coefficient of pressure offset
data = bus.read_i2c_block_data(0x77, 0xA8, 2)
C4 = data[0] * 256 + data[1]
# Read reference temperature
data = bus.read_i2c_block_data(0x77, 0xAA, 2)
C5 = data[0] * 256 + data[1]
# Read temperature coefficient of the temperature
data = bus.read_i2c_block_data(0x77, 0xAC, 2)
C6 = data[0] * 256 + data[1]

print("coefficients C1~C6: ", C1, C2, C3, C4, C5, C6)

# MS5611_01BXXX address, 0x77(119)
# 0x40(64) Pressure conversion(OSR = 256) command
bus.write_byte(0x77, 0x40)
time.sleep(0.5)
# Read digital pressure value
# Read data back from 0x00(0), 3 bytes
# D1 MSB2, D1 MSB1, D1 LSB
value = bus.read_i2c_block_data(0x77, 0x00, 3)
D1 = value[0] * 65536 + value[1] * 256 + value[2]

# MS5611_01BXXX address, 0x76(118)
# 0x50(80) Temperature conversion(OSR = 256) command
bus.write_byte(0x77, 0x50)
time.sleep(0.5)
# Read digital temperature value
# Read data back from 0x00(0), 3 bytes
# D2 MSB2, D2 MSB1, D2 LSB
value = bus.read_i2c_block_data(0x77, 0x00, 3)
D2 = value[0] * 65536 + value[1] * 256 + value[2]

print("coefficients D1, D2: ", D1, D2)

dT = D2 - C5 * 256
TEMP = 2000 + dT * C6 / 8388608
OFF = C2 * 65536 + (C4 * dT) / 128
SENS = C1 * 32768 + (C3 * dT ) / 256
T2 = 0
OFF2 = 0
SENS2 = 0
if TEMP >= 2000 :
    T2 = 0
    OFF2 = 0
    SENS2 = 0
elif TEMP < 2000 :
    T2 = (dT * dT) / 2147483648
    OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2
    SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4
    if TEMP < -1500 :
        OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500))
        SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2
TEMP = TEMP - T2
OFF = OFF - OFF2
SENS = SENS - SENS2
pressure = ((((D1 * SENS) / 2097152) - OFF) / 32768.0) / 100.0

cTemp = TEMP / 100.0
fTemp = cTemp * 1.8 + 32
# Output data to screen
print("Pressure : %.2f mbar" %pressure)
print("Temperature in Celsius : %.2f C" %cTemp)
print("Temperature in Fahrenheit : %.2f F" %fTemp)