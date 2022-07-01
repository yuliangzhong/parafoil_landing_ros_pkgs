# source code from 
# http://www.pibits.net/code/raspberry-pi-and-ms5611-barometric-pressure-sensor-example.php
# datasheet
# MS5611-01BA03

import smbus
import time
import matplotlib.pyplot as plt
import numpy as np

SENSOR_ADDR = 0x77

def GetData(bus, OSR):

    # Read 12 bytes of calibration data
    # Read pressure sensitivity
    data = bus.read_i2c_block_data(SENSOR_ADDR, 0xA2, 2)
    C1 = data[0] * 256 + data[1]
    # Read pressure offset
    data = bus.read_i2c_block_data(SENSOR_ADDR, 0xA4, 2)
    C2 = data[0] * 256 + data[1]
    # Read temperature coefficient of pressure sensitivity
    data = bus.read_i2c_block_data(SENSOR_ADDR, 0xA6, 2)
    C3 = data[0] * 256 + data[1]
    # Read temperature coefficient of pressure offset
    data = bus.read_i2c_block_data(SENSOR_ADDR, 0xA8, 2)
    C4 = data[0] * 256 + data[1]
    # Read reference temperature
    data = bus.read_i2c_block_data(SENSOR_ADDR, 0xAA, 2)
    C5 = data[0] * 256 + data[1]
    # Read temperature coefficient of the temperature
    data = bus.read_i2c_block_data(SENSOR_ADDR, 0xAC, 2)
    C6 = data[0] * 256 + data[1]

    # print("coefficients C1~C6: ", C1, C2, C3, C4, C5, C6)

    D1_addr = 0x40
    D2_addr = 0x50
    D_ts = 0.0025 # [s]

    if OSR == 4096:
        D_ts = 0.040
        D1_addr = 0x48
        D2_addr = 0x58
    elif OSR == 2048:
        D_ts = 0.020
        D1_addr = 0x46
        D2_addr = 0x56
    elif OSR == 1024:
        D_ts = 0.010
        D1_addr = 0x44
        D2_addr = 0x54
    elif OSR == 512:
        D_ts = 0.005
        D1_addr = 0x42
        D2_addr = 0x52
    else:
        pass # by default: OSR = 256

    # MS5611_01BXXX address, 0x77(119)
    # 0x40(64) Pressure conversion(OSR = 256) command
    bus.write_byte(SENSOR_ADDR, D1_addr)
    time.sleep(D_ts)
    # Read digital pressure value
    # Read data back from 0x00(0), 3 bytes
    # D1 MSB2, D1 MSB1, D1 LSB
    value = bus.read_i2c_block_data(SENSOR_ADDR, 0x00, 3)
    D1 = value[0] * 65536 + value[1] * 256 + value[2]

    # MS5611_01BXXX address, 0x77(119)
    # 0x50(80) Temperature conversion(OSR = 256) command
    bus.write_byte(SENSOR_ADDR, D2_addr)
    time.sleep(D_ts)
    # Read digital temperature value
    # Read data back from 0x00(0), 3 bytes
    # D2 MSB2, D2 MSB1, D2 LSB
    value = bus.read_i2c_block_data(SENSOR_ADDR, 0x00, 3)
    D2 = value[0] * 65536 + value[1] * 256 + value[2]

    # print("coefficients D1, D2: ", D1, D2)

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

    return pressure, cTemp

if __name__ == '__main__':

    # Get I2C bus
    bus = smbus.SMBus(1)

    # # MS5611_01BXXX address, SENSOR_ADDR(119)
    # # 0x1E(30) Reset command
    bus.write_byte(SENSOR_ADDR, 0x1E)
    time.sleep(0.2)

    len = 10
    warm_up_pressure, _ = GetData(bus, 256)
    pressure_buffer = warm_up_pressure * np.ones(len)

    count = 0

    while(True):
        count += 1
        # Get data
        pressure, _ = GetData(bus, 1024)
        # Update buffer
        pressure_buffer[:-1] = pressure_buffer[1:]
        pressure_buffer[-1] = pressure
        # get measurement
        filtered_pressure = np.mean(pressure_buffer)
        print("Pressure : %.4f mbar" %filtered_pressure)
        # plot
        plt.plot([count-1, count], pressure_buffer[-2:], color='red')
        plt.pause(0.001)
        time.sleep(0.1)        