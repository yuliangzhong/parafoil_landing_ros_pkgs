from smbus2 import SMBus, i2c_msg
import time

#########################
# SensorName: ADS1219IPWR
# Interface: I2C
#########################

# $ sudo i2cdetect -y 1
SENSOR_ADDR = 0x45

# Command byte definition
RESET = 0b00000110
START_SYNC = 0b00001000
RDATA = 0b00010000
WREG = 0b01000000

# Register configuration
# | MUX(3) | GAIN(1) | DR(2) | CM(1) | VREF(1) |
CONFIG = 0b10000010
V_REF = 2.048 # [V]

if __name__ == '__main__':    

    bus = SMBus(1)

    # reset sensor
    bus.write_byte(SENSOR_ADDR, RESET)
    time.sleep(0.001)

    # start/restart conversions
    bus.write_byte(SENSOR_ADDR, START_SYNC)
    time.sleep(0.001)


    # write register configuration
    bus.write_byte_data(SENSOR_ADDR, WREG, CONFIG)
    time.sleep(0.001)

    # start/restart conversions
    bus.write_byte(SENSOR_ADDR, START_SYNC)
    time.sleep(0.1)

    try:
        while(True):
            write = i2c_msg.write(SENSOR_ADDR, [RDATA])
            read = i2c_msg.read(SENSOR_ADDR, 3)
            bus.i2c_rdwr(write, read)
            volt_data = list(read)

            volt_count = (volt_data[0] << 16) + (volt_data[1] <<8 ) + volt_data[2]
            volt = volt_count * V_REF / (2**23)

            R1 = 2*499  # Resistance1 --X--X--, X = 499 [Ohm]
            R2 = 71.5   # Resistance2 --X--   , X = 71.5 [Ohm]
            Vs = volt / R2 * (R1+R2)
            print("Current battary voltage: %.2f [V]" %Vs)

            time.sleep(0.5)

    except KeyboardInterrupt:
        pass
