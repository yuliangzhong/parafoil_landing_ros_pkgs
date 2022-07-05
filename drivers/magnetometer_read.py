import spidev
import time

#########################
# SensorName: MMC5983MA
# Interface: SPI
#########################

# $ ls /dev/spi*
# $ /dev/spidev<bus>.<device>
BUS = 1
DEVICE = 2

# | 0/1 _ | Register Map (6) | Data (8 bits) |

# Register Map Definition (6 bits)
X_OUT0 = 0x00
X_OUT1 = 0x01

Y_OUT0 = 0x02
Y_OUT1 = 0x03

Z_OUT0 = 0x04
Z_OUT1 = 0x05

XYZ_OUT2 = 0x06

T_OUT = 0x07
STATUS = 0x08
CTRL0 = 0x09
CTRL1 = 0x0A
CTRL2 = 0x0B
CTRL3 = 0x0C

PRODUCT_ID = 0x2F

NORMALIZE_CONST = 131072
SENSITIVITY = 8
GAUSS_TO_microTESLA = 100

# To read, set the first bit of the address to 1
def read_from_reg(spi, reg_addr):
    data = spi.xfer([reg_addr | 0b10000000, 0x00])
    return data[1]
    
# To write, set the first bit of the address to 0 (default)
def write_to_reg(spi, reg_addr, info):
    spi.xfer([reg_addr, info])
    time.sleep(0.002)   # you should wait

if __name__ == '__main__':    

    spi = spidev.SpiDev()
    spi.open(BUS, DEVICE)

    # SPI config
    spi.max_speed_hz = 5000
    spi.mode = 0

    # Sensor soft reset
    write_to_reg(spi, CTRL1, 0b10000000)
    time.sleep(0.015) # wait 15ms for reset

    # Turn on the temperature measurement
    write_to_reg(spi, CTRL0, 0b00000010)
    
    # Primary test
    device_id = read_from_reg(spi, PRODUCT_ID)
    print("Device ID: 0x%x" %device_id)
    temperature_raw = read_from_reg(spi, T_OUT)
    temperature = temperature_raw / 256.0 * (125.0 - (-75.0)) + (-75.0)
    print("Current temperature is: %.1f *C" %temperature)

    # Start reading: 18bit reading
    try:
        while(True):
            # Turn on the magnetic measurement
            write_to_reg(spi, CTRL0, 0b00000001)

            x_raw0 = read_from_reg(spi, X_OUT0)
            x_raw1 = read_from_reg(spi, X_OUT1)
            
            y_raw0 = read_from_reg(spi, Y_OUT0)
            y_raw1 = read_from_reg(spi, Y_OUT1)

            z_raw0 = read_from_reg(spi, Z_OUT0)
            z_raw1 = read_from_reg(spi, Z_OUT1)

            xyz_raw2 = read_from_reg(spi, XYZ_OUT2)

            x_raw = (x_raw0 << 10) + (x_raw1 << 2) + ((xyz_raw2 & 0b11000000) >> 6)
            y_raw = (y_raw0 << 10) + (y_raw1 << 2) + ((xyz_raw2 & 0b00110000) >> 4)
            z_raw = (z_raw0 << 10) + (z_raw1 << 2) + ((xyz_raw2 & 0b00001100) >> 2)

            x_mag = (x_raw - NORMALIZE_CONST) / NORMALIZE_CONST * SENSITIVITY * GAUSS_TO_microTESLA
            y_mag = (y_raw - NORMALIZE_CONST) / NORMALIZE_CONST * SENSITIVITY * GAUSS_TO_microTESLA
            z_mag = (z_raw - NORMALIZE_CONST) / NORMALIZE_CONST * SENSITIVITY * GAUSS_TO_microTESLA

            print("x_mag: %.3f [uT]; y_mag %.3f [uT]; z_mag %.3f [uT]" %(x_mag, y_mag, z_mag))

            time.sleep(0.1)

    except KeyboardInterrupt:
        spi.close()
