import spidev
import time

spi = spidev.SpiDev()
spi.open(0, 0)

# SPI config
spi.max_speed_hz = 100000
spi.mode = 3

def read_from_reg(spi, reg_addr):
    data = spi.xfer([reg_addr | 0b10000000, 0x00])
    return data[1]
    
# To write, set the first bit of the address to 0 (default)
def write_to_reg(spi, reg_addr, info):
    spi.xfer([reg_addr, info])
    time.sleep(0.002)   # you should wait
    
    
out = read_from_reg(spi, 0x00)
time.sleep(0.015) # wait 15ms for reset
print(out)
