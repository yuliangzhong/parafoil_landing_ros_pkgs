import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3Stamped

import spidev
import time

BUS = 1
DEVICE = 2

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

class Magnetometer(Node):

    def __init__(self):
        super().__init__('magnetometer')
        self.publisher_ = self.create_publisher(Vector3Stamped, 'magnetometer', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.spi = spidev.SpiDev()
        self.spi.open(BUS, DEVICE)

        # SPI config
        self.spi.max_speed_hz = 5000
        self.spi.mode = 0

        # Sensor soft reset
        self.write_to_reg(CTRL1, 0b10000000)
        time.sleep(0.015) # wait 15ms for reset

    def timer_callback(self):
        # Turn on the magnetic measurement
        self.write_to_reg(CTRL0, 0b00000001)

        x_raw0 = self.read_from_reg(X_OUT0)
        x_raw1 = self.read_from_reg(X_OUT1)
        
        y_raw0 = self.read_from_reg(Y_OUT0)
        y_raw1 = self.read_from_reg(Y_OUT1)

        z_raw0 = self.read_from_reg(Z_OUT0)
        z_raw1 = self.read_from_reg(Z_OUT1)

        xyz_raw2 = self.read_from_reg(XYZ_OUT2)

        x_raw = (x_raw0 << 10) + (x_raw1 << 2) + ((xyz_raw2 & 0b11000000) >> 6)
        y_raw = (y_raw0 << 10) + (y_raw1 << 2) + ((xyz_raw2 & 0b00110000) >> 4)
        z_raw = (z_raw0 << 10) + (z_raw1 << 2) + ((xyz_raw2 & 0b00001100) >> 2)

        x_mag = (x_raw - NORMALIZE_CONST) / NORMALIZE_CONST * SENSITIVITY * GAUSS_TO_microTESLA
        y_mag = (y_raw - NORMALIZE_CONST) / NORMALIZE_CONST * SENSITIVITY * GAUSS_TO_microTESLA
        z_mag = (z_raw - NORMALIZE_CONST) / NORMALIZE_CONST * SENSITIVITY * GAUSS_TO_microTESLA

        msg = Vector3Stamped()
        msg.vector.x = x_mag
        msg.vector.y = y_mag
        msg.vector.z = z_mag
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info("x_mag: %.3f [uT]; y_mag %.3f [uT]; z_mag %.3f [uT]" %(x_mag, y_mag, z_mag))
    
    # To read, set the first bit of the address to 1
    def read_from_reg(self, reg_addr):
        data = self.spi.xfer([reg_addr | 0b10000000, 0x00])
        return data[1]
        
    # To write, set the first bit of the address to 0 (default)
    def write_to_reg(self, reg_addr, info):
        self.spi.xfer([reg_addr, info])
        time.sleep(0.002)   # you should wait


def main(args=None):
    rclpy.init(args=args)

    magnetometer = Magnetometer()

    rclpy.spin(magnetometer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    magnetometer.spi.close()
    magnetometer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()