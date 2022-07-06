import rclpy
from rclpy.node import Node

from interfaces.msg import DoubleStamped

import smbus
import time

# $ sudo i2cdetect -y 1
SENSOR_ADDR = 0x77

# Define constants
SEA_LEVEL_P = 1013.25 # [mbar/hPa]
GAS_CONST = 8.31432 # [Nm/mol/K]
GRAVITY_ACC = 9.80665 # [N/kg]
MOLAR_MASS_EARTH_AIR = 0.0289644 # [kg/mol]
STD_TEMP_LAPSE_RATE = 0.0065 # [K/m]

class AbsoluteAltimeter(Node):

    def __init__(self):
        super().__init__('absolute_altimeter')
        self.publisher_ = self.create_publisher(DoubleStamped, 'absolute_height', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.bus = smbus.SMBus(1)
        self.bus.write_byte(SENSOR_ADDR, 0x1E)
        time.sleep(0.1)

    def timer_callback(self):
        pressure, temperature = self.GetData()
        h = ((SEA_LEVEL_P / pressure)**(GAS_CONST*STD_TEMP_LAPSE_RATE/GRAVITY_ACC/MOLAR_MASS_EARTH_AIR) -1) * (temperature+273.15) / STD_TEMP_LAPSE_RATE
        self.get_logger().info("Absolute height: %.1f m, Pressure : %.4f mbar, Temperature: %.2f *C" %(h, pressure, temperature))

        msg = DoubleStamped()
        msg.data = h
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
    
    def GetData(self):
        data = self.bus.read_i2c_block_data(SENSOR_ADDR, 0xA2, 2)
        C1 = (data[0] << 8) + data[1]
        data = self.bus.read_i2c_block_data(SENSOR_ADDR, 0xA4, 2)
        C2 = (data[0] << 8) + data[1]
        data = self.bus.read_i2c_block_data(SENSOR_ADDR, 0xA6, 2)
        C3 = (data[0] << 8) + data[1]
        data = self.bus.read_i2c_block_data(SENSOR_ADDR, 0xA8, 2)
        C4 = (data[0] << 8) + data[1]
        data = self.bus.read_i2c_block_data(SENSOR_ADDR, 0xAA, 2)
        C5 = (data[0] << 8) + data[1]
        data = self.bus.read_i2c_block_data(SENSOR_ADDR, 0xAC, 2)
        C6 = (data[0] << 8) + data[1]

        D_ts = 0.010
        D1_addr = 0x44
        D2_addr = 0x54

        self.bus.write_byte(SENSOR_ADDR, D1_addr)
        time.sleep(D_ts)
        value = self.bus.read_i2c_block_data(SENSOR_ADDR, 0x00, 3)
        D1 = (value[0] << 16) + (value[1] << 8) + value[2]

        self.bus.write_byte(SENSOR_ADDR, D2_addr)
        time.sleep(D_ts)
        value = self.bus.read_i2c_block_data(SENSOR_ADDR, 0x00, 3)
        D2 = (value[0] << 16) + (value[1] << 8) + value[2]

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


def main(args=None):
    rclpy.init(args=args)

    absolute_altimeter = AbsoluteAltimeter()

    rclpy.spin(absolute_altimeter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    absolute_altimeter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()