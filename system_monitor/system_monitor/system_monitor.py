import rclpy
from rclpy.node import Node

from interfaces.msg import DoubleStamped

from smbus2 import SMBus, i2c_msg
import time

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
GAIN = 1
V_REF = 2.048 # [V]

R1 = 2*499  # Resistance1 --X--X--, X = 499 [Ohm]
R2 = 71.5   # Resistance2 --X--   , X = 71.5 [Ohm]

class SystemMonitor(Node):

    def __init__(self):
        super().__init__('system_monitor')
        self.publisher_ = self.create_publisher(DoubleStamped, 'battery_voltage', 10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.bus = SMBus(1)

        # reset sensor
        self.bus.write_byte(SENSOR_ADDR, RESET)
        time.sleep(0.001)

        # start/restart conversions
        self.bus.write_byte(SENSOR_ADDR, START_SYNC)
        time.sleep(0.001)


        # write register configuration
        self.bus.write_byte_data(SENSOR_ADDR, WREG, CONFIG)
        time.sleep(0.001)

        # start/restart conversions
        self.bus.write_byte(SENSOR_ADDR, START_SYNC)
        time.sleep(0.1)

    def timer_callback(self):
        write = i2c_msg.write(SENSOR_ADDR, [RDATA])
        read = i2c_msg.read(SENSOR_ADDR, 3)
        self.bus.i2c_rdwr(write, read)
        volt_data = list(read)

        volt_count = (volt_data[0] << 16) + (volt_data[1] <<8 ) + volt_data[2]
        volt = volt_count * V_REF / GAIN / (2**23)

        Vs = volt / R2 * (R1+R2)
        self.get_logger().info("Current battary voltage: %.2f [V]" %Vs)

        msg = DoubleStamped()
        msg.data = Vs
        msg.header.stamp = self.get_clock().now().to_msg()        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    system_monitor = SystemMonitor()

    rclpy.spin(system_monitor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    system_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()