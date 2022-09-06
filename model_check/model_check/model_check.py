import os
import yaml
import numpy as np
from math import cos, sin, pi
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Vector3Stamped
import rclpy
from rclpy.node import Node

# constants
R = 6371000 # m, radius of the earth

# dirname = os.path.dirname(__file__)
# filename = os.path.join(dirname, '../config/config.yaml')
# print(filename)

class ModelCheck(Node):

    def __init__(self):
        super().__init__('model_check')
        self.gps_pub = self.create_publisher(NavSatFix, 'model_check', 1)
        self.pos_pub = self.create_publisher(Vector3Stamped, 'model_pos', 1)
        self.cmd_sub = self.create_subscription(Vector3Stamped, '/rockpara_actuators_node/auto_commands', self.cmd_callback, 1)
        # params = {}
        # with open("config.yaml", "r") as stream:
        #     try:
        #         params = yaml.safe_load(stream)
        #     except yaml.YAMLError as exc:
        #         print(exc)
        # load system parameters
        # a = params['a']
        # k = params['k']
        # self.Ts = params['Ts']

        # # load init conditions
        # self.pos = np.array([params['x0'], params['y0']]).reshape(-1,1)
        # self.psi_s = np.array([params['psi0'], params['psi_dot0']]).reshape(-1,1)
        # self.Vh = params['Vh']
        a = 10
        k = -5
        self.Ts = 0.2
        self.pos = np.array([5.11, -20.54]).reshape(-1,1)
        self.psi_s = np.array([-0.575, 0.169]).reshape(-1,1)
        self.Vh = 5.35
        self.origin_lat = 47.35504 # degree
        self.origin_lon = 8.51982 # degree

        # forward Euler
        Ac = np.array([[0, 1], [0, 1/a]])
        Bc = np.array([0, k/a]).reshape(-1,1)
        self.Ad = np.eye(2) + self.Ts*Ac
        self.Bd = self.Ts*Bc

    
    def cmd_callback(self, msg):
        dl = msg.vector.x
        dr = msg.vector.y
        da = dr - dl
        tmp_psi_s = self.psi_s
        self.psi_s = np.dot(self.Ad, self.psi_s) + self.Bd*da
        avg_psi = (tmp_psi_s[0][0] + self.psi_s[0][0])/2
        self.pos += self.Ts*self.Vh*np.array([cos(avg_psi), sin(avg_psi)]).reshape(-1,1)
        print(self.pos.reshape(-1))

        cvt_msg = NavSatFix()
        cvt_msg.header = msg.header
        cvt_msg.latitude = float(self.pos[0][0] / R / pi * 180 + self.origin_lat)
        cvt_msg.longitude = float(self.pos[1][0] / (R * cos(self.origin_lat / 180 * pi)) / pi * 180 + self.origin_lon)
        self.gps_pub.publish(cvt_msg)

        pos_msg = Vector3Stamped()
        pos_msg.header = msg.header
        pos_msg.vector.x = self.pos[0][0]
        pos_msg.vector.y = self.pos[1][0]
        self.pos_pub.publish(pos_msg)


def main(args=None):
    rclpy.init(args=args)

    model_check = ModelCheck()

    rclpy.spin(model_check)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    model_check.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
