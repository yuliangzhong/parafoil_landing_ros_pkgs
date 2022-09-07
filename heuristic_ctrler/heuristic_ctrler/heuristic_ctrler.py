import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped
from math import atan2, sin, cos, pi


##### constants #####
R = 6371000 # m, radius of the earth
origin_lat = 47.35504 # degree
origin_lon = 8.51982 # degree



class HeuristicCtrler(Node):

    def __init__(self):
        super().__init__('heuristic_ctrler')

        ##### load parameters #####
        self.declare_parameters(
            namespace='',
            parameters=[
                ('Ts', 0.5),
                ('N', 4),
                ('um', 0.5),
                ('control_mode', 1),
                ('heading', 0.0),
                ('landing_lat', 47.35504),
                ('landing_lon', 8.51982),
            ]
        )
        Ts = self.get_parameter('Ts').get_parameter_value().double_value
        self.N = self.get_parameter('N').get_parameter_value().integer_value
        self.um = self.get_parameter('um').get_parameter_value().double_value
        self.control_mode = self.get_parameter('control_mode').get_parameter_value().integer_value
        self.heading = self.get_parameter('heading').get_parameter_value().double_value
        landing_lat = self.get_parameter('landing_lat').get_parameter_value().double_value
        landing_lon = self.get_parameter('landing_lon').get_parameter_value().double_value

        self.land_y = (landing_lon - origin_lon) / 180 * pi * R * cos(origin_lat / 180 * pi) # east = y+
        self.land_x = (landing_lat - origin_lat) / 180 * pi * R # north = x+

        self.cmd_pub = self.create_publisher(Vector3Stamped, '/rockpara_actuators_node/auto_commands', 1)
        self.psi_pub = self.create_publisher(Float32, '/estimated_heading', 1)
        self.timer = self.create_timer(Ts, self.cmd_callback)
        self.cnt = 0

        self.pos_sub = self.create_subscription(Vector3Stamped, '/position', self.pos_callback, 1)
        self.pos_now = None
        self.pos_last = None
        self.yaw_now = 0.0


    def pos_callback(self, msg):
        if self.pos_now == None:
            self.pos_now = msg
        else:
            self.pos_last = self.pos_now
            self.pos_now = msg
        
        if self.pos_last != None and self.pos_now != None:
            dx = self.pos_now.vector.x - self.pos_last.vector.x
            dy = self.pos_now.vector.y - self.pos_last.vector.y
            if abs(dx) >= 1e-3 or abs(dy) >= 1e-3:
                self.yaw_now = atan2(dy, dx)
                psi_msg = Float32()
                psi_msg.data = self.yaw_now / pi * 180
                self.psi_pub.publish(psi_msg)


    def cmd_callback(self):

        if self.cnt % self.N == 0:
            if self.control_mode == -1:
                u = -0.6*self.um
            elif self.control_mode == 0:
                u = 0.6*self.um
            elif self.control_mode == 1:
                err = atan2(sin(self.yaw_now - self.heading), cos(self.yaw_now - self.heading))
                u = self.cal_diff_brake(err, self.um)
            elif self.control_mode == 2:
                yaw_d = atan2(self.land_y - self.pos_now.vector.y, self.land_x - self.pos_now.vector.x)
                err = atan2(sin(self.yaw_now - yaw_d), cos(self.yaw_now - yaw_d))
                u = self.cal_diff_brake(err, self.um)
            else:
                print("Warning, the controller receives an undefined mode!")
                u = 0

        else:
            u = 0
        
        cmd_msg = Vector3Stamped()
        cmd_msg.vector.z = 0.0
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        if u >= 0:
            cmd_msg.vector.x = 0.5
            cmd_msg.vector.y = max(0.5 - u, 0)
        else:
            cmd_msg.vector.x = max(0.5 + u, 0)
            cmd_msg.vector.y = 0.5
        
        self.cmd_pub.publish(cmd_msg)
        self.cnt += 1
    
    def cal_diff_brake(self, err, um):
        if pi*3/4 < err <= pi:
            u = 2*um/pi*(err - pi) + um
        elif pi/2 < err <= pi*3/4:
            u = um/pi*(err - pi/2) + um/4
        elif -pi/2 < err <= pi/2:
            u = um/pi/2*err
        elif -pi*3/4 < err <= -pi/2:
            u = um/pi*(err + pi/2) - um/4
        else:
            u = 2*um/pi*(err + pi) - um
        return u



def main(args=None):
    rclpy.init(args=args)
    heuristic_ctrler = HeuristicCtrler()
    rclpy.spin(heuristic_ctrler)
    heuristic_ctrler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# ubuntu@ubuntu-desktop:~/Github/ros2_ws$ ros2 run heuristic_ctrler heuristic_ctrler --ros-args --params-file ~/Github/ros2_ws/src/heuristic_ctrler/config/params.yaml
