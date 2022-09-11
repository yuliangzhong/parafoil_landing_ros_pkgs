import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3Stamped
from math import atan2, sin, cos, pi


##### constants #####
R = 6371000 # m, radius of the earth
origin_lat = 47.35504 # degree
origin_lon = 8.51982 # degree
ds = 0.1 # [m]


class HeuristicCtrler(Node):

    def __init__(self):
        super().__init__('heuristic_ctrler')

        ##### load parameters #####
        self.declare_parameters(
            namespace='',
            parameters=[
                ('um', 0.5),
                ('control_mode', 0),
                ('Ts', 2.0),
                ('ratio', 0.5),
                ('heading', 0.0),
                ('Kp', 0.0),
                ('Kd', 0.0),
                ('max_heading_diff', 0.0),
                ('landing_lat', 47.35504),
                ('landing_lon', 8.51982),
            ]
        )
        self.um = self.get_parameter('um').get_parameter_value().double_value
        self.control_mode = self.get_parameter('control_mode').get_parameter_value().integer_value
        Ts = self.get_parameter('Ts').get_parameter_value().double_value
        ratio = self.get_parameter('ratio').get_parameter_value().double_value
        self.heading = self.get_parameter('heading').get_parameter_value().double_value
        self.Kp = self.get_parameter('Kp').get_parameter_value().double_value
        self.Kd = self.get_parameter('Kd').get_parameter_value().double_value
        self.max_heading_diff = self.get_parameter('max_heading_diff').get_parameter_value().double_value

        landing_lat = self.get_parameter('landing_lat').get_parameter_value().double_value
        landing_lon = self.get_parameter('landing_lon').get_parameter_value().double_value
        self.get_logger().info("um, control_mode, Ts, ratio, heading, Kp, Kd, max_hf, lat, lon")
        self.get_logger().info("%f, %d, %f, %f, %f, %f, %f, %f, %f, %f\n" %(self.um, self.control_mode, Ts, ratio, self.heading, self.Kp, self.Kd, self.max_heading_diff, landing_lat, landing_lon))

        self.land_y = (landing_lon - origin_lon) / 180 * pi * R * cos(origin_lat / 180 * pi) # east = y+
        self.land_x = (landing_lat - origin_lat) / 180 * pi * R # north = x+

        self.N = Ts // 0.01
        self.n = (Ts * ratio) // 0.01

        self.cmd_pub = self.create_publisher(Vector3Stamped, '/rockpara_actuators_node/auto_commands', 1)
        self.psi_pub = self.create_publisher(Float32, '/estimated_heading', 1)
        self.psi_dot_pub = self.create_publisher(Float32, '/estimated_psi_dot', 1)
        self.up_pub = self.create_publisher(Float32, '/p_element', 1)
        self.ud_pub = self.create_publisher(Float32, '/d_element', 1)

        self.status_pub = self.create_publisher(Bool, '/estimation_status', 1)

        self.timer = self.create_timer(0.01, self.cmd_callback)
        self.cnt = 0

        self.pos_sub = self.create_subscription(Vector3Stamped, '/position', self.pos_callback, 1)
        self.pos_now = None
        self.pos_last = None
        self.pos_dlast = None

        self.yaw_now = 0.0
        self.yaw_dot_now = 0.0
        self.is_good_status = False

        self.u_last = 0.0

    # updates each 0.4 seconds
    def pos_callback(self, msg):
        if self.pos_now == None:
            self.pos_now = msg
        elif self.pos_last == None:
            self.pos_last = self.pos_now
            self.pos_now = msg
        else:
            self.pos_dlast = self.pos_last
            self.pos_last = self.pos_now
            self.pos_now = msg
        
        if self.pos_dlast != None and self.pos_last != None and self.pos_now != None:
            dx_now = self.pos_now.vector.x - self.pos_last.vector.x
            dy_now = self.pos_now.vector.y - self.pos_last.vector.y

            dx_last = self.pos_last.vector.x - self.pos_dlast.vector.x
            dy_last = self.pos_last.vector.y - self.pos_dlast.vector.y

            if (abs(dx_now) >= ds or abs(dy_now) >= ds) and (abs(dx_last) >= ds or abs(dy_last) >= ds):
                self.yaw_now = atan2(dy_now, dx_now)
                yaw_last = atan2(dy_last, dx_last)
                dyaw = self.yaw_now - yaw_last
                dyaw = atan2(sin(dyaw), cos(dyaw)) # warp to [-pi, pi]
                self.yaw_dot_now = dyaw / 0.4 # rad/s

                # publish yaw_now
                psi_msg = Float32()
                psi_msg.data = self.yaw_now / pi * 180
                self.psi_pub.publish(psi_msg)

                # publish yaw_dot_now
                psi_dot_msg = Float32()
                psi_dot_msg.data = self.yaw_dot_now / pi * 180 # deg/s
                self.psi_dot_pub.publish(psi_dot_msg)
                
                self.is_good_status = True
            else:
                self.is_good_status = False

            # publish status
            status_msg = Bool()
            status_msg.data = self.is_good_status
            self.status_pub.publish(status_msg)

    # updates each 0.01 seconds
    def cmd_callback(self):

        if self.control_mode == 0:
            u = self.um

        elif self.control_mode == 1:
            if self.cnt < self.n:
                u = self.um
            else:
                u = 0
        
        elif self.control_mode == 2 or self.control_mode == 3:
        
            if self.cnt == self.N -1:
                if self.control_mode == 2:
                    yaw_d = self.heading / 180 * pi
                else:
                    try:
                        yaw_d = atan2(self.land_y - self.pos_now.vector.y, self.land_x - self.pos_now.vector.x)
                    except:
                        yaw_d = self.heading / 180 * pi

                err = atan2(sin(self.yaw_now - yaw_d), cos(self.yaw_now - yaw_d))
                err = max(min(err, self.max_heading_diff/180*pi), -self.max_heading_diff/180*pi) # keep the error in linear zone
                up = self.Kp*err
                ud = self.Kd*self.yaw_dot_now
                u = up + ud

                up_msg = Float32()
                up_msg.data = up
                self.up_pub.publish(up_msg)
                ud_msg = Float32()
                ud_msg.data = ud
                self.ud_pub.publish(ud_msg)
            
            else:
                u = self.u_last
        
        else:
            u = 0

        self.u_last = u
        cmd_msg = Vector3Stamped()
        cmd_msg.vector.z = 0.0
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        if u >= 0:
            cmd_msg.vector.x = 0.5
            cmd_msg.vector.y = max(0.5 - u, 0.0)
        else:
            cmd_msg.vector.x = max(0.5 + u, 0.0)
            cmd_msg.vector.y = 0.5
        
        self.cmd_pub.publish(cmd_msg)
        
        self.cnt += 1
        if self.cnt >= self.N:
            self.cnt = 0



def main(args=None):
    rclpy.init(args=args)
    heuristic_ctrler = HeuristicCtrler()
    rclpy.spin(heuristic_ctrler)
    heuristic_ctrler.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# ubuntu@ubuntu-desktop:~/Github/ros2_ws$ ros2 run heuristic_ctrler heuristic_ctrler --ros-args --params-file ~/Github/ros2_ws/src/heuristic_ctrler/config/params.yaml
