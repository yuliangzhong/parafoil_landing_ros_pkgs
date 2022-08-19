import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3Stamped


class MsgRepub(Node):

    def __init__(self):
        super().__init__('msg_repub')
        self.publisher_ = self.create_publisher(Vector3Stamped, 'body_ang_vel', 1)
        self.subscription = self.create_subscription(Vector3Stamped,'position',self.listener_callback,1)
        self.start_time = 1660475775
        self.end_time =   1660475914

    def listener_callback(self, msg):
        if self.start_time <= msg.header.stamp.sec <= self.end_time:
            body_msg = Vector3Stamped()
            body_msg.header = msg.header
            body_msg.vector.x, body_msg.vector.y, body_msg.vector.z = 0.0, 0.0, 0.0
            self.publisher_.publish(body_msg)
            print("publishing body ang")
        else:
            print("waiting %d" % msg.header.stamp.sec)


def main(args=None):
    rclpy.init(args=args)

    msg_repub = MsgRepub()

    rclpy.spin(msg_repub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    msg_repub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()