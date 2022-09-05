import rclpy
from rclpy.node import Node
import sys
import time
import subprocess
from geometry_msgs.msg import Vector3Stamped

cmd = "ros2 bag record -a"
time_header = '166236'
start_times = [3845, 4731, 5161, 6392]
end_times   = [3907, 4789, 5227, 6449]
times_cnt = 4

class MsgRepub(Node):

    def __init__(self):
        super().__init__('msg_repub')
        self.subscription = self.create_subscription(Vector3Stamped, 'position', self.listener_callback, 1)
        self.ind = 0
        self.start_time = int(time_header+str(start_times[self.ind]))
        self.end_time =   int(time_header+str(end_times[self.ind]))
        self.process = None
        self.record_flag = False

    def listener_callback(self, msg):
        print(msg.header.stamp.sec , self.start_time)
        if msg.header.stamp.sec < self.start_time:
            print("Current ind: %d. You have to wait %d seconds" % (self.ind, self.start_time - msg.header.stamp.sec))
        elif self.start_time <= msg.header.stamp.sec <= self.end_time:
            if self.record_flag == False:
                self.process = subprocess.Popen("exec "+cmd, stdout=subprocess.PIPE, shell=True)
                self.record_flag = True
                time.sleep(3)
            print("%.2f %% recorded" % ((msg.header.stamp.sec - self.start_time)/(self.end_time - self.start_time)*100))
        elif msg.header.stamp.sec > self.end_time:
            self.process.terminate()
            time.sleep(3)
            self.record_flag = False
            self.ind += 1
            if self.ind < times_cnt:
                self.start_time = int(time_header+str(start_times[self.ind]))
                self.end_time =   int(time_header+str(end_times[self.ind]))
            else:
                print("All done!")
                sys.exit()


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