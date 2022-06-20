#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "interfaces/msg/position.hpp"
#include "interfaces/msg/velocity.hpp"
#include "interfaces/msg/pose.hpp"
#include "interfaces/msg/body_acceleration.hpp"
#include "interfaces/msg/body_angular_velocity.hpp"
#include "interfaces/msg/states.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;

using Position = interfaces::msg::Position;
using Velocity = interfaces::msg::Velocity;
using Pose = interfaces::msg::Pose;
using Acc = interfaces::msg::BodyAcceleration;
using Omega = interfaces::msg::BodyAngularVelocity;

class EKF : public rclcpp::Node
{
  public:
    EKF() : Node("ekf")
    {
      pos_sub_.subscribe(this, "dummy_position");
      vel_sub_.subscribe(this, "dummy_velocity");
      pose_sub_.subscribe(this, "dummy_pose");
      body_acc_sub_.subscribe(this, "dummy_body_acc");
      body_omega_sub_.subscribe(this, "dummy_body_omega");

      sync_ = std::make_shared<message_filters::TimeSynchronizer<Position, Velocity, Pose, Acc, Omega>>
                                                      (pos_sub_, vel_sub_, pose_sub_, body_acc_sub_, body_omega_sub_, 10);
      
      sync_->registerCallback(std::bind(&EKF::sync_callback, this, _1, _2, _3, _4, _5));
    }

  private:
    void sync_callback(const Position::ConstSharedPtr& msg_pos,
                       const Velocity::ConstSharedPtr& msg_vel,
                       const Pose::ConstSharedPtr& msg_pose,
                       const Acc::ConstSharedPtr& msg_body_acc,
                       const Omega::ConstSharedPtr& msg_body_omega) const
    {
      // implement Extended Kalman Filter here

      RCLCPP_INFO(this->get_logger(), "I am at: (%.2f, %.2f, %.2f)", msg_pos->x, msg_pos->y, msg_pos->z);
    }

    message_filters::Subscriber<Position> pos_sub_;
    message_filters::Subscriber<Velocity> vel_sub_;
    message_filters::Subscriber<Pose> pose_sub_;
    message_filters::Subscriber<Acc> body_acc_sub_;
    message_filters::Subscriber<Omega> body_omega_sub_;
      
    std::shared_ptr<message_filters::TimeSynchronizer<Position, Velocity, Pose, Acc, Omega>> sync_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKF>());
  rclcpp::shutdown();
  return 0;
}

// reference
// https://answers.ros.org/question/361637/using-c-message-filters-in-ros2/
// https://github.com/alsora/ros2-code-examples/blob/master/simple_time_sync/main.cpp
// https://answers.ros.org/question/291876/how-can-i-subscribe-to-2-different-topics-in-a-synchronised-manner-in-ros2/
