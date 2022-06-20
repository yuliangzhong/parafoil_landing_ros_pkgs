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

#include <iostream>
using std::endl;
using std::cout;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;


class EKF : public rclcpp::Node
{
  public:
    EKF() : Node("ekf")
    {
      // pos_sub_ = this->create_subscription<interfaces::msg::Position>(
      // "dummy_position", 10, std::bind(&EKF::position_callback, this, _1));
      pos_sub_.subscribe(this, "dummy_position");
      vel_sub_.subscribe(this, "dummy_velocity");
      pose_sub_.subscribe(this, "dummy_pose");
      body_acc_sub_.subscribe(this, "dummy_body_acc");
      body_omega_sub_.subscribe(this, "dummy_body_omega");

      sync_ = std::make_shared<message_filters::TimeSynchronizer<interfaces::msg::Position, 
                                                      interfaces::msg::Velocity,
                                                      interfaces::msg::Pose,
                                                      interfaces::msg::BodyAcceleration,
                                                      interfaces::msg::BodyAngularVelocity>>
                                                      (pos_sub_, vel_sub_, pose_sub_, body_acc_sub_, body_omega_sub_, 6);
      
      // syncApproximate_.registerCallback(std::bind(&EKF::sync_callback, this, _1, _2, _3, _4, _5));
    }

  private:
    void sync_callback(const interfaces::msg::Position::ConstSharedPtr& msg_pos,
                       const interfaces::msg::Velocity::ConstSharedPtr& msg_vel,
                       const interfaces::msg::Pose::ConstSharedPtr& msg_pose,
                       const interfaces::msg::BodyAcceleration::ConstSharedPtr& msg_body_acc,
                       const interfaces::msg::BodyAngularVelocity::ConstSharedPtr& msg_body_omega) const
    {
      cout<<"in sync_callback"<<endl;
      RCLCPP_INFO(this->get_logger(), "I am at: (%.2f, %.2f, %.2f)", msg_pos->x, msg_pos->y, msg_pos->z);
    }

    // rclcpp::Subscription<interfaces::msg::Position>::SharedPtr pos_sub_;
    // rclcpp::Subscription<interfaces::msg::Velocity>::SharedPtr vel_sub_;
    // rclcpp::Subscription<interfaces::msg::Pose>::SharedPtr pose_sub_;
    // rclcpp::Subscription<interfaces::msg::BodyAcceleration>::SharedPtr body_acc_sub_;
    // rclcpp::Subscription<interfaces::msg::BodyAngularVelocity>::SharedPtr body_omega_sub_;
    message_filters::Subscriber<interfaces::msg::Position> pos_sub_;
    message_filters::Subscriber<interfaces::msg::Velocity> vel_sub_;
    message_filters::Subscriber<interfaces::msg::Pose> pose_sub_;
    message_filters::Subscriber<interfaces::msg::BodyAcceleration> body_acc_sub_;
    message_filters::Subscriber<interfaces::msg::BodyAngularVelocity> body_omega_sub_;

    // typedef message_filters::sync_policies::ApproximateTime<interfaces::msg::Position, interfaces::msg::Velocity, 
    //                                                           interfaces::msg::Pose, interfaces::msg::BodyAcceleration,
    //                                                           interfaces::msg::BodyAngularVelocity> approximate_policy;
    // message_filters::Synchronizer<approximate_policy> syncApproximate_(approximate_policy(10), pos_sub_, vel_sub_, pose_sub_, body_acc_sub_, body_omega_sub_);
      

    std::shared_ptr<message_filters::TimeSynchronizer<interfaces::msg::Position, 
                                                      interfaces::msg::Velocity,
                                                      interfaces::msg::Pose,
                                                      interfaces::msg::BodyAcceleration,
                                                      interfaces::msg::BodyAngularVelocity>> sync_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKF>());
  rclcpp::shutdown();
  return 0;
}