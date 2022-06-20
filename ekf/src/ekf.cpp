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


using std::placeholders::_1;

class EKF : public rclcpp::Node
{
  public:
    EKF()
    : Node("ekf")
    {
      pos_sub_ = this->create_subscription<interfaces::msg::Position>(
      "dummy_position", 10, std::bind(&EKF::position_callback, this, _1));
    }

  private:
    void position_callback(const interfaces::msg::Position::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I am at: (%.2f, %.2f, %.2f)", msg->x, msg->y, msg->z);
    }

    rclcpp::Subscription<interfaces::msg::Position>::SharedPtr pos_sub_;
    // rclcpp::Subscription<interfaces::msg::Velocity>::SharedPtr vel_sub_;
    // rclcpp::Subscription<interfaces::msg::Pose>::SharedPtr pose_sub_;
    // rclcpp::Subscription<interfaces::msg::BodyAcceleration>::SharedPtr body_acc_sub_;
    // rclcpp::Subscription<interfaces::msg::BodyAngularVelocity>::SharedPtr body_omega_sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKF>());
  rclcpp::shutdown();
  return 0;
}