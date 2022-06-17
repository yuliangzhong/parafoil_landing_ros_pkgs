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


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class DummyProcessedDataPub : public rclcpp::Node
{
    public:
    DummyProcessedDataPub() : Node("dummy_processed_data_pub")
    {
        timer_ = this->create_wall_timer(500ms, std::bind(&DummyProcessedDataPub::timer_callback, this));

        pos_pub_ = this->create_publisher<interfaces::msg::Position>("dummy_position", 10);
        vel_pub_ = this->create_publisher<interfaces::msg::Velocity>("dummy_velocity", 10);
        pose_pub_ = this->create_publisher<interfaces::msg::Pose>("dummy_pose", 10);
        body_acc_pub_ = this->create_publisher<interfaces::msg::BodyAcceleration>("dummy_body_acc", 10);
        body_omega_pub_ = this->create_publisher<interfaces::msg::BodyAngularVelocity>("dummy_body_omega", 10);
    }

    private:
    void timer_callback()
    {
        auto pos = interfaces::msg::Position();
        pos.x = 0.0; pos.y = 0.0; pos.z = 0.0;
        pos_pub_->publish(pos);

        auto vel = interfaces::msg::Velocity();
        vel.vx = 0.0; vel.vy = 0.0; vel.vz = 0.0;
        vel_pub_->publish(vel);

        auto pose = interfaces::msg::Pose();
        pose.roll = 0.0; pose.pitch = 0.0; pose.yaw = 0.0;
        pose_pub_->publish(pose);

        auto body_acc = interfaces::msg::BodyAcceleration();
        body_acc.ax = 0.0; body_acc.ay = 0.0; body_acc.az = 0.0;
        body_acc_pub_->publish(body_acc);

        auto body_omega = interfaces::msg::BodyAngularVelocity();
        body_omega.wx = 0.0; body_omega.wy = 0.0; body_omega.wz = 0.0;
        body_omega_pub_->publish(body_omega);

        RCLCPP_INFO(this->get_logger(), "Published once");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::Position>::SharedPtr pos_pub_;
    rclcpp::Publisher<interfaces::msg::Velocity>::SharedPtr vel_pub_;
    rclcpp::Publisher<interfaces::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<interfaces::msg::BodyAcceleration>::SharedPtr body_acc_pub_;
    rclcpp::Publisher<interfaces::msg::BodyAngularVelocity>::SharedPtr body_omega_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyProcessedDataPub>());
  rclcpp::shutdown();
  return 0;
}