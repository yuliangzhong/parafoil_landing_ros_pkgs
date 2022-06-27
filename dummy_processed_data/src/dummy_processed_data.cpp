#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <cmath>

#include "interfaces/msg/position.hpp"
#include "interfaces/msg/velocity.hpp"
#include "interfaces/msg/pose.hpp"
#include "interfaces/msg/body_acceleration.hpp"
#include "interfaces/msg/body_angular_velocity.hpp"

#define PI 3.14159265358979323846


class DummyProcessedDataPub : public rclcpp::Node
{
    public:
    DummyProcessedDataPub() : Node("dummy_processed_data_pub"), timer_count_(0), publisher_timestep_in_ms_(100)
    {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(publisher_timestep_in_ms_), 
                                         std::bind(&DummyProcessedDataPub::timer_callback, this));

        pos_pub_ = this->create_publisher<interfaces::msg::Position>("dummy_position", 10);
        vel_pub_ = this->create_publisher<interfaces::msg::Velocity>("dummy_velocity", 10);
        pose_pub_ = this->create_publisher<interfaces::msg::Pose>("dummy_pose", 10);
        body_acc_pub_ = this->create_publisher<interfaces::msg::BodyAcceleration>("dummy_body_acc", 10);
        body_omega_pub_ = this->create_publisher<interfaces::msg::BodyAngularVelocity>("dummy_body_omega", 10);
    }

    private:
    void timer_callback()
    {
        double current_time_approx = timer_count_*publisher_timestep_in_ms_/1000; // [s]

        auto pos = interfaces::msg::Position();
        pos.x = 0.0; 
        pos.y = 0.0; pos.z = -1.0;
        pos.header.stamp = this->get_clock()->now();
        pos_pub_->publish(pos);

        auto vel = interfaces::msg::Velocity();
        vel.vx = 0.0;
        vel.vy = 0.0; vel.vz = 0.0;
        vel.header.stamp = this->get_clock()->now();
        vel_pub_->publish(vel);

        auto pose = interfaces::msg::Pose();
        pose.roll = 0.0; pose.pitch = 0.0; 
        pose.yaw = my_mod(current_time_approx*z_w_ + PI, 2*PI) - PI; // [-pi, pi)
        pose.header.stamp = this->get_clock()->now();
        pose_pub_->publish(pose);

        auto body_acc = interfaces::msg::BodyAcceleration();
        body_acc.ax = 0.0; body_acc.ay = 0.0; body_acc.az = 0.0;
        body_acc.header.stamp = this->get_clock()->now();
        body_acc_pub_->publish(body_acc);

        auto body_omega = interfaces::msg::BodyAngularVelocity();
        body_omega.wx = 0.0; body_omega.wy = 0.0; body_omega.wz = z_w_;
        body_omega.header.stamp = this->get_clock()->now();
        body_omega_pub_->publish(body_omega);

        RCLCPP_INFO(this->get_logger(), "Published once,  heading: %.2f", pose.yaw);

        ++timer_count_;
    }

    double my_mod(double x, double y)
    {
        return x - floor(x/y)*y;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<interfaces::msg::Position>::SharedPtr pos_pub_;
    rclcpp::Publisher<interfaces::msg::Velocity>::SharedPtr vel_pub_;
    rclcpp::Publisher<interfaces::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<interfaces::msg::BodyAcceleration>::SharedPtr body_acc_pub_;
    rclcpp::Publisher<interfaces::msg::BodyAngularVelocity>::SharedPtr body_omega_pub_;

    size_t timer_count_;
    size_t publisher_timestep_in_ms_;

    // for testing
    double x_vel_ = 0.01; // [m/s]
    double z_w_ = 0.5; // [rad/s]
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyProcessedDataPub>());
  rclcpp::shutdown();
  return 0;
}