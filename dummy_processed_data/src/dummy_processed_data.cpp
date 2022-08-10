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

#include "geometry_msgs/msg/vector3_stamped.hpp"

using Vector3Stamped = geometry_msgs::msg::Vector3Stamped;

#define PI 3.14159265358979323846


class DummyProcessedDataPub : public rclcpp::Node
{
    public:
    DummyProcessedDataPub() : Node("dummy_processed_data_pub"), timer_count_(0), publisher_timestep_in_ms_(100)
    {
        timer_ = this->create_wall_timer(std::chrono::milliseconds(publisher_timestep_in_ms_), 
                                         std::bind(&DummyProcessedDataPub::timer_callback, this));

        pos_pub_ = this->create_publisher<Vector3Stamped>("position", 1);
        // vel_pub_ = this->create_publisher<Vector3Stamped>("velocity", 10);
        pose_pub_ = this->create_publisher<Vector3Stamped>("debug_rpy", 1);
        body_acc_pub_ = this->create_publisher<Vector3Stamped>("body_acc", 1);
        body_ang_vel_pub_ = this->create_publisher<Vector3Stamped>("body_ang_vel", 1);
    }

    private:
    void timer_callback()
    {
        double current_time_approx = static_cast<double>(timer_count_)*static_cast<double>(publisher_timestep_in_ms_)/1000; // [s]

        auto pos = Vector3Stamped();
        pos.vector.x = 0.5*cos(current_time_approx*z_w_); 
        pos.vector.y = 0.5*sin(current_time_approx*z_w_); 
        pos.vector.z = -1.0;
        pos.header.stamp = this->get_clock()->now();
        pos_pub_->publish(pos);

        // auto vel = interfaces::msg::Velocity();
        // vel.vx = -0.5*z_w_*sin(current_time_approx*z_w_);
        // vel.vy = 0.5*z_w_*cos(current_time_approx*z_w_);
        // vel.vz = 0.0;
        // vel.header.stamp = this->get_clock()->now();
        // vel_pub_->publish(vel);

        auto pose = Vector3Stamped();
        pose.vector.x = 0.0; pose.vector.y = 0.0; 
        pose.vector.z = my_mod(current_time_approx*z_w_); // [-pi, pi)
        pose.header.stamp = this->get_clock()->now();
        pose_pub_->publish(pose);

        auto body_acc = Vector3Stamped();
        body_acc.vector.x = -z_w_*z_w_*0.5; body_acc.vector.y = 0.0; body_acc.vector.z = 0.0;
        body_acc.header.stamp = this->get_clock()->now();
        body_acc_pub_->publish(body_acc);

        auto body_ang_vel = Vector3Stamped();
        body_ang_vel.vector.x = 0.0; body_ang_vel.vector.y = 0.0; body_ang_vel.vector.z = z_w_;
        body_ang_vel.header.stamp = this->get_clock()->now();
        body_ang_vel_pub_->publish(body_ang_vel);

        // RCLCPP_INFO(this->get_logger(), "Published once,  heading: %.3f, timer_count: %.1f", pose.yaw, static_cast<double>(timer_count_));
        RCLCPP_INFO(this->get_logger(), "Published once");

        ++timer_count_;
    }

    double my_mod(double x)
    {
        return atan2(sin(x), cos(x));
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Vector3Stamped>::SharedPtr pos_pub_;
    // rclcpp::Publisher<interfaces::msg::Velocity>::SharedPtr vel_pub_;
    rclcpp::Publisher<Vector3Stamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<Vector3Stamped>::SharedPtr body_acc_pub_;
    rclcpp::Publisher<Vector3Stamped>::SharedPtr body_ang_vel_pub_;

    size_t timer_count_;
    size_t publisher_timestep_in_ms_;

    // for testing
    double z_w_ = 0.1; // [rad/s]
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyProcessedDataPub>());
  rclcpp::shutdown();
  return 0;
}