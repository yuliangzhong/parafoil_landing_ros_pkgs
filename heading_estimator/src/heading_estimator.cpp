#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include <cmath>
#include <eigen3/Eigen/Dense>

using MatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using Vector3Stamped = geometry_msgs::msg::Vector3Stamped;
using std::placeholders::_1;

class HeadingEstimator : public rclcpp::Node
{
  public:
    HeadingEstimator() : Node("heading_estimater")
    {
      yaw_avg_pub = this->create_publisher<Vector3Stamped>("yaw_avg", 1);
      pos_sub = this->create_subscription<Vector3Stamped>("position", 10, std::bind(&HeadingEstimator::sub_callback, this, _1));
    }

  private:
    void sub_callback(const Vector3Stamped & msg) const
    {
        // MatrixXd tmp = MatrixXd::Zero(2,sz);
        // tmp.block(0,0,2,sz-1) = pos_buffer.block(0,1,2,sz-1);
        // tmp(0,sz-1) = msg.vector.x;
        // tmp(1,sz-1) = msg.vector.y;
        
        // pos_buffer = tmp;

        // if (pos_buffer(0,0) != 0.0 and pos_buffer(1,0) != 0.0)
        // {
        //     double dx = pos_buffer.block(0,sz/2,1,sz/2).mean() - pos_buffer.block(0,0,1,sz/2).mean();
        //     double dy = pos_buffer.block(1,sz/2,1,sz/2).mean() - pos_buffer.block(1,0,1,sz/2).mean();
        //     double yaw_avg = atan2(dy, dx);

        //     auto yaw_msg = Vector3Stamped();
        //     // yaw_msg.header.stamp = this->get_clock()->now();
        //     yaw_msg.vector.x = yaw_avg;
        //     yaw_msg.vector.y = 0.0;
        //     yaw_msg.vector.z = 0.0;
        //     yaw_avg_pub->publish(yaw_msg);
        //     RCLCPP_INFO(this->get_logger(), "Avg yaw: %.3f", yaw_avg);
                 
    }

    rclcpp::Publisher<Vector3Stamped>::SharedPtr yaw_avg_pub;
    rclcpp::Subscription<Vector3Stamped>::SharedPtr pos_sub;
    size_t sz = 60; // should be even
    MatrixXd pos_buffer = MatrixXd::Zero(2,sz);

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeadingEstimator>());
  rclcpp::shutdown();
  return 0;
}