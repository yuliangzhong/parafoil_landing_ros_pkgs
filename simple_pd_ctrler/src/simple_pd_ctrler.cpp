#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/vector3_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::min;
using std::max;

using Vector3Stamped = geometry_msgs::msg::Vector3Stamped;

#define PI 3.14159265358979323846

const double dL = 0.05;

class SimplePdCtrler : public rclcpp::Node
{
  public:
    SimplePdCtrler() : Node("simple_pd_ctrler")
    {
      cmd_pub = this->create_publisher<Vector3Stamped>("delta_left_right_01", 1);
      timer_ = this->create_wall_timer(100ms, std::bind(&SimplePdCtrler::cmd_callback, this));

      vel_sub = this->create_subscription<Vector3Stamped>("estimated_vel", 1, std::bind(&SimplePdCtrler::vel_callback, this, _1));
      body_ang_vel_sub = this->create_subscription<Vector3Stamped>(
                                             "body_ang_vel", 1, std::bind(&SimplePdCtrler::body_ang_vel_callback, this, _1));
    }

  private:
    void cmd_callback()
    {
      if (vel_update_flag == false or ang_update_flag == false) {return; }

      double yaw_now = vel_now.vector.x;
      // interpret estimated vel
      if (vel_now.vector.y < 0)
      {
        yaw_now = vel_now.vector.x + PI;
      }
      double delta_yaw = yaw_now - yaw_d;
      double yaw_dot_now = body_ang_vel_now.vector.z; // approx

      double err = atan2(sin(delta_yaw), cos(delta_yaw));
      double u = Kp*err + Kd*yaw_dot_now;

      auto cmd = Vector3Stamped();
      cmd.header.stamp = this->get_clock()->now();
      cmd.vector.x = min(min(0.5 + u, 1.0), last_cmd_l + dL);
      cmd.vector.x = max(max(cmd.vector.x, 0.0), last_cmd_l - dL);
      cmd.vector.y = min(min(0.5 - u, 1.0), last_cmd_r + dL);
      cmd.vector.y = max(max(cmd.vector.y, 0.0), last_cmd_r - dL);
      cmd.vector.z = 0.0;
      cmd_pub->publish(cmd);
      last_cmd_l = cmd.vector.x;
      last_cmd_r = cmd.vector.y;
     
      RCLCPP_INFO(this->get_logger(), "delta_l, delta_r = [%.2f, %.2f]", cmd.vector.x, cmd.vector.y);
      
      vel_update_flag = false;
      ang_update_flag = false;
    }

    void vel_callback(const Vector3Stamped & msg)
    {
      vel_now = msg;
      vel_update_flag = true;
    }

    void body_ang_vel_callback(const Vector3Stamped & msg)
    {
      body_ang_vel_now = msg;
      ang_update_flag = true;
    }

    // desired heading
    double yaw_d = 0.0;
    double Kp = 1;  // >=0
    double Kd = 0.1; // >=0

    // state storage
    Vector3Stamped vel_now;
    Vector3Stamped body_ang_vel_now;
    double last_cmd_l = 0;
    double last_cmd_r = 0;

    // publisher
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Vector3Stamped>::SharedPtr cmd_pub;
    
    // subscribers
    rclcpp::Subscription<Vector3Stamped>::SharedPtr vel_sub;
    rclcpp::Subscription<Vector3Stamped>::SharedPtr body_ang_vel_sub;

    // update flags
    bool vel_update_flag = false;
    bool ang_update_flag = false;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePdCtrler>());
  rclcpp::shutdown();
  return 0;
}