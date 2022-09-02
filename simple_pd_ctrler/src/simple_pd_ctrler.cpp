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

const double dT = 0.1; // [s]
const double dL = 0.5 * dT; // normalized [0,1] brake change for each time step (dT)

class SimplePdCtrler : public rclcpp::Node
{
  public:
    SimplePdCtrler() : Node("simple_pd_ctrler")
    {
      // delta_left_right_01
      cmd_pub = this->create_publisher<Vector3Stamped>("/rockpara_actuators_node/auto_commands", 1);
      timer_ = this->create_wall_timer(200ms, std::bind(&SimplePdCtrler::cmd_callback, this));

      vel_sub = this->create_subscription<Vector3Stamped>("estimated_vel", 1, std::bind(&SimplePdCtrler::vel_callback, this, _1));
    }

  private:
    void cmd_callback()
    {
      if (vel_update_flag == false)
      {
        vel_init_flag = false;
        last_cmd_l = 0.5;
        last_cmd_r = 0.5;
        return;
      }

      // interpret estimated vel
      double yaw_now = (vel_now.vector.y < 0 ? vel_now.vector.x + PI : vel_now.vector.x);
      double yaw_last = (vel_last.vector.y < 0 ? vel_last.vector.x + PI : vel_last.vector.x);
      
      // simple pd controller
      double err = atan2(sin(yaw_now - yaw_d), cos(yaw_now - yaw_d));
      double yaw_dot_now = atan2(sin(yaw_now - yaw_last), cos(yaw_now - yaw_last))/dT;
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
    }

    void vel_callback(const Vector3Stamped & msg)
    {
      if (vel_init_flag == false)
      {
        vel_last = msg;
        vel_init_flag = true;
      }
      else
      {
        vel_last = vel_now;
        vel_now = msg;
        vel_update_flag = true;
      }
    }

    // desired heading
    double yaw_d = 0.0;
    double Kp = 1;  // >=0
    double Kd = 0.1; // >=0

    // publishers / subscribers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Vector3Stamped>::SharedPtr cmd_pub;
    rclcpp::Subscription<Vector3Stamped>::SharedPtr vel_sub;

    // control flags
    bool vel_init_flag = false;
    bool vel_update_flag = false;

    // state storage
    Vector3Stamped vel_last;
    Vector3Stamped vel_now;
    double last_cmd_l;
    double last_cmd_r;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePdCtrler>());
  rclcpp::shutdown();
  return 0;
}