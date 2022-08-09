#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/states.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

using std::placeholders::_1;
using std::placeholders::_2;

using Vector3Stamped = geometry_msgs::msg::Vector3Stamped;
using States = interfaces::msg::States;

using ApproPolicy = message_filters::sync_policies::ApproximateTime<States, Vector3Stamped>;
using Sync = message_filters::Synchronizer<ApproPolicy>;

#define PI 3.14159265358979323846

class SimplePdCtrler : public rclcpp::Node
{
  public:
    SimplePdCtrler() : Node("simple_pd_ctrler"), sync_(Sync(ApproPolicy(10), states_sub, body_ang_vel_sub))
    {
      cmd_pub = this->create_publisher<Vector3Stamped>("delta_left_right_01", 1);

      states_sub.subscribe(this, "estimated_states");
      body_ang_vel_sub.subscribe(this, "body_ang_vel");

      // for approximate sync
      sync_.registerCallback(std::bind(&SimplePdCtrler::control_callback, this, _1, _2));
    }

  private:
    void control_callback(const States::ConstSharedPtr& msg_states,
                          const Vector3Stamped::ConstSharedPtr& msg_body_acc)
    {
      double delta_yaw = msg_states->yaw - yaw_d;
      double yaw_dot_now = msg_body_acc->vector.z; // approx

      double err = warp2pi(delta_yaw);
      double u = Kp*err + Kd*yaw_dot_now;

      auto cmd = Vector3Stamped();
      cmd.header.stamp = this->get_clock()->now();
      cmd.vector.x = 0.5 + u;
      cmd.vector.y = 0.5 - u;
      cmd.vector.z = 0.0;
      cmd_pub->publish(cmd);
     
      RCLCPP_INFO(this->get_logger(), "delta_l, delta_r = [%.2f, %.2f]", cmd.vector.x, cmd.vector.y);
    }

    double warp2pi(double x)
    {
        return atan2(sin(x), cos(x));
    }

    // desired heading
    double yaw_d = 0.0;
    double Kp = 1;  // >=0
    double Kd = 0.03; // >=0

    // subscribers
    message_filters::Subscriber<States> states_sub;
    message_filters::Subscriber<Vector3Stamped> body_ang_vel_sub;
    
    // for approximate sync
    Sync sync_;

    // publisher
    rclcpp::Publisher<Vector3Stamped>::SharedPtr cmd_pub;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePdCtrler>());
  rclcpp::shutdown();
  return 0;
}