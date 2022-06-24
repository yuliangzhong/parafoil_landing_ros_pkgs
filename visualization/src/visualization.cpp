#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include "interfaces/msg/states.hpp"

using std::placeholders::_1;

#define PI 3.141592653


class Visualization : public rclcpp::Node
{
  public:
    explicit Visualization() : Node("visualization")
    {
      states_sub_ = this->create_subscription<interfaces::msg::States>("estimated_states", 10, std::bind(&Visualization::states_callback, this, _1));
      static_desired_landing_frame_tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      current_states_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      this->pub_static_desired_landing_frame();
    }

  private:
    void pub_static_desired_landing_frame()
    {
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "world";
      t.child_frame_id = "desired_landing_frame";

      t.transform.translation.x = 0;
      t.transform.translation.y = 0;
      t.transform.translation.z = 0;

      tf2::Quaternion q;
      q.setRPY(PI, 0, 0);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      static_desired_landing_frame_tf_publisher_->sendTransform(t);

    }
    void states_callback(const interfaces::msg::States::SharedPtr msg)
    {
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "desired_landing_frame";
      t.child_frame_id = "parafoil_base";

      t.transform.translation.x = msg->x;
      t.transform.translation.y = msg->y;
      t.transform.translation.z = msg->z;

      tf2::Quaternion q;
      q.setRPY(msg->roll, msg->pitch, msg->yaw);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      // Send the transformation
      current_states_tf_broadcaster_->sendTransform(t);
    }
    
    rclcpp::Subscription<interfaces::msg::States>::SharedPtr states_sub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_desired_landing_frame_tf_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> current_states_tf_broadcaster_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Visualization>());
  rclcpp::shutdown();
  return 0;
}