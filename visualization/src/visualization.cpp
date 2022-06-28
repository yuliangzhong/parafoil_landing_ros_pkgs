#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "interfaces/msg/states.hpp"

using std::placeholders::_1;

#define PI 3.14159265358979323846


class Visualization : public rclcpp::Node
{
  public:
    explicit Visualization() : Node("visualization")
    {
      states_sub_ = this->create_subscription<interfaces::msg::States>("estimated_states", 10, std::bind(&Visualization::states_callback, this, _1));
      
      static_desired_landing_frame_tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      this->pub_static_desired_landing_frame();
      
      current_states_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      x_vel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("x_vel", 10);
      y_vel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("y_vel", 10);
      z_vel_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("z_vel", 10);
    }

  private:
    void set_transform(geometry_msgs::msg::TransformStamped &t, double x, double y, double z, double roll, double pitch, double yaw)
    {
      t.header.stamp = this->get_clock()->now();
      t.transform.translation.x = x;
      t.transform.translation.y = y;
      t.transform.translation.z = z;

      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw); // = Z-Y-X euler angle
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
    }

    visualization_msgs::msg::Marker create_marker(const geometry_msgs::msg::TransformStamped &t, char axis, double scale)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = t.child_frame_id;
      m.header.stamp = t.header.stamp;
      m.type = visualization_msgs::msg::Marker::ARROW;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = 0;
      m.pose.position.y = 0;
      m.pose.position.z = 0;
      auto q_inv = tf2::Quaternion(t.transform.rotation.x, t.transform.rotation.y, 
                                   t.transform.rotation.z, t.transform.rotation.w).inverse();
      tf2::Quaternion q_rot;

      m.scale.x = scale;
      m.scale.y = 0.01;
      m.scale.z = 0.01;
      if (axis == 'x')
      {
        m.color.r = 1;  m.color.g = 0;  m.color.b = 0;
        q_rot = q_inv;
      }
      else if (axis == 'y')
      {
        m.color.r = 0;  m.color.g = 1;  m.color.b = 0;
        q_rot = q_inv*tf2::Quaternion(0, 0, sqrt(2)/2, sqrt(2)/2);
      }
      else
      {
        m.color.r = 0;  m.color.g = 0;  m.color.b = 1;
        q_rot = q_inv*tf2::Quaternion(0, -sqrt(2)/2, 0, sqrt(2)/2);
      }
      m.color.a = 1;
      m.pose.orientation.x = q_rot.x(); 
      m.pose.orientation.y = q_rot.y();
      m.pose.orientation.z = q_rot.z(); 
      m.pose.orientation.w = q_rot.w();
      return m;
    }

    void pub_static_desired_landing_frame()
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = "world";
      t.child_frame_id = "desired_landing_frame";
      set_transform(t, 0, 0, 0, PI, 0, 0);
      static_desired_landing_frame_tf_publisher_->sendTransform(t);
    }

    void states_callback(const interfaces::msg::States::SharedPtr msg)
    {
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = "desired_landing_frame";
      t.child_frame_id = "parafoil_base";
      set_transform(t, msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw);
      current_states_tf_broadcaster_->sendTransform(t);

      x_vel_pub_->publish(create_marker(t, 'x', 15*msg->vx));
      y_vel_pub_->publish(create_marker(t, 'y', 15*msg->vy));
      z_vel_pub_->publish(create_marker(t, 'z', 15*msg->vz));
    }
    
    rclcpp::Subscription<interfaces::msg::States>::SharedPtr states_sub_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_desired_landing_frame_tf_publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> current_states_tf_broadcaster_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr x_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr y_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr z_vel_pub_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Visualization>());
  rclcpp::shutdown();
  return 0;
}