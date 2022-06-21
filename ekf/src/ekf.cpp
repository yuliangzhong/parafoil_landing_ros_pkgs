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

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include <eigen3/Eigen/Dense>
#include <cmath>

#include <iostream>

using std::cout; using std::endl;

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;
using std::placeholders::_5;

using Position = interfaces::msg::Position;
using Velocity = interfaces::msg::Velocity;
using Pose = interfaces::msg::Pose;
using Acc = interfaces::msg::BodyAcceleration;
using Omega = interfaces::msg::BodyAngularVelocity;

using ApproPolicy = message_filters::sync_policies::ApproximateTime<Position, Velocity, Pose, Acc, Omega>;
using Sync = message_filters::Synchronizer<ApproPolicy>;

using MatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;
// using Matrix9d = Eigen::Matrix<double, 9, 9>;
// using Vector9d = Eigen::Matrix<double, 9, 1>;
// using Matrix6d = Eigen::Matrix<double, 6, 6>;
// using Vector6d = Eigen::Matrix<double, 6, 1>;

#define pi 3.141592653

class EKF : public rclcpp::Node
{
  public:
    EKF() : Node("ekf"), sync_(Sync(ApproPolicy(10), pos_sub_, vel_sub_, pose_sub_, body_acc_sub_, body_omega_sub_))
    {
      pos_sub_.subscribe(this, "dummy_position");
      vel_sub_.subscribe(this, "dummy_velocity");
      pose_sub_.subscribe(this, "dummy_pose");
      body_acc_sub_.subscribe(this, "dummy_body_acc");
      body_omega_sub_.subscribe(this, "dummy_body_omega");

      // for exact sync
      // sync_ = std::make_shared<message_filters::TimeSynchronizer<Position, Velocity, Pose, Acc, Omega>>
      //                                                 (pos_sub_, vel_sub_, pose_sub_, body_acc_sub_, body_omega_sub_, 10);
      // sync_->registerCallback(std::bind(&EKF::sync_ekf_callback, this, _1, _2, _3, _4, _5));

      // for approximate sync
      sync_.registerCallback(std::bind(&EKF::sync_ekf_callback, this, _1, _2, _3, _4, _5));

      // Read from param
      Ts_ = 0.5; // [ms]
      // initialize EKF parameters
      states_mu_ = VectorXd::Zero(9);

      states_sigma_ = MatrixXd::Zero(9,9);
      states_sigma_.block(0,0,3,3) = 4*MatrixXd::Identity(3,3);
      states_sigma_.block(3,3,3,3) = 2*MatrixXd::Identity(3,3);
      states_sigma_.block(6,6,3,3) = 0.4*MatrixXd::Identity(3,3);

      Q_ = MatrixXd::Zero(6,6);
      Q_.block(0,0,3,3) = pow(0.1,2)*MatrixXd::Identity(3,3);
      Q_.block(3,3,3,3) = pow(0.1/180*pi,2)*MatrixXd::Identity(3,3);

      R_ = MatrixXd::Zero(9,9);
      R_.block(0,0,3,3) = pow(2,2)*MatrixXd::Identity(3,3);
      R_.block(3,3,3,3) = pow(0.1,2)*MatrixXd::Identity(3,3);
      R_(6,6) = pow(0.1/180*pi,2);
      R_(7,7) = pow(0.1/180*pi,2);
      R_(8,8) = pow(0.5/180*pi,2);
    }

  private:
    void sync_ekf_callback(const Position::ConstSharedPtr& msg_pos,
                           const Velocity::ConstSharedPtr& msg_vel,
                           const Pose::ConstSharedPtr& msg_pose,
                           const Acc::ConstSharedPtr& msg_body_acc,
                           const Omega::ConstSharedPtr& msg_body_omega)
    {
      ///////// Extended Kalman Filter Implementation Starts
      VectorXd U(6);
      U << msg_body_acc->ax, msg_body_acc->ay, msg_body_acc->az,
           msg_body_omega->wx, msg_body_omega->wy, msg_body_omega->wz;
      VectorXd z_bar(9);
      z_bar << msg_pos->x, msg_pos->y, msg_pos->z,
               msg_vel->vx, msg_vel->vy, msg_vel->vz,
               msg_pose->roll, msg_pose-> pitch, msg_pose->yaw;

      // Prior update
      VectorXd xp = VectorXd::Zero(9);
      MatrixXd A = MatrixXd::Zero(9,9); 
      MatrixXd S = MatrixXd::Zero(9,6);
      MatrixXd Pp = A*states_sigma_*A.transpose() + S*Q_*S.transpose();

      // Measurement update
      VectorXd z = xp;
      MatrixXd H = MatrixXd::Identity(9,9);
      MatrixXd M = MatrixXd::Identity(9,9);
      MatrixXd K = ((H*Pp*H.transpose() + M*R_*M.transpose()).transpose().ldlt().solve((Pp*H.transpose()).transpose())).transpose();
      // MatrixXd K = Pp*H.transpose()*(H*Pp*H.transpose() + M*R_*M.transpose()).inverse();
      VectorXd Dz = z_bar - z;
      Dz(8) = fmod(Dz(8)+pi, 2*pi) - pi;

      states_mu_ = xp + K*Dz;
      states_sigma_ = (MatrixXd::Identity(9,9) - K*H)*Pp;

      // publish current estimation
      // states_pub = fmod(states_mu_(8)+pi, 2*pi) - pi;








      RCLCPP_INFO(this->get_logger(), "I am at: (%.2f, %.2f, %.2f)", msg_pos->x, msg_pos->y, msg_pos->z);
      
      ///////// Extended Kalman Filter Ends
    }

    message_filters::Subscriber<Position> pos_sub_;
    message_filters::Subscriber<Velocity> vel_sub_;
    message_filters::Subscriber<Pose> pose_sub_;
    message_filters::Subscriber<Acc> body_acc_sub_;
    message_filters::Subscriber<Omega> body_omega_sub_;

    // for exact sync
    // std::shared_ptr<message_filters::TimeSynchronizer<Position, Velocity, Pose, Acc, Omega>> sync_;
    
    // for approximate sync
    Sync sync_;

    // EKF parameters
    double Ts_;
    VectorXd states_mu_; // [x y z vx vy vz roll pitch yaw] 9*1
    MatrixXd states_sigma_; // 9*9
    MatrixXd Q_; // input noise 6*6
    MatrixXd R_; // sensor noise 9*9
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKF>());
  rclcpp::shutdown();
  return 0;
}

// reference
// https://answers.ros.org/question/361637/using-c-message-filters-in-ros2/
// https://github.com/alsora/ros2-code-examples/blob/master/simple_time_sync/main.cpp
// https://answers.ros.org/question/291876/how-can-i-subscribe-to-2-different-topics-in-a-synchronised-manner-in-ros2/
// https://answers.ros.org/question/280856/synchronizer-with-approximate-time-policy-in-a-class-c/
// https://eigen.tuxfamily.org/dox/group__QuickRefPage.html