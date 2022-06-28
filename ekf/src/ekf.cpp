#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

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

#define PI 3.14159265358979323846


class EKF : public rclcpp::Node
{
  public:
    EKF() : Node("ekf"), sync_(Sync(ApproPolicy(10), pos_sub_, vel_sub_, pose_sub_, body_acc_sub_, body_omega_sub_))
    {
      states_pub_ = this->create_publisher<interfaces::msg::States>("estimated_states", 10);

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
      Ts_ = 0.1; // [s]
      
      // initialize EKF parameters
      states_mu_ = VectorXd::Zero(9);

      states_sigma_ = MatrixXd::Zero(9,9);
      states_sigma_.block(0,0,3,3) = 4*MatrixXd::Identity(3,3);
      states_sigma_.block(3,3,3,3) = 2*MatrixXd::Identity(3,3);
      states_sigma_.block(6,6,3,3) = 0.4*MatrixXd::Identity(3,3);

      Q_ = MatrixXd::Zero(6,6);
      Q_.block(0,0,3,3) = pow(0.1,2)*MatrixXd::Identity(3,3);
      Q_.block(3,3,3,3) = pow(0.1/180*PI,2)*MatrixXd::Identity(3,3);

      R_ = MatrixXd::Zero(9,9);
      R_.block(0,0,3,3) = pow(2,2)*MatrixXd::Identity(3,3);
      R_.block(3,3,3,3) = pow(0.1,2)*MatrixXd::Identity(3,3);
      R_(6,6) = pow(0.1/180*PI,2);
      R_(7,7) = pow(0.1/180*PI,2);
      R_(8,8) = pow(0.5/180*PI,2);
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

      double phi = states_mu_(6); // roll
      double theta = states_mu_(7); // pitch
      double psi = states_mu_(8); // yaw

      MatrixXd C_IB(3,3);
      C_IB << cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(theta)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),
              cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),
              -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta);

      MatrixXd L(3,3);
      L <<  1, sin(phi)*tan(theta), cos(phi)*tan(theta),
            0, cos(phi), -sin(phi),
            0, sin(phi)/cos(theta), cos(phi)/cos(theta);

      MatrixXd dC_IB_dphi(3,3);
      dC_IB_dphi << 0, cos(phi)*sin(theta)*cos(psi), -sin(phi)*sin(theta)*cos(psi)+cos(phi)*sin(psi),
                    0, cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi), -sin(phi)*sin(theta)*sin(psi)-cos(phi)*cos(psi),
                    0, cos(phi)*cos(theta), -sin(phi)*cos(theta);

      MatrixXd dC_IB_dtheta(3,3); 
      dC_IB_dtheta << -sin(theta)*cos(psi), sin(phi)*cos(theta)*cos(psi)+sin(theta)*sin(psi), cos(phi)*cos(theta)*cos(psi),
                      -sin(theta)*sin(psi), sin(phi)*cos(theta)*sin(psi), cos(phi)*cos(theta)*sin(psi),
                      -cos(theta), -sin(phi)*sin(theta), -cos(phi)*sin(theta);

      MatrixXd dC_IB_dpsi(3,3); 
      dC_IB_dpsi << -cos(theta)*sin(psi), -sin(phi)*sin(theta)*sin(psi)-cos(theta)*cos(psi), -cos(phi)*sin(theta)*sin(psi)+sin(phi)*cos(psi),
                    cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi),
                    0, 0, 0;
      
      MatrixXd dL_dphi(3,3); 
      dL_dphi << 0, cos(phi)*tan(theta), -sin(phi)*tan(theta),
                 0, -sin(phi), -cos(phi),
                 0, cos(phi)/cos(theta), -sin(phi)/cos(theta);

      MatrixXd dL_dtheta(3,3); 
      dL_dtheta << 0, sin(phi)/pow(cos(theta),2), cos(phi)/pow(cos(theta),2),
                   0, 0, 0,
                   0, sin(phi)*sin(theta)/pow(cos(theta),2), cos(phi)*sin(theta)/pow(cos(theta),2);
      
      MatrixXd dL_dpsi = MatrixXd::Zero(3,3);

      MatrixXd J_C_IB(3,9);
      J_C_IB.block(0,0,3,3) = dC_IB_dphi;
      J_C_IB.block(0,3,3,3) = dC_IB_dtheta;
      J_C_IB.block(0,6,3,3) = dC_IB_dpsi;

      MatrixXd J_L(3,9);
      J_L.block(0,0,3,3) = dL_dphi;
      J_L.block(0,3,3,3) = dL_dtheta;
      J_L.block(0,6,3,3) = dL_dpsi;


      VectorXd xp = VectorXd::Zero(9);
      MatrixXd A = MatrixXd::Zero(9,9); 
      MatrixXd S = MatrixXd::Zero(9,6);

      xp.segment(0,3) = states_mu_.segment(0,3) + Ts_*states_mu_.segment(3,3);
      xp.segment(3,3) = states_mu_.segment(3,3) + Ts_*C_IB*U.segment(0,3);
      xp.segment(6,3) = states_mu_.segment(6,3) + Ts_*L*U.segment(3,3);

      MatrixXd Ua_diag(9,3);
      Ua_diag.block(0,0,3,1) = U.segment(0,3);
      Ua_diag.block(3,1,3,1) = U.segment(0,3);
      Ua_diag.block(6,2,3,1) = U.segment(0,3);

      MatrixXd Uw_diag(9,3);
      Uw_diag.block(0,0,3,1) = U.segment(3,3);
      Uw_diag.block(3,1,3,1) = U.segment(3,3);
      Uw_diag.block(6,2,3,1) = U.segment(3,3);

      A.block(0,0,3,3).setIdentity();
      A.block(0,3,3,3) = Ts_*MatrixXd::Identity(3,3);
      A.block(0,6,3,3).setZero();
      A.block(3,0,3,3).setZero();
      A.block(3,3,3,3).setIdentity();
      A.block(3,6,3,3) = Ts_*J_C_IB*Ua_diag;
      A.block(6,0,3,6).setZero();
      A.block(6,6,3,3) = MatrixXd::Identity(3,3) + Ts_*J_L*Uw_diag;

      S.block(0,0,3,6).setZero();
      S.block(3,0,3,3) = Ts_*C_IB;
      S.block(3,3,3,3).setZero();
      S.block(6,0,3,3).setZero();
      S.block(6,3,3,3) = Ts_*L;

      MatrixXd Pp = A*states_sigma_*A.transpose() + S*Q_*S.transpose();

      // Measurement update
      VectorXd z = xp;
      MatrixXd H = MatrixXd::Identity(9,9);
      MatrixXd M = MatrixXd::Identity(9,9);
      MatrixXd K_trans = (H*Pp*H.transpose() + M*R_*M.transpose()).transpose().ldlt().solve(H*Pp.transpose());
      MatrixXd K = K_trans.transpose();
      VectorXd Dz = z_bar - z;
      Dz(8) = my_mod(Dz(8) + PI, 2*PI) - PI;

      states_mu_ = xp + K*Dz;
      states_sigma_ = (MatrixXd::Identity(9,9) - K*H)*Pp;

      // publish current estimation
      auto estimated_state = interfaces::msg::States();
      estimated_state.x = states_mu_(0);
      estimated_state.y = states_mu_(1);
      estimated_state.z = states_mu_(2);
      estimated_state.vx = states_mu_(3);
      estimated_state.vy = states_mu_(4);
      estimated_state.vz = states_mu_(5);
      estimated_state.roll = states_mu_(6);
      estimated_state.pitch = states_mu_(7);
      estimated_state.yaw = my_mod(states_mu_(8) + PI, 2*PI) - PI;
      states_pub_->publish(estimated_state);

      RCLCPP_INFO(this->get_logger(), "I am at: (%.3f, %.3f, %.3f), heading: %.3f",
                                       states_mu_(0), states_mu_(1), states_mu_(2), estimated_state.yaw);
      
      ///////// Extended Kalman Filter Ends
    }

    double my_mod(double x, double y)
    {
        return x - floor(x/y)*y;
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

    // publisher
    rclcpp::Publisher<interfaces::msg::States>::SharedPtr states_pub_;

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