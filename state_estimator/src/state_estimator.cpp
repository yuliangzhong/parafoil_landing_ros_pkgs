#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"

#include <eigen3/Eigen/Dense>
#include <cmath>
#include <iostream>
using std::cout; using std::endl;

using namespace std::chrono_literals;
using std::placeholders::_1;

using Vector3Stamped = geometry_msgs::msg::Vector3Stamped;
using MatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

#define PI 3.14159265358979323846

double Vh_guess = 4.0; // [m/s]
double Vz_guess = 2.0; // [m/s]
double ang_vel_accu = 1 /180*PI; // [rad/s]

class StateEstimator : public rclcpp::Node
{
  public:
    StateEstimator() : Node("state_estimator"), Ts(0.1)
    {
        pos_pub = this->create_publisher<Vector3Stamped>("estimated_pos", 1);
        vel_pub = this->create_publisher<Vector3Stamped>("estimated_vel", 1);
        timer_ = this->create_wall_timer(100ms, std::bind(&StateEstimator::ekf_callback, this));

        pos_sub = this->create_subscription<Vector3Stamped>("position", 1, std::bind(&StateEstimator::pos_callback, this, _1));
        body_ang_vel_sub = this->create_subscription<Vector3Stamped>(
                                             "body_ang_vel", 1, std::bind(&StateEstimator::body_ang_vel_callback, this, _1));
      
        states_mu = VectorXd::Zero(6);
        states_sigma = MatrixXd::Zero(6,6);
        states_sigma.block(0,0,3,3) = 4*MatrixXd::Identity(3,3);
        states_sigma(3,3) = 1;
        states_sigma(4,4) = 0.5;
        states_sigma(5,5) = 0.5;

        // Q = pow(ang_vel_accu,2);
        Q = 0.5;
        R = 0.5*MatrixXd::Identity(5,5);
    }

  private:
    void ekf_callback()
    {
        if(pos_update_flag == false or ang_update_flag == false) {return;}
        
        printf("position_last: %.3f, %.3f, %.3f, ", pos_last.vector.x, pos_last.vector.y, pos_last.vector.z);
        printf("position: %.3f, %.3f, %.3f, ", pos_now.vector.x, pos_now.vector.y, pos_now.vector.z);
        printf("body_ang_vel: %.3f, %.3f, %.3f \n ", body_ang_vel_now.vector.x, body_ang_vel_now.vector.y, body_ang_vel_now.vector.z);

        cout<<"states_mu\n"<<states_mu<<endl;
        cout<<"states_sigma\n"<<states_sigma<<endl;
        cout<<"Q\n"<<Q<<endl;
        cout<<"R\n"<<R<<endl;
        ///// EKF starts /////
        // state X = [x, y, z, yaw, Vh, Vz]^T 6*1
        // control input U = wz 1*1
        // dynamics disturbance D = dwz 1*1
        // sensor observation Zs = [I_r_IB; dx; dy] 5*1
        // sensor error E = [e_pos; edx; edy] 5*1
        double wz = body_ang_vel_now.vector.z; // approx
        cout<<"wz\n"<<wz<<endl;
        ///// Prior update /////
        // [xp, A, S] = Dyn(Ts, state_mu, U, 0);
        // Pp = A*state_sigma*A' + S*Q*S';
        VectorXd Xp(6);
        Xp << states_mu(0) + Ts*states_mu(4)*cos(states_mu(3)),
              states_mu(1) + Ts*states_mu(4)*sin(states_mu(3)),
              states_mu(2) + Ts*states_mu(5),
              states_mu(3) + Ts*wz,
              states_mu(4),
              states_mu(5);
        // X_dot = zeros(6,1);
        // X_dot(1) = X(1) + Ts*X(5)*cos(X(4));
        // X_dot(2) = X(2) + Ts*X(5)*sin(X(4));
        // X_dot(3) = X(3) + Ts*X(6);
        // X_dot(4) = X(4) + Ts*(U+D);
        // X_dot(5) = X(5);
        // X_dot(6) = X(6);
        cout<<"xp\n"<<Xp<<endl;
        
        MatrixXd A(6,6);
        A << 1, 0, 0, -Ts*states_mu(4)*sin(states_mu(3)), Ts*cos(states_mu(3)), 0,
             0, 1, 0,  Ts*states_mu(4)*cos(states_mu(3)), Ts*sin(states_mu(3)), 0,
             0, 0, 1, 0, 0, Ts,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1;
        
        cout<<"A\n"<<A<<endl;

        // A = zeros(6,6);
        // A(1,:) = [1, 0, 0, -Ts*X(5)*sin(X(4)), Ts*cos(X(4)), 0];
        // A(2,:) = [0, 1, 0, Ts*X(5)*cos(X(4)), Ts*sin(X(4)), 0];
        // A(3,:) = [0, 0, 1, 0, 0, Ts];
        // A(4,:) = [0, 0, 0, 1, 0, 0];
        // A(5,:) = [0, 0, 0, 0, 1, 0];
        // A(6,:) = [0, 0, 0, 0, 0, 1];
        
        VectorXd S(6);
        S << 0, 0, 0, Ts, 0, 0;
        // S = [0; 0; 0; Ts; 0; 0];
        
        cout<<"S\n"<<S<<endl;

        
        MatrixXd Pp = A*states_sigma*A.transpose() + S*Q*S.transpose();
        cout<<"Pp\n"<<Pp<<endl;

        ///// Measurement update /////
        // z_bar = [I_r_IB; dx; dy];
        // [z, H, M] = Obs(N*Ts, xp, zeros(5,1));
        double px = pos_now.vector.x;
        double py = pos_now.vector.y;
        double pz = pos_now.vector.z;
        double dx = px - pos_last.vector.x;
        double dy = py - pos_last.vector.y;
        VectorXd z_bar(5);
        z_bar << px, py, pz, dx, dy;
        cout<<"zbar\n"<<z_bar<<endl;

        VectorXd Zs(5);
        Zs << Xp(0), Xp(1), Xp(2), Ts*Xp(4)*cos(Xp(3)), Ts*Xp(4)*sin(Xp(3));
        // z = zeros(5,1);
        // z(1:3) = X(1:3) + E(1:3);
        // z(4) = T*X(5)*cos(X(4)) + E(4);
        // z(5) = T*X(5)*sin(X(4)) + E(5);
        cout<<"Zs\n"<<Zs<<endl;

        MatrixXd H = MatrixXd::Zero(5,6);
        H.block(0,0,3,3).setIdentity();
        H(3,3) = -Ts*Xp(4)*sin(Xp(3));
        H(3,4) = Ts*cos(Xp(3));
        H(4,3) = Ts*Xp(4)*cos(Xp(3));
        H(4,4) = Ts*sin(Xp(3));
        // H = zeros(5,6);
        // H(1:3,:) = [eye(3), zeros(3,3)];
        // H(4,:) = [0, 0, 0, -T*X(5)*sin(X(4)), T*cos(X(4)),0];
        // H(5,:) = [0, 0, 0, T*X(5)*cos(X(4)), T*sin(X(4)),0];
        cout<<"H\n"<<H<<endl;

        MatrixXd M = MatrixXd::Identity(5,5);
        // M = eye(5);
        cout<<"M\n"<<M<<endl;

        MatrixXd K_trans = (H*Pp*H.transpose() + M*R*M.transpose()).transpose().ldlt().solve(H*Pp.transpose());
        MatrixXd K = K_trans.transpose();
        cout<<"K\n"<<K<<endl;

        VectorXd Dz = z_bar - Zs;
        cout<<"Dz\n"<<Dz<<endl;

        // K = Pp*H'/(H*Pp*H' + M*R*M');
        // Dz = z_bar - z;

        states_mu = Xp + K*Dz;
        states_sigma = (MatrixXd::Identity(6,6) - K*H)*Pp;
        cout<<"states_mu\n"<<states_mu<<endl;
        cout<<"states_sigma\n"<<states_sigma<<endl;
        // state_mu = xp + K*Dz;
        // state_sigma = (eye(6) - K*H)*Pp;

        ///// EKF ends   /////

        // publish results
        auto pos_msg = Vector3Stamped();
        pos_msg.vector.x = states_mu(0);
        pos_msg.vector.y = states_mu(1);
        pos_msg.vector.z = states_mu(2);
        auto vel_msg = Vector3Stamped();
        vel_msg.vector.x = atan2(sin(states_mu(3)), cos(states_mu(3))); // warp yaw output to [-pi,pi)
        vel_msg.vector.y = states_mu(4);
        vel_msg.vector.z = states_mu(5);

        printf("pos_msg: %.3f, %.3f, %.3f, ", pos_msg.vector.x, pos_msg.vector.y, pos_msg.vector.z);
        printf("vel_msg: %.3f, %.3f, %.3f, ", vel_msg.vector.x, vel_msg.vector.y, vel_msg.vector.z);

        pos_msg.header.stamp = this->get_clock()->now();
        vel_msg.header.stamp = pos_msg.header.stamp;

        pos_pub->publish(pos_msg);
        vel_pub->publish(vel_msg);
        
        // close flag
        pos_update_flag = false;
        ang_update_flag = false;

        // printf("pos: (%.3f, %.3f, %.3f), heading: %.3f \n", pos_msg.vector.x, pos_msg.vector.y, pos_msg.vector.z, vel_msg.vector.x);
    }

    void pos_callback(const Vector3Stamped & msg)
    {
        if(pos_update_flag == false)
        {
            // init EKF
            states_mu(0) = msg.vector.x;
            states_mu(1) = msg.vector.y;
            states_mu(2) = msg.vector.z;
            states_mu(3) = 0.0;
            states_mu(4) = Vh_guess;
            states_mu(5) = Vz_guess;
            
            pos_now = msg;
            pos_update_flag = true;

        }
        else
        {
            pos_last = pos_now;
            pos_now = msg;
            pos_update_flag = true;
        }
    }

    void body_ang_vel_callback(const Vector3Stamped & msg)
    {
        body_ang_vel_now = msg;
        ang_update_flag = true;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Vector3Stamped>::SharedPtr pos_pub;
    rclcpp::Publisher<Vector3Stamped>::SharedPtr vel_pub;
    
    rclcpp::Subscription<Vector3Stamped>::SharedPtr pos_sub;
    rclcpp::Subscription<Vector3Stamped>::SharedPtr body_ang_vel_sub;

    // data storage
    Vector3Stamped pos_last;
    Vector3Stamped pos_now;
    Vector3Stamped body_ang_vel_now;
   
    // EKF parameters
    double Ts;
    VectorXd states_mu ; // [x y z yaw Vh Vz] 6*1
    MatrixXd states_sigma; // 6*6
    double Q; // input noise 1*1
    MatrixXd R; // observation noise 5*5

    // EKF update_flag
    bool pos_update_flag = false;
    bool ang_update_flag = false;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimator>());
  rclcpp::shutdown();
  return 0;
}