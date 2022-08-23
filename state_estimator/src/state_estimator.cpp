#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "std_msgs/msg/int32.hpp"

#include <eigen3/Eigen/Dense>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

using Vector3Stamped = geometry_msgs::msg::Vector3Stamped;
using Int32 = std_msgs::msg::Int32;
using MatrixXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
using VectorXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

#define PI 3.14159265358979323846

double Vh_guess = 0.46; // [m/s]
double Vz_guess = 1.31; // [m/s]
double ang_vel_accu = 10 /180*PI; // [rad/s]

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
        
        ekf_switch_sub = this->create_subscription<Int32>("ekf_switch", 1, std::bind(&StateEstimator::switch_callback, this, _1));

        states_mu = VectorXd::Zero(6);
        states_sigma = MatrixXd::Zero(6,6);
        states_sigma.block(0,0,3,3) = 4*MatrixXd::Identity(3,3);
        states_sigma(3,3) = 1;
        states_sigma(4,4) = 0.5;
        states_sigma(5,5) = 0.5;

        Q = pow(ang_vel_accu,2);
        R = 0.1*MatrixXd::Identity(5,5);
    }

  private:
    void ekf_callback()
    {
        if(pos_update_flag == false or ang_update_flag == false or ekf_switch_flag == false) {return;}

        ///// EKF starts /////
        // state X = [x, y, z, yaw, Vh, Vz]^T 6*1
        // control input U = wz 1*1
        // dynamics disturbance D = dwz 1*1
        // sensor observation Zs = [I_r_IB; dx; dy] 5*1
        // sensor error E = [e_pos; edx; edy] 5*1
        double wz = body_ang_vel_now.vector.z; // approx

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
        
        MatrixXd A(6,6);
        A << 1, 0, 0, -Ts*states_mu(4)*sin(states_mu(3)), Ts*cos(states_mu(3)), 0,
             0, 1, 0,  Ts*states_mu(4)*cos(states_mu(3)), Ts*sin(states_mu(3)), 0,
             0, 0, 1, 0, 0, Ts,
             0, 0, 0, 1, 0, 0,
             0, 0, 0, 0, 1, 0,
             0, 0, 0, 0, 0, 1;

        VectorXd S(6);
        S << 0, 0, 0, Ts, 0, 0;
        
        MatrixXd Pp = A*states_sigma*A.transpose() + S*Q*S.transpose();

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

        VectorXd Zs(5);
        Zs << Xp(0), Xp(1), Xp(2), Ts*Xp(4)*cos(Xp(3)), Ts*Xp(4)*sin(Xp(3));

        MatrixXd H = MatrixXd::Zero(5,6);
        H.block(0,0,3,3).setIdentity();
        H(3,3) = -Ts*Xp(4)*sin(Xp(3));
        H(3,4) = Ts*cos(Xp(3));
        H(4,3) = Ts*Xp(4)*cos(Xp(3));
        H(4,4) = Ts*sin(Xp(3));

        MatrixXd M = MatrixXd::Identity(5,5);

        MatrixXd K_trans = (H*Pp*H.transpose() + M*R*M.transpose()).transpose().ldlt().solve(H*Pp.transpose());
        MatrixXd K = K_trans.transpose();

        VectorXd Dz = z_bar - Zs;
        // K = Pp*H'/(H*Pp*H' + M*R*M');
        // Dz = z_bar - z;

        states_mu = Xp + K*Dz;
        states_sigma = (MatrixXd::Identity(6,6) - K*H)*Pp;
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

        pos_msg.header.stamp = this->get_clock()->now();
        vel_msg.header.stamp = pos_msg.header.stamp;

        pos_pub->publish(pos_msg);
        vel_pub->publish(vel_msg);
        
        // close flag
        pos_update_flag = false;
        ang_update_flag = false;

        printf("pos: (%.3f, %.3f, %.3f), heading: %.3f \n", pos_msg.vector.x, pos_msg.vector.y, pos_msg.vector.z, vel_msg.vector.x);
    }

    void pos_callback(const Vector3Stamped & msg)
    {
        if (ekf_init_flag == false)
        {
            // init EKF
            states_mu(0) = msg.vector.x;
            states_mu(1) = msg.vector.y;
            states_mu(2) = msg.vector.z;
            states_mu(3) = 0.0;
            states_mu(4) = Vh_guess;
            states_mu(5) = Vz_guess;
            
            pos_now = msg;
            ekf_init_flag = true;
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

    void switch_callback(const Int32 & msg)
    {
        if (msg.data == 0)
        {
            ekf_init_flag = false;
            ekf_switch_flag = false;
        }
        else
        {
            ekf_switch_flag = true;
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Vector3Stamped>::SharedPtr pos_pub;
    rclcpp::Publisher<Vector3Stamped>::SharedPtr vel_pub;
    
    rclcpp::Subscription<Vector3Stamped>::SharedPtr pos_sub;
    rclcpp::Subscription<Vector3Stamped>::SharedPtr body_ang_vel_sub;
    rclcpp::Subscription<Int32>::SharedPtr ekf_switch_sub;

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

    // EKF update flags
    bool ekf_init_flag = false;
    bool ekf_switch_flag = false;
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