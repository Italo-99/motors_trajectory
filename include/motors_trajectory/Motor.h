#ifndef MOTOR_MOVER_H
#define MOTOR_MOVER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

struct MotorParams {
    std::string     group_name =     "manipulator";
    std::string     joint_name =     "joint1";
    double          upper_limit =    6.28; 
    double          lower_limit =   -6.28;
    double          vel_limit =      1.0;
    double          acc_limit =      5.0;
    double          ctrl_rate =      500;        //Frequency of the control loop (Hz)
    double          tolerance =      0.001;      //Tolerance for setpoint reached
    double          min_vel =        0.1;        //Minimum velocity to be achieved during motion
    double          min_vel_region = 0.0;       //Percentage of path where the motor will proceed at min vel until it reaches the target
};

class MotorMover : public rclcpp::Node
{
    //NOTE: Position is always expressed in RADIANS
    public:
        MotorMover(MotorParams& motor_params);

        void spinner();

        //Stop the motor by setting the target position to the current position
        void stop();

        //Change the target position of the motor
        void setTargetPos(double target_pos);

        //Get the current position of the motor
        double getCurrentPos();

        //Get the current target position of the motor
        double getTargetPos();

        //True if current pos is within tolerance of target pos
        bool targetReached(); 

    private:

        void motorPosUpdate();
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr& js);
        void publishFakeMove(double current_pos, double current_vel);
        void moveMotorCallback(const std_msgs::msg::Float64::SharedPtr& msg);

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_; //Get current position and velocity for feedback-loop control
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motor_control_sub_; //Update the setpoint

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr fake_move_pub_; //Publish the setpoint to the fake controller

        // Class attributes
        MotorParams params_;
    
        // Class variables 
        double target_pos_; 
        double current_pos_;
        double current_vel_;
        double current_target_;     //Position to be reached at next control cycle
        double target_vel_;         //Velocity to be achieved at next control cycle
        double min_vel_distance_;   //Distance to be covered at min vel
        double ctrl_time_;
        bool   st_joint_sub_;
        int    motor_index_; // Index of the motor in the joint state message, -1 if waiting for initial joint state
        bool   target_reached_;

        double getDistance(); //Get absolute distance between current and target position
};

#endif // MOTOR_MOVER_H