#ifndef MOTOR_MOVER_H
#define MOTOR_MOVER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"
#include "motors_trajectory/srv/motor_params.hpp"

struct MotorParams {
    std::string     group_name =     "manipulator";
    std::string     joint_name =     "joint1";
    double          upper_limit =    6.28; 
    double          lower_limit =   -6.28;
    double          vel_limit =      1.0;
    double          acc_limit =      5.0;
    int             ctrl_rate =      500;        //Frequency of the control loop (Hz)
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
        void spinOnce();

        //Stop the motor by setting the target position to the current position
        void stop();

        //Change the target position of the motor
        void setTargetPos(double target_pos);
        void setTargetPosPercentage(double perc); //Value between 0 and 100

        //Get the current position of the motor
        double getCurrentPos() const;

        //Get the current target position of the motor
        double getTargetPos() const;

        //True if current pos is within tolerance of target pos
        bool targetReached() const; 

        void motorPosUpdate(); //Update the motor position and velocity based on the current target and status
    
        private:

        void declareParameters(); //Declare the parameters of the motor
        
        void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr& js);
        void publishFakeMove(double current_pos, double current_vel);
        void moveMotorCallback(const std_msgs::msg::Float64::SharedPtr& msg);
        void motorParamsCallback(const motors_trajectory::srv::MotorParams::Request::SharedPtr request,
                                 motors_trajectory::srv::MotorParams::Response::SharedPtr response);

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_; //Get current position and velocity for feedback-loop control
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr motor_control_sub_; //Update the setpoint

        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr fake_move_pub_; //Publish the setpoint to the fake controller

        rclcpp::Service<motors_trajectory::srv::MotorParams>::SharedPtr change_params_srv_; //Service to change the motor parameters

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

        rclcpp::TimerBase::SharedPtr mainloop_timer_; //Timer to call the motorPosUpdate function at a fixed rate
        rclcpp::executors::SingleThreadedExecutor executor_; //Executor to run the node

        double getDistance() const; //Get absolute distance between current and target position
};

#endif // MOTOR_MOVER_H