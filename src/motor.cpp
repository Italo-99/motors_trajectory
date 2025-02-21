#include "motors_trajectory/Motor.h"

MotorMover::MotorMover(MotorParams& params) 
                       : rclcpp::Node(params_.joint_name + "_motor_mover"),
                       params_(params),
                       current_vel_(0),
                       ctrl_time_(1/params_.ctrl_rate),
                       target_reached_(true)
{
    // Subscriber to joint state
    motor_index_     = -1;
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 1, 
        [this](const sensor_msgs::msg::JointState::SharedPtr js) {
            jointStateCallback(js);
        }
    );

    // Subscriber to user commands
    motor_control_sub_ = this->create_subscription<std_msgs::msg::Float64>(
        params_.group_name + "/motor_control", 1, 
        [this](const std_msgs::msg::Float64::SharedPtr msg) {
            moveMotorCallback(msg);
        }
    );

    // Send command to move group fake controller
    fake_move_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "move_group/fake_controller_joint_states", 1
    );


    RCLCPP_INFO(get_logger(), "%s motor of group %s control sampling time set at %f s",
                params_.joint_name.c_str(), params_.group_name.c_str(), ctrl_time_);
}

void MotorMover::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr& js)
{
    if (motor_index_ == -1)
    {
        // Don't execute this if function next time
        RCLCPP_INFO(get_logger(), "Searching joint << %s >> among active.",params_.joint_name.c_str());

        for (unsigned int k = 0; k < js->name.size(); k++)
        {            
            if (js->name[k] == params_.joint_name)
            {
                RCLCPP_INFO(get_logger(), "Joint << %s >> found among active joints.",params_.joint_name.c_str());
                motor_index_ = k;
                current_pos_ = js->position[k];
                break;
            }
            else if (k == js->name.size() - 1)
            {
                RCLCPP_ERROR(get_logger(), "Joint << %s >> not found among active joints. Exiting ...",params_.joint_name.c_str());
                rclcpp::shutdown();
            }
        }
        
        // Keep the motor stationary at the beginning
        target_pos_     = current_pos_;
        current_target_ = target_pos_;
        current_vel_ = 0;
    }
    else
    {
        current_pos_ = js->position[motor_index_];
        current_vel_ = target_vel_; //Use the last computed velocity as current velocity
    }
}

// Update current motor position
void MotorMover::motorPosUpdate()
{
    if (target_reached_)
    {
        //Check we are still at the target position
        target_reached_ = getDistance() < params_.tolerance;
        return;
    }

    if (getDistance() < params_.tolerance)
    {
        //Position has been reached, send stop command
        RCLCPP_INFO(get_logger(), "Target position reached: %f", target_pos_);
        target_reached_ = true;
        target_vel_ = 0.;
        current_target_ = current_pos_;
        publishFakeMove(current_target_, target_vel_);
        return;
    }

    if (!target_reached_)
    {
        // If target pos is greater than current pos, pos must increase otherwise it should decrease
        short sign = (target_pos_ > current_pos_) ? 1 : -1;

        if (getDistance() < (pow(params_.vel_limit, 2) - pow(params_.min_vel, 2)) / (2 * params_.acc_limit) + min_vel_distance_) //If withing breaking distance of target
        {
            RCLCPP_INFO_ONCE(get_logger(), "Br: %f, D: %f", (pow(params_.vel_limit, 2) - pow(params_.min_vel, 2)) / (2 * params_.acc_limit), min_vel_distance_);
            if (abs(current_vel_) > params_.min_vel) //If velocity is above minimum
            {
                target_vel_ = current_vel_ - sign * params_.acc_limit * ctrl_time_;  //Decrease velocity
                if (abs(target_vel_) < params_.min_vel) //If velocity is below minimum, set it to minimum
                {
                    target_vel_ = sign * params_.min_vel;
                }
            } else { //If velocity is below minimum
                target_vel_ = current_vel_ + sign * params_.acc_limit * ctrl_time_;  //Increase velocity
                if (abs(target_vel_) > params_.min_vel) //If velocity is above minimum, set it to minimum
                {
                    target_vel_ = sign * params_.min_vel;
                }
            }
        }
        else //We are far enough from the target so we can try to reach max velocity
        {
            if (abs(current_vel_) < params_.vel_limit) //If velocity is below max
            {
                if (abs(current_vel_) < params_.min_vel){ //If either stationary or very slow
                    target_vel_ = sign * params_.min_vel;
                } else { //Accelerate
                    target_vel_ = current_vel_ + sign * params_.acc_limit * ctrl_time_;
                }

                if (abs(target_vel_) > params_.vel_limit) //If velocity is above max, set it to max
                {
                    target_vel_ = sign * params_.vel_limit;
                }
            }
            else { //If velocity is above max
                target_vel_ = current_vel_ - sign * params_.acc_limit * ctrl_time_; //Decelerate

                if (abs(target_vel_) < params_.vel_limit) //If velocity is below max, set it to max
                {
                    target_vel_ = sign * params_.vel_limit;
                }
            }
        }

        
        current_target_ = current_target_ + target_vel_ * ctrl_time_;
        //RCLCPP_INFO(get_logger(), "Current position: %f, Current velocity: %f, Target position: %f, Target velocity: %f", current_pos_, current_vel_, current_target_, target_vel_);

        // Publish computed current motor target
        publishFakeMove(current_target_, target_vel_);  
    }
}

// Update current target pose
void MotorMover::moveMotorCallback(const std_msgs::msg::Float64::SharedPtr& msg) 
{
    RCLCPP_INFO(get_logger(), "Received new target position: %f", msg->data);
    setTargetPos(msg->data);
}

// Setter of target pose for child class
void MotorMover::setTargetPos(double target_pos)
{
    //Cap the target position to imposed position limits
    if(target_pos_ < params_.lower_limit) {
        target_pos_ = params_.lower_limit;
    }
    else if(target_pos_ > params_.upper_limit) {
        target_pos_ = params_.upper_limit;
    }
    else {
        target_pos_ = target_pos;
    }

    min_vel_distance_ = abs(target_pos_ - current_pos_) * params_.min_vel_region;
}

// Getter of current position of the motor
double MotorMover::getCurrentPos()
{
    return current_pos_;
}

//Getter of current setpoint of the motor
double MotorMover::getTargetPos()
{
    return target_pos_;
}

bool MotorMover::targetReached()
{
    return target_reached_;
}

void MotorMover::stop()
{
    target_pos_ = current_pos_;
}

// Fake controller publisher to move group 
void MotorMover::publishFakeMove(double current_pos,double current_vel)
{
    sensor_msgs::msg::JointState joint_state_msg;

    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name.push_back(params_.joint_name);
    joint_state_msg.position.push_back(current_pos);
    joint_state_msg.velocity.push_back(current_vel);
    fake_move_pub_->publish(joint_state_msg);
}

// Spinner ROS + motor update
void MotorMover::spinner()
{
    rclcpp::Rate loop_rate(params_.ctrl_rate);
    while (rclcpp::ok())
    {
        motorPosUpdate();
        rclcpp::spin_some(shared_from_this());
        loop_rate.sleep();
    }
}

double MotorMover::getDistance(){
    return abs(target_pos_ - current_pos_);
}