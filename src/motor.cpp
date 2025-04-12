#include "motors_trajectory/Motor.h"

MotorMover::MotorMover(MotorParams& params) 
                       : rclcpp::Node(params.joint_name + "_motor_mover"),
                       params_(params),
                       current_vel_(0),
                       motor_index_(-1),
                       target_reached_(true)
{
    declareParameters();

    params_.group_name =        this->get_parameter("group_name").as_string();
    params_.joint_name =        this->get_parameter("joint_name").as_string();
    params_.upper_limit =       this->get_parameter("upper_limit").as_double();
    params_.lower_limit =       this->get_parameter("lower_limit").as_double();
    params_.vel_limit =         this->get_parameter("vel_limit").as_double();
    params_.acc_limit =         this->get_parameter("acc_limit").as_double();
    params_.ctrl_rate =         this->get_parameter("ctrl_rate").as_int();
    params_.tolerance =         this->get_parameter("tolerance").as_double();
    params_.min_vel =           this->get_parameter("min_vel").as_double();
    params_.min_vel_region =    this->get_parameter("min_vel_region").as_double();

    ctrl_time_ = 1.0 / params_.ctrl_rate;

    // Subscriber to joint state
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

    change_params_srv_ = this->create_service<motors_trajectory::srv::MotorParams>(
        params_.joint_name + "/change_motor_params",
        [this](const std::shared_ptr<motors_trajectory::srv::MotorParams::Request> request,
               std::shared_ptr<motors_trajectory::srv::MotorParams::Response> response) {
            motorParamsCallback(request, response);
        }
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
        current_pos_ = current_target_;
        current_vel_ = target_vel_; //Use the last computed velocity as current velocity
    }
}

void MotorMover::motorParamsCallback(const motors_trajectory::srv::MotorParams::Request::SharedPtr request,
                                     motors_trajectory::srv::MotorParams::Response::SharedPtr response)
{
    params_.lower_limit = request->lower_limit;
    params_.upper_limit = request->upper_limit;
    params_.vel_limit = request->vel_limit;
    params_.acc_limit = request->acc_limit;
    params_.ctrl_rate = request->ctrl_rate;
    params_.tolerance = request->tolerance;
    params_.min_vel = request->min_vel;
    params_.min_vel_region = request->min_vel_region;

    ctrl_time_ = 1/params_.ctrl_rate;
    
    response->success = true;
}

// Update current motor position
void MotorMover::motorPosUpdate()
{
    if (target_reached_)
    {
        target_reached_ = (getDistance() < params_.tolerance);
        //Check we are still at the target position
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
            //Proceed at minimum velocity
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

void MotorMover::setTargetPosPercentage(double perc)
{
    setTargetPos(params_.lower_limit + perc / 100.0 * (params_.upper_limit - params_.lower_limit));
}

// Getter of current position of the motor
double MotorMover::getCurrentPos() const
{
    return current_pos_;
}

//Getter of current setpoint of the motor
double MotorMover::getTargetPos() const
{
    return target_pos_;
}

bool MotorMover::targetReached() const
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
    if (current_pos < params_.lower_limit) {
        current_pos = params_.lower_limit;
    }
    else if (current_pos > params_.upper_limit) {
        current_pos = params_.upper_limit;
    }

    sensor_msgs::msg::JointState joint_state_msg;

    joint_state_msg.header.stamp = this->now();
    joint_state_msg.name.push_back(params_.joint_name);
    joint_state_msg.position.push_back(current_pos);
    joint_state_msg.velocity.push_back(current_vel);
    fake_move_pub_->publish(joint_state_msg);
}

void MotorMover::declareParameters()
{
    this->declare_parameter("group_name", params_.group_name);
    this->declare_parameter("joint_name", params_.joint_name);
    this->declare_parameter("upper_limit", params_.upper_limit);
    this->declare_parameter("lower_limit", params_.lower_limit);
    this->declare_parameter("vel_limit", params_.vel_limit);
    this->declare_parameter("acc_limit", params_.acc_limit);
    this->declare_parameter("ctrl_rate", params_.ctrl_rate);
    this->declare_parameter("tolerance", params_.tolerance);
    this->declare_parameter("min_vel", params_.min_vel);
    this->declare_parameter("min_vel_region", params_.min_vel_region);
}

// Spinner ROS + motor update
void MotorMover::spinner()
{
    executor_.add_node(get_node_base_interface());

    mainloop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(ctrl_time_ * 1000)),
        [this]() {
            motorPosUpdate();
        }
    );

    executor_.spin();
}

double MotorMover::getDistance() const {
    return abs(target_pos_ - current_pos_);
}