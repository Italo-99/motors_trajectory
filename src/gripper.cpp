#include "motors_trajectory/Gripper.h"

Gripper::Gripper(MotorParams params, std::string node_name) : rclcpp::Node(node_name), node_name_(node_name), params_(params)
{    
    declareParameters();

    joint_name_ = this->get_parameter("joint_name").as_string();
    group_name_ = this->get_parameter("group_name").as_string();
    ctrl_rate_ = this->get_parameter("ctrl_rate").as_int();

    sim_ = this->get_parameter("sim").as_bool();

    // Declare service open/close gripper
    gripper_control_srv_ = this->create_service<std_srvs::srv::SetBool>(
        group_name_ + "/move_gripper",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
               std_srvs::srv::SetBool::Response::SharedPtr res) {
            moveGripperCallback(req, res);
        }
    );

    // Declare service grab/detach an object to the gripper
    gripper_grab_srv_ = this->create_service<std_srvs::srv::SetBool>(
        group_name_ + "/grabbing_gripper",
        [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
               std_srvs::srv::SetBool::Response::SharedPtr res) {
            grabbingGripperCallback(req, res);
        }
    );

    // Build a subscriber which receives if an object is within ee tips
    obj_found_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/obj_gripper_found", 1,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            objFoundCallback(msg);
        }
    );

    // Build a publisher to send command to attach/detach an object at the gripper
    attach_obj_pub_ = this->create_publisher<std_msgs::msg::Bool>("/attach_obj_to_gripper", 1);

    // Initialize commands
    grab_obj_cmd_       = false;
    obj_gripper_found_  = false;
}

Gripper::~Gripper(){gripper_mover_.reset();}

bool Gripper::moveGripperCallback(const std_srvs::srv::SetBool::Request::SharedPtr& req,
                                  std_srvs::srv::SetBool::Response::SharedPtr& res)
{
    double target_pos = req->data ? 100. : 0.;
    gripper_mover_->setTargetPosPercentage(target_pos);
    res->success = true;
    res->message = req->data ? "Gripper closed" : "Gripper opened";
    return true;
}

bool Gripper::grabbingGripperCallback(const std_srvs::srv::SetBool::Request::SharedPtr& req,
                                      std_srvs::srv::SetBool::Response::SharedPtr& res)
{
    if (req->data)   {grabObj();}
    else            {detachObj();}
    res->success = true;
    res->message = req->data ? "Sent comand to grab the object at the gripper" : "Sent comand to detach the object from the gripper";
    return true;
}

void Gripper::grabObj()
{
    grab_obj_cmd_ = true;

    // Check if the object has been detected
    if (!obj_gripper_found_)
    {
        // Close the gripper
        gripper_mover_->setTargetPosPercentage(100.0);
    }
    else 
    {
        // Stop closing and attach the object
        if (sim_) {gripper_mover_->stop();}
        // else TODO: implement this function based on real position encoding
        std_msgs::msg::Bool msg;
        msg.data = true;
        attach_obj_pub_->publish(msg);
    }
}

void Gripper::detachObj()
{
    // Detach the object and open the gripper
    grab_obj_cmd_ = false;
    std_msgs::msg::Bool msg;
    msg.data = false;
    attach_obj_pub_->publish(msg);
    gripper_mover_->setTargetPosPercentage(0.0);
}

void Gripper::objFoundCallback(const std_msgs::msg::Bool::SharedPtr& msg)
{
    obj_gripper_found_ = msg->data;
    if (grab_obj_cmd_ && obj_gripper_found_) {grabObj();}
}

void Gripper::declareParameters(){

    this->declare_parameter("sim", true);

    //Motor params
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

MotorParams Gripper::getMotorParams(){
    MotorParams params;
    params.group_name =        this->get_parameter("group_name").as_string();
    params.joint_name =        this->get_parameter("joint_name").as_string();
    params.upper_limit =       this->get_parameter("upper_limit").as_double();
    params.lower_limit =       this->get_parameter("lower_limit").as_double();
    params.vel_limit =         this->get_parameter("vel_limit").as_double();
    params.acc_limit =         this->get_parameter("acc_limit").as_double();
    params.ctrl_rate =         this->get_parameter("ctrl_rate").as_int();
    params.tolerance =         this->get_parameter("tolerance").as_double();
    params.min_vel =           this->get_parameter("min_vel").as_double();
    params.min_vel_region =    this->get_parameter("min_vel_region").as_double();
    return params;
}

void Gripper::gripperSpinner()
{
    // Set spinner rate
    rclcpp::Rate rate(ctrl_rate_);

    MotorParams params = getMotorParams();
    RCLCPP_INFO(this->get_logger(), "Gripper node initialized for joint: %s", params.joint_name.c_str());
    gripper_mover_ = std::make_shared<MotorMover>(params);

    mainloop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1./ctrl_rate_ * 1000)),
        [this]() {
            gripper_mover_->motorPosUpdate();
        }
    );


    executor_.add_node(this->get_node_base_interface());
    executor_.add_node(gripper_mover_->get_node_base_interface());
    
    executor_.spin();
}