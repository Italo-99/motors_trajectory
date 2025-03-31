#ifndef GRIPPER_H
#define GRIPPER_H

#include "motors_trajectory/Motor.h"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/bool.hpp"

class Gripper : public rclcpp::Node
{
public:
    Gripper(MotorParams params, std::string node_name);
    ~Gripper();

    // ROS spinner + update gripper position
    void gripperSpinner();

private:

    // Open/close gripper service
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gripper_control_srv_;
    bool moveGripperCallback(const std_srvs::srv::SetBool::Request::SharedPtr& req,
                             std_srvs::srv::SetBool::Response::SharedPtr& res);

    // Grab/detach gripper service
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gripper_grab_srv_;
    bool grabbingGripperCallback(const std_srvs::srv::SetBool::Request::SharedPtr&  req,
                                 std_srvs::srv::SetBool::Response::SharedPtr& res);

    // Subscriber of object found in the middle of robot tips
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obj_found_sub_;
    void objFoundCallback(const std_msgs::msg::Bool::SharedPtr& msg);

    // Grabbing and detach object at the gripper: pipeline implementation
    void grabObj();
    void detachObj();

    // Publisher to attach/detach an object at the gripper
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr attach_obj_pub_;

    // Class attributes
    std::string         group_name_;
    std::string         joint_name_;
    double              ctrl_rate_;

    bool                sim_;
    bool                obj_gripper_found_;
    bool                grab_obj_cmd_;

    std::shared_ptr<MotorMover> gripper_mover_;
    std::string                 node_name_;

    rclcpp::TimerBase::SharedPtr mainloop_timer_;
    rclcpp::executors::SingleThreadedExecutor executor_;

    MotorParams params_;

    void declareParameters();
    MotorParams getMotorParams();
};

#endif // GRIPPER_H