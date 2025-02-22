#include "motors_trajectory/Motor.h"

int main(int argc, char* argv[]) {
    
    MotorParams params;
    params.group_name = "robotiq_85_gripper";
    params.joint_name = "robotiq_85_left_knuckle_joint";
    params.upper_limit = 0.8;
    params.lower_limit = 0.0;
    params.vel_limit = 1.0;
    params.acc_limit = 2.0;
    params.ctrl_rate = 500;
    params.min_vel = 0.1;
    params.min_vel_region = 0.02;
    
    rclcpp::init(argc, argv);

    std::shared_ptr<MotorMover> motor_mover = std::make_shared<MotorMover>(params);
    motor_mover->spinner();

    rclcpp::shutdown();

    return 0;
}