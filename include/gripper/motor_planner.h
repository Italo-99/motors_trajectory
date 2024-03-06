#ifndef MOTOR_PLANNER_H
#define MOTOR_PLANNER_H

#include <moveit/move_group_interface/move_group_interface.h>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

class MotorPlanner{
public:
    MotorPlanner(const std::string& group_name);

    bool moveFingerJoint(double target_position);

private:
    moveit::planning_interface::MoveGroupInterface move_group_;
};

#endif // MOTOR_PLANNER_H
