#include "robotiq85_gripper/motor_planner.h"

MotorPlanner::MotorPlanner(const std::string& group_name)
    : move_group_(group_name) {
}

bool MotorPlanner::moveFingerJoint(double target_position) {
    std::vector<double> joint_values;
    joint_values = move_group_.getCurrentJointValues();
    ROS_INFO_ONCE("Finger joint value: %f", joint_values[0]);

    // Assuming finger_joint is the last joint in the joint_values array
    if (joint_values.size() > 0) {
        joint_values[0] = target_position; // Update finger_joint position
        move_group_.setJointValueTarget(joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            move_group_.move();
            return true;
        } else {
            ROS_ERROR("Failed to plan motion to target position.");
            return false;
        }
    } else {
        ROS_ERROR("Failed to get current joint values.");
        return false;
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "gripper_plan_publisher");
    ros::NodeHandle nh;

    MotorPlanner m("gripper");

    m.moveFingerJoint(0.5);

    return 0;
}


// int main(int argc, char** argv) {
    //     // Initialize the ROS node
    //     ros::init(argc, argv, "gripper_state_publisher");
    //     ros::NodeHandle nh;

    //     // Create a publisher for the joint states
    //     ros::Publisher joint_states_pub = nh.advertise<sensor_msgs::JointState>(
    //                             "/move_group/fake_controller_joint_states", 1);

    //     // Create a joint state message
    //     sensor_msgs::JointState joint_state_msg;
    //     joint_state_msg.header.stamp = ros::Time::now();
    //     joint_state_msg.name.resize(1);
    //     joint_state_msg.position.resize(1);

    //     // Set the joint names
    //     joint_state_msg.name = "finger_joint";

    //     // Set the initial joint positions (all set to 0 for demonstration)
    //     for (size_t i = 0; i < joint_state_msg.position.size(); ++i) {
    //         joint_state_msg.position[i] = 0.0;
    //     }

    //     joint_state_msg.position[0] = 0.50;

    //     // Publish the joint states periodically
    //     ros::Rate rate(10);  // Publish at 10 Hz
    //     while (ros::ok()) {
    //         joint_state_msg.header.stamp = ros::Time::now(); // Update timestamp
    //         joint_states_pub.publish(joint_state_msg);
    //         ros::spinOnce();
    //         rate.sleep();
    //     }

    //     return 0;
    // }

