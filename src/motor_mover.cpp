/* LICENSE
 * Software License Agreement (Apache Licence 2.0)
 *
 *  Copyright (c) [2024], [Andrea Pupa] [italo Almirante]
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in
 *      the documentation and/or other materials provided with the
 *      distribution.
 *   3. The name of the author may not be used to endorse or promote
 *      products derived from this software without specific prior
 *      written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: [Andrea Pupa] [Italo Almirante]
 *  Created on: [2024-01-17]
*/

#include "robotiq85_gripper/motor_mover.h"

MotorMover::MotorMover( std::string         motor_name,
                        std::string         joint_name,
                        std::vector<double> joint_limits,
                        double              vel_limit,
                        double              acc_limit,
                        double              ctrl_rate,
                        bool                inst_target)
    : nh_("~"), motor_name_(motor_name), joint_name_(joint_name),
                joint_limits_(joint_limits), vel_limit_(vel_limit),
                acc_limit_(acc_limit), ctrl_rate_(ctrl_rate),
                inst_target_(inst_target)
{
    // Subscriber to user commands
    gripper_control_sub_ = nh_.subscribe(motor_name_ + "/gripper_control",
                            1, &MotorMover::moveMotorCallback, this);
    // Send command to move group fake controller
    fake_move_pub_       = nh_.advertise<sensor_msgs::JointState>(
                            "/move_group/fake_controller_joint_states", 1);
    // Init class variables
    target_pos_     = 0;
    current_vel_    = 0;
    current_pos_    = joint_limits_[0];
    ctrl_time_      = 1/ctrl_rate_;
    time_acc_dec_   = vel_limit/acc_limit;

    ROS_INFO("%s motor control sampling time set at %f s",motor_name_.c_str(),ctrl_time_);
    ROS_INFO("Acceleration time set at %f s",time_acc_dec_);
}

// MotorMover::~MotorMover()
// {
//     // Insert destructor
//     return 0;
// }

// Update current motor position
void MotorMover::motorPosUpdate()
{
    // Check if instantaneous setpoint update mode has been set
    if (inst_target_)
    {
        current_pos_ = target_pos_;
        return;
    }

    // Check if target position has been reached, exit from the function
    if ((target_pos_-current_pos_<0.001) || (target_pos_-current_pos_>-0.001))
    {
        current_vel_ = 0;
        current_pos_ = target_pos_;
        return;
    }

    // If target pos is greater than current pos, pos must increase
    if (target_pos_ > current_pos_)
    {
        // If current pose is near the target (within 5 deg)
        if      (target_pos_ - current_pos_ < 0.087)
        {
            // Follow current velocity to 30% of its limit
            if      (current_vel_ < vel_limit_*0.3 - 0.001)
            {
                current_vel_ += acc_limit_*ctrl_time_;
                if (current_vel_ > vel_limit_*0.3)
                {
                    current_vel_ = vel_limit_*0.3;
                }
            }
            else if (current_vel_ > vel_limit_*0.3 + 0.001)
            {
                current_vel_ -= acc_limit_*ctrl_time_;
                if (current_vel_ < vel_limit_*0.3)
                {
                    current_vel_ = vel_limit_*0.3;
                }
            }
        }
        // Increase current speed up to the limit
        else if (current_vel_ < vel_limit_ - 0.001)
        {
            current_vel_ += acc_limit_*ctrl_time_;
            if (current_vel_ > vel_limit_)
            {
                current_vel_ = vel_limit_;
            }
        }
        // Update current position
        current_pos_ += current_vel_*ctrl_time_;
        if (current_pos_ > target_pos_) {current_pos_ = target_pos_;}
    }

    // Else if target pos is smaller than current pos, pos must decrease
    else
    {
        // If current pose is near the target (within 5 deg)
        if      (current_pos_ - target_pos_ < 0.087)
        {
            // Follow current velocity to 30% of its limit
            if      (current_vel_ < vel_limit_*0.3 - 0.001)
            {
                current_vel_ += acc_limit_*ctrl_time_;
                if (current_vel_ > vel_limit_*0.3)
                {
                    current_vel_ = vel_limit_*0.3;
                }
            }
            else if (current_vel_ > vel_limit_*0.3 + 0.001)
            {
                current_vel_ -= acc_limit_*ctrl_time_;
                if (current_vel_ < vel_limit_*0.3)
                {
                    current_vel_ = vel_limit_*0.3;
                }
            }
        }
        // Increase current speed up to the limit
        else if (current_vel_ < vel_limit_ - 0.001)
        {
            current_vel_ += acc_limit_*ctrl_time_;
            if (current_vel_ > vel_limit_)
            {
                current_vel_ = vel_limit_;
            }
        }
        // Update current position
        current_pos_ -= current_vel_*ctrl_time_;
        if (current_pos_ < target_pos_) {current_pos_ = target_pos_;} 
    }

    // Publish computed current motor pos
    publishFakeMove(current_pos_);
}

// Update current target pose
void MotorMover::moveMotorCallback(const std_msgs::Float64::ConstPtr& msg) 
{
    double target_pos = msg->data;  // within [0,100] interval

    // Remap target_pos to fit within joint limits
    target_pos = std::max(joint_limits_[0], std::min(joint_limits_[1],
                 target_pos * (joint_limits_[1] - joint_limits_[0]) / 100.0));

    // Check if target_pos is within joint limits
    if (target_pos < joint_limits_[0] || target_pos > joint_limits_[1]) {
        ROS_WARN("Target position is out of joint limits!");
        return;
    }

    // Update target pose
    target_pos_ = target_pos;
}

// Setter of target pose for child class
void MotorMover::setTargetPos(double target)
{
    target_pos_ = target;
}

// Fake controller publisher to move group 
void MotorMover::publishFakeMove(double current_pos)
{
    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.name.push_back(joint_name_);
    joint_state_msg.position.push_back(current_pos);
    fake_move_pub_.publish(joint_state_msg);
}

// Spinner ROS + motor update
void MotorMover::spinner()
{
    ros::spinOnce();
    motorPosUpdate();
}
