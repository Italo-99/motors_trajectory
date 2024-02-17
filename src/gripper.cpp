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

#include "robotiq85_gripper/gripper.h"

Gripper::Gripper(std::string node_name)
{
    // Load node name param
    node_name_ = node_name;

    // Load params into global class attributes
    check_param();

    // Call the motor_mover class
    gripper_mover_ = new MotorMover(gripper_name_,joint_name_,
                                    joint_limits_,vel_limit_,acc_limit_,
                                    ctrl_rate_,inst_target_);

    // Declare service open/close gripper
    gripper_control_srv_ = nh_.advertiseService(gripper_name_+ 
                        "/move_gripper", &Gripper::moveGripperCallback, this);
}

Gripper::~Gripper(){delete gripper_mover_;}

void Gripper::check_param()
{
    if (!nh_.getParam(node_name_+"/gripper_name", gripper_name_))
        {
            ROS_ERROR("parameter not found");
            ros::shutdown();
            return;
        }
    if (!nh_.getParam(node_name_+"/joint_name", joint_name_))
        {
            ROS_ERROR("parameter not found");
            ros::shutdown();
            return;
        }
    if (!nh_.getParam(node_name_+"/joint_limits", joint_limits_))
        {
            ROS_ERROR("parameter not found");
            ros::shutdown();
            return;
        }
    if (!nh_.getParam(node_name_+"/vel_limit", vel_limit_))
        {
            ROS_ERROR("parameter not found");
            ros::shutdown();
            return;
        }
    if (!nh_.getParam(node_name_+"/acc_limit", acc_limit_))
        {
            ROS_ERROR("parameter not found");
            ros::shutdown();
            return;
        }
    if (!nh_.getParam(node_name_+"/ctrl_rate", ctrl_rate_))
        {
            ROS_ERROR("parameter not found");
            ros::shutdown();
            return;
        }
    if (!nh_.getParam(node_name_+"/inst_target", inst_target_))
        {
            ROS_ERROR("parameter not found");
            ros::shutdown();
            return;
        }
    if (!nh_.getParam(node_name_+"/tips_strike", tips_strike_))
        {
            ROS_ERROR("parameter not found");
            ros::shutdown();
            return;
        }
    if (!nh_.getParam(node_name_+"/tcp_closed", tcp_closed_))
        {
            ROS_ERROR("parameter not found");
            ros::shutdown();
            return;
        }
    if (!nh_.getParam(node_name_+"/tcp_opened", tcp_opened_))
        {
            ROS_ERROR("parameter not found");
            ros::shutdown();
            return;
        }
    if (!nh_.getParam(node_name_+"/tcp", tcp_))
        {
            ROS_ERROR("parameter not found");
            ros::shutdown();
            return;
        }
}

bool Gripper::moveGripperCallback(std_srvs::SetBool::Request&  req,
                                  std_srvs::SetBool::Response& res)
{
    double target_pos = req.data ? 100. : 0.;
    gripper_mover_->setTargetPos(target_pos);
    res.success = true;
    res.message = req.data ? "Gripper closed" : "Gripper opened";
    return true;
}

void Gripper::gripper_spinner()
{
    // Set spinner rate
    ros::Rate rate(ctrl_rate_);

    // ROS spinner
    while (ros::ok())
    {
        gripper_mover_->spinner();
        rate.sleep();
    }
}
