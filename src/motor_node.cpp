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

#include "gripper/motor_mover.h"

int main(int argc, char** argv)
{
    // Node name (IT MUST BE EQUAL TO THE MANIPULATOR/ROBOT/GRIPPER NAME)
    // std::string node_name = "motor_node";

    // Init node
    ros::init(argc, argv,"motor_node");
    ros::NodeHandle nh;
    std::string node_name = ros::this_node::getName(); 

    // Declare params
    std::string motor_name;
    std::string joint_name;
    std::vector<double> joint_limits;
    double vel_limit, acc_limit, ctrl_rate;
    bool inst_target, sim;

    // Read params
    if (!nh.getParam(node_name+"/group_name", motor_name))
        {
            ROS_ERROR("Motor name parameter not found");
            ros::shutdown();
            
        }
    if (!nh.getParam(node_name+"/joint_name", joint_name))
        {
            ROS_ERROR("Joint name parameter not found");
            ros::shutdown();            
        }
    if (!nh.getParam(node_name+"/joint_limits", joint_limits))
        {
            ROS_ERROR("Joint limits parameter not found");
            ros::shutdown();
            
        }
    if (!nh.getParam(node_name+"/vel_limit", vel_limit))
        {
            ROS_ERROR("Vel limit parameter not found");
            ros::shutdown();
            
        }
    if (!nh.getParam(node_name+"/acc_limit", acc_limit))
        {
            ROS_ERROR("Acc limit parameter not found");
            ros::shutdown();
            
        }
    if (!nh.getParam(node_name+"/ctrl_rate", ctrl_rate))
        {
            ROS_WARN("Ctrl rate parameter not found, set default 50 Hz");
            ctrl_rate = 50;
        }
    if (!nh.getParam(node_name+"/inst_target", inst_target))
        {
            ROS_ERROR("Inst target parameter not found");
            ros::shutdown();
            
        }
    if (!nh.getParam(node_name+"/sim", sim))    // TODO: maybe this is not needed
        {
            sim = false;
        }
    // End params list
    // Call motor mover constructor
    MotorMover motor(motor_name, joint_name, joint_limits, vel_limit, acc_limit, ctrl_rate, inst_target, false);

    // Set spinner rate
    ros::Rate rate(ctrl_rate);

    // ROS spinner
    while (ros::ok())
    {
        motor.spinner();
        rate.sleep();
    }

    // When ROS spinner is blocked
    ROS_INFO("Gripper node is shutting down");

    return 0;
}
