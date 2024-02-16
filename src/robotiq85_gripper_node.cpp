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

int main(int argc, char** argv)
{
    // Init node
    ros::init(argc, argv, "robotiq85_gripper");
    ros::NodeHandle nh("~");

    // Init node param
    std::string         gripper_name;
    std::string         joint_name;
    std::vector<double> joint_limits;
    double              vel_limit;
    double              acc_limit;
    double              ctrl_rate;
    bool                inst_target;
    double              tips_strike;
    double              tcp_closed;
    double              tcp_opened;

    nh.param<std::string>("robotiq85_gripper/gripper_name", gripper_name);
    nh.param<std::string>("robotiq85_gripper/joint_name", joint_name);
    nh.param<std::vector<double>>("robotiq85_gripper/joint_limits", joint_limits);
    nh.param<double>("robotiq85_gripper/vel_limit", vel_limit);
    nh.param<double>("robotiq85_gripper/acc_limit", acc_limit);
    nh.param<double>("robotiq85_gripper/ctrl_rate", ctrl_rate);
    nh.param<bool>("robotiq85_gripper/inst_target", inst_target);
    nh.param<double>("robotiq85_gripper/tips_strike", tips_strike);
    nh.param<double>("robotiq85_gripper/tcp_closed", tcp_closed);
    nh.param<double>("robotiq85_gripper/tcp_opened", tcp_opened);

    // Call gripper constructor
    Gripper robotiq85_gripper(gripper_name,joint_name,joint_limits,
                              vel_limit,acc_limit,ctrl_rate,inst_target,
                              tips_strike,tcp_closed,tcp_opened);
    ros::Rate rate(ctrl_rate);

    // ROS spinner
    while (ros::ok()) {
        robotiq85_gripper.gripper_spinner();
        rate.sleep();
    }

    ROS_INFO("Gripper node is shutting down");

    return 0;
}
