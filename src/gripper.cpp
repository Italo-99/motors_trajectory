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

Gripper::Gripper(std::string         gripper_name,
                 std::string         joint_name,
                 std::vector<double> joint_limits,
                 double              vel_limit,
                 double              acc_limit,
                 double              ctrl_rate,
                 bool                inst_target,
                 double              tips_strike,
                 double              tcp_closed,
                 double              tcp_opened)
    : MotorMover(gripper_name, joint_name, joint_limits,
                 vel_limit, acc_limit, ctrl_rate,inst_target),
                 tips_strike_(tips_strike),
                 tcp_closed_(tcp_closed), tcp_opened_(tcp_opened)
{
    // Instantiate global class variables
    joint_limits_.push_back(joint_limits[0]);
    joint_limits_.push_back(joint_limits[1]);

    // Declare service open/close gripper
    gripper_control_srv_ = nh_.advertiseService(gripper_name + 
                        "/move_gripper", &Gripper::moveGripperCallback, this);
}

// Gripper::~Gripper()
// {
//     // Insert destructor
//     return 0;
// }

bool Gripper::moveGripperCallback(std_srvs::SetBool::Request&  req,
                                  std_srvs::SetBool::Response& res)
{
    double target_pos = req.data ? joint_limits_[1] : joint_limits_[0];
    setTargetPos(target_pos);
    res.success = true;
    return true;
}

void Gripper::gripper_spinner()
{
    spinner();
}
