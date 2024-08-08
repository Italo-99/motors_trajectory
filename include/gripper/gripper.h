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

#ifndef GRIPPER_H
#define GRIPPER_H

#include "gripper/motor_mover.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Bool.h"

class Gripper
{
public:
    Gripper(std::string node_name);
    ~Gripper();

    // ROS spinner + update gripper position
    void gripper_spinner(void);

private:

    // Open/close gripper service
    ros::ServiceServer gripper_control_srv_;
    bool moveGripperCallback(std_srvs::SetBool::Request&  req,
                             std_srvs::SetBool::Response& res);

    // Grab/detach gripper service
    ros::ServiceServer gripper_grab_srv_;
    bool grabbingGripperCallback(std_srvs::SetBool::Request&  req,
                                 std_srvs::SetBool::Response& res);

    // Subscriber of object found in the middle of robot tips
    ros::Subscriber objFoundSubscriber_;
    void objFoundCallback(const std_msgs::Bool::ConstPtr& msg);

    // Grabbing and detach object at the gripper: pipeline implementation
    void grabObj(void);
    void detachObj(void);

    // Publisher to attach/detach an object at the gripper
    ros::Publisher attachObjGripperPub_;

    // Class attributes
    std::string         group_name_;
    std::string         joint_name_;
    std::vector<double> joint_limits_;
    double              vel_limit_;
    double              acc_limit_;
    double              ctrl_rate_;
    bool                inst_target_;
    double              tips_strike_;
    double              tcp_closed_;
    double              tcp_opened_;
    double              tcp_;
    bool                sim_;
    bool                obj_gripper_found_;
    bool                grab_obj_cmd_;

    MotorMover*         gripper_mover_;
    std::string         node_name_;

    // ROS node variables
    ros::NodeHandle nh_;

    // Check class attributes
    void check_param(void);
};

#endif // GRIPPER_H
