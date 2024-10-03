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

#ifndef MOTOR_MOVER_H
#define MOTOR_MOVER_H

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include <vector>

class MotorMover 
{
public:
    MotorMover( std::string&         group_name, 
                std::string&         joint_name, 
                std::vector<double>& joint_limits,  
                double               vel_limit, 
                double               acc_limit, 
                double               ctrl_rate,
                bool                 inst_target,
                bool                 gripper);   
                
    //Set inst_target to true to update setpoint at target instantaneously 

    // ~MotorMover();

    // ROS spinner
    void spinner(void);

    // Setter of target pose for child class
    void setTargetPos(double);

    // Get of current (setpoint) pose for child class
    double getCurrentPos(void);

private:
    
    // ROS variables
    ros::NodeHandle nh_;
    ros::Subscriber motor_control_sub_;
    ros::Subscriber joint_state_sub_;
    ros::Subscriber instKine_setter_sub_;
    ros::Publisher  fake_move_pub_;

    // Class attributes
    std::string         group_name_;
    std::string         joint_name_;
    std::vector<double> joint_limits_;
    double              vel_limit_;
    double              acc_limit_;
    double              ctrl_rate_;
    bool                inst_target_;
    bool                gripper_;

    // Class variables 
    double target_pos_;
    double current_pos_;
    double current_target_;
    double ctrl_time_;
    double current_vel_;
    bool   first_joint_sub_;
    int    motor_index_;
    bool   inst_kine_;
    bool   target_reached_;
    
    // Update current motor position
    void motorPosUpdate(void);

    // Reading joint state
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& js);

    // Update motor position target pose
    void moveMotorCallback(const std_msgs::Float64::ConstPtr& msg);

    // Motor moving function through move group fake controller
    void publishFakeMove(double target_pos, double current_vel);

    // Instantaneous kine mode callback
    void instantKineSetterCallback(const std_msgs::Bool::ConstPtr& msg);

};

#endif // MOTOR_MOVER_H
