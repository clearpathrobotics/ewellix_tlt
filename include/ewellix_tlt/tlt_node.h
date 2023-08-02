/**
Software License Agreement (BSD)

\file      tlt_node.h
\authors   Luis Camero <lcamero@clearpathrobotics.com>
\copyright Copyright (c) 2023, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of Clearpath Robotics nor the names of its contributors
  may be used to endorse or promote products derived from this software
  without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef TLTNODE_H
#define TLTNODE_H

#include "joint_trajectory_action_server.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "serial_com_tlt.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include <string>
#include <thread>
#include <chrono>


class TltNode
{
    public:
        TltNode(ros::NodeHandle);
        ~TltNode();

    private:
        SerialComTlt srl_;
        thread com_thread_;
        thread join_states_thread_;
        // Actions
        JointTrajectoryActionServer* srv_follow_joint_trajectory_;

        // Publishers
        ros::Publisher pub_joint_states_;
        ros::Publisher pub_column_position_;

        // Services
        ros::ServiceServer srv_init_sequence_;

        // Subscribers
        ros::Subscriber sub_column_stop_;
        ros::Subscriber sub_column_position_;
        ros::Subscriber sub_column_duration_down_;
        ros::Subscriber sub_column_duration_up_;
        ros::Subscriber sub_joy_;
        ros::Subscriber sub_motor1_ticks_;
        ros::Subscriber sub_motor2_ticks_;

        // Subscriber Callback
        void cbStop(std_msgs::Empty);
        void cbPosition(std_msgs::Float32);
        void cbDurationUp(std_msgs::Float32);
        void cbDurationDown(std_msgs::Float32);
        void cbMotor1Ticks(std_msgs::Int16);
        void cbMotor2Ticks(std_msgs::Int16);
        void cbJoy(sensor_msgs::Joy);
        void publishJoinStates();

        // Service Calls
        bool srvInitSequence(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

};

#endif //TLTNODE_H
