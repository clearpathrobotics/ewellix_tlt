/**
Software License Agreement (BSD)

\file      joint_trajectory_action_server.cpp
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

#include "ewellix_tlt/joint_trajectory_action_server.h"

JointTrajectoryActionServer::JointTrajectoryActionServer(const std::string server_name, ros::NodeHandle &nh, SerialComTlt &srl):
    server_name_(server_name),
    nh_(nh),
    server_(nh, server_name, boost::bind(&JointTrajectoryActionServer::goalReceivedCb, this, _1), boost::bind(&JointTrajectoryActionServer::preemptReceivedCb, this, _1), false),
    server_state_(ActionServerState::INIT)
{
    srl_ = &srl;
    server_.start();
    setServerState(ActionServerState::IDLE);
}

JointTrajectoryActionServer::~JointTrajectoryActionServer(){
    return;
}

void JointTrajectoryActionServer::goalReceivedCb(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle){
    ROS_INFO("Goal Received!");
    // check goal
    if (!isGoalAcceptable(goal_handle)){
        goal_handle.setRejected();
        return;
    }
    // stop previous goal
    if(server_state_ != ActionServerState::IDLE)
    {
        ROS_WARN("Already processing another goal. Cancelling previous.");
        stopAllMovement();
    }
    setServerState(ActionServerState::SETUP);
    // accept goal
    ROS_INFO("Goal Accepted!");
    goal_ = goal_handle;
    goal_.setAccepted();
    // push trajectory to queue
    auto points = goal_.getGoal()->trajectory.points;
    for(auto it = points.begin(); it != points.end(); it++){
        srl_->motionQueuePositionGoal(it->positions[0], 100);
    }
    srl_->motionQueuePrune();
    // wait to return back to idle
    bool processing = true;
    control_msgs::FollowJointTrajectoryResult result;
    sleep(1);
    while(processing){
        sleep(1);
        switch(srl_->state_){
            case SerialComTlt::State::MOTION:
                //ROS_INFO("Moving...");
                break;
            case SerialComTlt::State::IDLE:
                ROS_INFO("Finished.");
                result.error_code = result.SUCCESSFUL;
                goal_.setSucceeded(result);
                processing = false;
                break;
            case SerialComTlt::State::FAILURE:
                ROS_INFO("Failed.");
                result.error_code = result.INVALID_GOAL;
                goal_.setAborted(result);
                processing = false;
                break;
        }

    }
    setServerState(ActionServerState::IDLE);
}

void JointTrajectoryActionServer::preemptReceivedCb(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle){
    ROS_INFO("Goal Preempted!");
    //stopAllMovement();
}

bool JointTrajectoryActionServer::isGoalAcceptable(actionlib::ActionServer<control_msgs::FollowJointTrajectoryAction>::GoalHandle goal_handle){
    if(!goal_handle.isValid()){
        ROS_INFO("Unaccetable goal. Goal is invalid.");
        return false;
    }

    control_msgs::FollowJointTrajectoryGoalConstPtr goal = goal_handle.getGoal();

    if(goal->trajectory.joint_names.size() != joint_names_.size()){
        ROS_INFO("Unacceptable goal. Joint number mismatch.");
        return false;
    }

    if (joint_names_ != goal->trajectory.joint_names)
    {
        ROS_INFO("Unacceptable goal. Joint names mismatch.");
        return false;
    }
    return true;
}

bool JointTrajectoryActionServer::isGoalToleranceRespected(bool check_time_tolerance){
    return true;
}

void JointTrajectoryActionServer::stopAllMovement(){
    srl_->motionStop();
}

void JointTrajectoryActionServer::setServerState(ActionServerState s){
    server_state_lock_.lock();
    server_state_ = s;
    server_state_lock_.unlock();
}
