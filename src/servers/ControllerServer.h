//
// Created by lruegeme on 31.05.16.
//

#pragma once

// Ros
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <control_msgs/FollowJointTrajectoryAction.h>



// Local
#include "Server.h"
#include "ServerException.h"

namespace ros4rsb {
    class ControllerServer : public Server {
    public:

        ControllerServer(const std::string &name, ros::NodeHandle &node);

        ~ControllerServer();

        bool zliftGoto(float in);
        bool headGoto(float j0, float j1);

        CREATE_SERVER_BUILDER_NESTED(ControllerServer)

    private:
        std::string name;
        ros::NodeHandle node;

        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *zliftClient;
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *headClient;

        rsb::patterns::LocalServerPtr server;

    };
}

