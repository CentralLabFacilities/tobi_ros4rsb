/* 
 * File:   GlobalPlannerStatePublisher.h
 * Author: alangfel
 *
 * Created on April 25, 2014, 10:40 AM
 */

#pragma once

#include "Publisher.h"

// ROS
#include <nav_msgs/GetPlan.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>

// RST
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

namespace ros4rsb {

    class GlobalPlannerStatePublisher : public PublisherImpl<std::string> {
        
    public:
        GlobalPlannerStatePublisher(const std::string &topicIn,
                const std::string &name,
                ros::NodeHandle &node);
        virtual ~GlobalPlannerStatePublisher();

        void setActiveGoal(actionlib_msgs::GoalID goalID);
        void publishPlannerState();


    private:
        void callback(const actionlib_msgs::GoalStatusArrayConstPtr &message);

        actionlib::SimpleClientGoalState::StateEnum lastState;

    };
    
}
