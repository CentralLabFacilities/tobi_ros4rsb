/* 
 * File:   SpeedDataPublisher.h
 * Author: alangfel
 *
 * Created on April 25, 2014, 1:02 PM
 */

#pragma once

#include "Publisher.h"

#include <rst/kinematics/Twist.pb.h>
#include <rsb/MetaData.h>

// ROS
#include <nav_msgs/Odometry.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

namespace ros4rsb {

    class SpeedDataPublisher : public PublisherImpl<rst::kinematics::Twist> {
    public:
        SpeedDataPublisher(const std::string &topicIn,std::string name, ros::NodeHandle node);
        virtual ~SpeedDataPublisher();
        void callback(const nav_msgs::Odometry::ConstPtr &message);

        CREATE_PUBLISHER_BUILDER_NESTED(SpeedDataPublisher)
    };

}
