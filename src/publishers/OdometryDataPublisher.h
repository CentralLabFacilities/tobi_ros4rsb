/* 
 * File:   OdometryDataPublisher.h
 * Author: alangfel
 *
 * Created on April 25, 2014, 11:01 AM
 */

#pragma once

#include "Publisher.h"

// ROS
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Quaternion.h> 
#include <tf2/LinearMath/Matrix3x3.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

#include <rst/geometry/Pose.pb.h>
#include <rst/navigation/Path.pb.h>

namespace ros4rsb {

    class OdometryDataPublisher : public PublisherImpl<rst::geometry::Pose> {
    public:
        OdometryDataPublisher(const std::string &topicIn,std::string name, ros::NodeHandle node);
        virtual ~OdometryDataPublisher();
        void callback(const nav_msgs::Odometry::ConstPtr &message);

        CREATE_PUBLISHER_BUILDER_NESTED(OdometryDataPublisher)
    };
}
