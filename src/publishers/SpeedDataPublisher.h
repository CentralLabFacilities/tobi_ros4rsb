/* 
 * File:   SpeedDataPublisher.h
 * Author: alangfel
 *
 * Created on April 25, 2014, 1:02 PM
 */

#ifndef SPEEDDATAPUBLISHER_H
#define	SPEEDDATAPUBLISHER_H

#include "Publisher.h"

#include <rst/kinematics/Twist.pb.h>
#include <rsb/MetaData.h>

// ROS
#include <nav_msgs/Odometry.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

namespace ros4rsb {

    class SpeedDataPublisher : public Publisher<rst::kinematics::Twist> {
    public:
        SpeedDataPublisher(std::string name, ros::NodeHandle node);
        virtual ~SpeedDataPublisher();
        void callback(const nav_msgs::Odometry::ConstPtr &message);
    };
}
#endif	/* SPEEDDATAPUBLISHER_H */

