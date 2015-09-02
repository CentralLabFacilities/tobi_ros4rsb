/*
 * SlamPos.h
 *
 *  Created on: 18.03.2012
 *      Author: leon
 */

#pragma once
#include "Publisher.h"

// ROS
#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_listener.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <rst/geometry/Pose.pb.h>

namespace ros4rsb {

class SlamPosPublisher: public PublisherImpl<rst::geometry::Pose> {
public:
	SlamPosPublisher(const std::string &topicIn,std::string name, ros::NodeHandle node);
	virtual ~SlamPosPublisher();
	// TODO: Change the message this Publisher is listening to
	void global_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &message);
    void local_callback(const nav_msgs::Odometry::ConstPtr &message);
	void publishThread();
        geometry_msgs::Pose getPose();

    CREATE_PUBLISHER_BUILDER_NESTED(SlamPosPublisher)

private:
    void convertPoseToPositionData(const geometry_msgs::Pose &pose,
            boost::shared_ptr<rst::geometry::Pose> positionData);
    
	boost::mutex mutex;
	boost::thread runner;
    tf::TransformListener *tfListener;
    geometry_msgs::Pose pose;
	boost::shared_ptr<rst::geometry::Pose> positionData;
	uint64_t timestamp;
	bool hasData;
};

} /* namespace ros4rsb */
