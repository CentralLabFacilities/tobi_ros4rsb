/*
 * SlamPos.h
 *
 *  Created on: 18.03.2012
 *      Author: leon
 */

#ifndef SLAMPOS_H_
#define SLAMPOS_H_

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

class SlamPosPublisher: public Publisher<rst::geometry::Pose> {

public:
	SlamPosPublisher(std::string name, ros::NodeHandle node, bool isLocalNavigation);
	virtual ~SlamPosPublisher();
	// TODO: Change the message this Publisher is listening to
	void global_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &message);
        void local_callback(const nav_msgs::Odometry::ConstPtr &message);
	void publishThread();
        geometry_msgs::Pose getPose();

private:
        void convertPoseToPositionData(
                const geometry_msgs::Pose &pose, 
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
#endif /* SLAMPOS_H_ */
