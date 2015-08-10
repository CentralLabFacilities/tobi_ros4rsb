/*
 * SlamPos.cpp
 *
 *  Created on: 18.03.2012
 *      Author: leon
 */

#include "SlamPosPublisher.h"

#include <LinearMath/btQuaternion.h> 
#include <LinearMath/btMatrix3x3.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

#include <rsb/MetaData.h>

using namespace std;
using namespace ros;
using namespace rst::geometry;
using namespace boost;

namespace ros4rsb {

SlamPosPublisher::SlamPosPublisher(string name, NodeHandle node,
		bool isLocalNavigation) :
	Publisher(name, node), hasData(false) {

	if (isLocalNavigation) {
		function<void(const nav_msgs::Odometry::ConstPtr&)> m1 = bind(
				mem_fn(&SlamPosPublisher::local_callback), this, _1);
		rosSubscriber = node.subscribe("odom", 1000, m1);
		tfListener = new tf::TransformListener(node);
		std::cout << name << " subscribed to odom topic." << std::endl;
	} else {
		function<void(const geometry_msgs::PoseWithCovarianceStampedConstPtr&)>
				m0 = bind(mem_fn(&SlamPosPublisher::global_callback), this, _1);
		rosSubscriber = node.subscribe("amcl_pose", 1000, m0);
		std::cout << name << " subscribed to acml_pose topic." << std::endl;
	}

	runner = boost::thread(&SlamPosPublisher::publishThread, this);

}

SlamPosPublisher::~SlamPosPublisher() {
	delete tfListener;
}

void SlamPosPublisher::publishThread() {

	while (true) {

		{
			boost::mutex::scoped_lock lock(mutex);

			if (hasData) {
				rsb::EventPtr event = rsbInformer->createEvent();
				event->setData(positionData);
				event->mutableMetaData().setUserTime("created", timestamp);
                                event->mutableMetaData().setUserTime("updated", timestamp);
				this->publish(event);
				hasData = false;
			}
		}

		usleep(1000 * 10);
	}
}

void SlamPosPublisher::global_callback(
		const geometry_msgs::PoseWithCovarianceStampedConstPtr &message) {

	boost::mutex::scoped_lock lock(mutex);

	this->pose = message->pose.pose;

	positionData = shared_ptr<Pose>(new Pose());
	convertPoseToPositionData(message->pose.pose, positionData);
	timestamp = message->header.stamp.sec * 1000000 + message->header.stamp.nsec
				/ 1000;
	hasData = true;

}

void SlamPosPublisher::local_callback(
		const nav_msgs::Odometry::ConstPtr &message) {

	boost::mutex::scoped_lock lock(mutex);

	geometry_msgs::PoseStamped poseOut;
	geometry_msgs::PoseStamped poseIn;
	poseIn.pose = message->pose.pose;
	poseIn.header.frame_id = "/odom";
	try {
		tfListener->transformPose("/map", poseIn, poseOut);
	} catch (tf::TransformException e) {
		ROS_INFO("No transform!");
	}

	this->pose = poseOut.pose;

	positionData = shared_ptr<Pose>(new Pose());
	convertPoseToPositionData(poseOut.pose, positionData);
	timestamp = poseIn.header.stamp.sec * 1000000 + poseIn.header.stamp.nsec
			/ 1000;
	hasData = true;
}

void SlamPosPublisher::convertPoseToPositionData(
		const geometry_msgs::Pose &pose, shared_ptr<Pose> positionData) {

	positionData->mutable_translation()->set_x(pose.position.x);
	positionData->mutable_translation()->set_y(pose.position.y);
	positionData->mutable_translation()->set_z(pose.position.z);

	positionData->mutable_rotation()->set_qx(pose.orientation.x);
	positionData->mutable_rotation()->set_qy(pose.orientation.y);
	positionData->mutable_rotation()->set_qz(pose.orientation.z);
	positionData->mutable_rotation()->set_qw(pose.orientation.w);
}

geometry_msgs::Pose SlamPosPublisher::getPose() {
	boost::mutex::scoped_lock lock(mutex);
	return pose;
}

} /* namespace ros4rsb */
