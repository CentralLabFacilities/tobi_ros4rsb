/*
 * GlobalPlanPublisher.cpp
 *
 *  Created on: June 15, 2012
 *      Author: prenner
 */

#include "GlobalPlanPublisher.h"

#define WAYPOINTS_PER_MESSAGE 10

using namespace std;
using namespace ros;
using namespace boost;
using namespace rst;

namespace ros4rsb {

GlobalPlanPublisher::GlobalPlanPublisher(string name, NodeHandle node) :
	Publisher(name, node) {

	function<void(const nav_msgs::PathConstPtr&)> m0 = bind(
			mem_fn(&GlobalPlanPublisher::callback), this, _1);

	tfListener = new tf::TransformListener(node);

	rosSubscriber = node.subscribe(
			"/move_base/NavfnROS/plan", 1000, m0);
	cout << name
			<< " GlobalPlanPublisher is subscribing to /move_base/NavfnROS/plan."
			<< endl;
}

GlobalPlanPublisher::~GlobalPlanPublisher() {
}

void GlobalPlanPublisher::callback(const nav_msgs::PathConstPtr &message) {

	// skip processing when there are no poses
	if (message->poses.empty()) {
		return;
	}

	if (message->header.stamp < ros::Time::now() - ros::Duration(0.15)) {
		return;
	}

	// create RST type
	shared_ptr<navigation::Path> globalPlan = shared_ptr<navigation::Path>(new navigation::Path());

	cout << "Iterating..." << endl;

	// skip some points for efficiency
	unsigned int step = message->poses.size() / WAYPOINTS_PER_MESSAGE;
	if (step == 0)
		++step;
	//cout << "Step: " << step << endl;
	for (unsigned int i = 0; i < message->poses.size(); i += step) {
	//	cout << "i = " << i << endl;

		geometry_msgs::PoseStamped poseOut;
		geometry_msgs::PoseStamped poseIn;
		poseIn.pose = message->poses[i].pose;
		poseIn.header.frame_id = "/odom";

		try {
			tfListener->transformPose("/map", poseIn, poseOut);
		} catch (tf::TransformException e) {
			ROS_INFO("No transform!");
		}
	//	cout << "Transformed..." << endl;

		//rst::geometry::Pose* positionData = globalPlan->add_pose();
                rst::geometry::Pose* positionData = globalPlan->add_poses();

		positionData->mutable_translation()->set_x(poseOut.pose.position.x);
		positionData->mutable_translation()->set_y(poseOut.pose.position.y);
		positionData->mutable_translation()->set_z(poseOut.pose.position.z);
                

            tfScalar roll, pitch, yaw;
            tf::Quaternion q = tf::Quaternion(
                    poseOut.pose.orientation.x,
                    poseOut.pose.orientation.y,
                    poseOut.pose.orientation.z,
                    poseOut.pose.orientation.w);
            tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
                
		positionData->mutable_rotation()->set_qx(poseOut.pose.orientation.x);
		positionData->mutable_rotation()->set_qy(yaw);
		positionData->mutable_rotation()->set_qz(poseOut.pose.orientation.z);
		positionData->mutable_rotation()->set_qw(poseOut.pose.orientation.w);


//		cout << "added waypoint..." << endl;
	}
//	cout << "publishing..." << endl;
	rsb::EventPtr event = rsbInformer->createEvent();
	event->setData(globalPlan);
	timestamp = message->header.stamp.sec * 1000000 + message->header.stamp.nsec
	/ 1000;
	//event->mutableMetaData().setCreateTime(timestamp);
	this->publish(event);
}

} /* namespace ros4rsb */

