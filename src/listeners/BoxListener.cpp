#include "BoxListener.h"

using namespace std;
using namespace ros;
using namespace boost;
using namespace rst;

namespace ros4rsb {

BoxListener::BoxListener(const string &scope, const string &topic, NodeHandle node) :
		ListenerPub(scope, topic, node) {
}

BoxListener::~BoxListener() {
}

void BoxListener::callback(BoxPtr input) {
	geometry_msgs::Pose pose;
	pose.orientation.w = 1.0;
	pose.position.x = input->transformation().translation().x();
	pose.position.y = input->transformation().translation().y();
	pose.position.z = input->transformation().translation().z();

	visualization_msgs::Marker marker;
	marker.header.stamp = ros::Time::now();
	marker.header.frame_id = input->transformation().translation().frame_id();
	marker.id = 0;
	marker.type = marker.CUBE;
	marker.ns = "box";
	marker.pose = pose;
	marker.scale.x = input->width();
	marker.scale.y = input->depth();
	marker.scale.z = input->height();
	marker.color.b = 1.0;
	marker.color.a = 1.0;

	publish(marker);
}

}

