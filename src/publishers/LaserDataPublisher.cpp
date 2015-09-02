/* 
 * File:   LaserDataPublisher.cpp
 * Author: biron
 * 
 * Created on March 13, 2012, 2:54 PM
 */

#include "LaserDataPublisher.h"
#include <rsb/MetaData.h>

using namespace std;
using namespace boost;
using namespace rst;

namespace ros4rsb {

LaserDataPublisher::LaserDataPublisher(const string &topicIn,string name, ros::NodeHandle node) :
		PublisherImpl(name, node), dataAvailable(false) {

    node.param<bool>("/isInterleavedLaserData", isInterleavedMode, true);

	function<void(const sensor_msgs::LaserScan::ConstPtr&)> m0 = bind(
			mem_fn(&LaserDataPublisher::callback), this, _1);

	rosSubscriber = node.subscribe(topicIn, 1000, m0);
	ROS_INFO_STREAM("LaserDataPublisher: " << name << " is subscribing to topic " << topicIn);
	ROS_INFO_STREAM("LaserDataPublisher: Interleaved mode: " << isInterleavedMode);
}

LaserDataPublisher::~LaserDataPublisher() {
}

void LaserDataPublisher::callback(
		const sensor_msgs::LaserScan::ConstPtr &message) {

	boost::mutex::scoped_try_lock lock(myMutex/*, boost::try_to_lock*/);
	if (!lock.owns_lock()) {
		return;
	}

	std::vector<float> ranges;

	if (message->ranges.size() < 1) {
		std::cerr << "Discarding empty message" << std::endl;
		return;
	}
	if (isInterleavedMode) {
		if (!dataAvailable) {
			this->storedMessage = message;
			dataAvailable = true;
			return;
		} else {
			//	      return;
			//  if (storedMessage->angle_min < message->angle_min) {
			if (storedMessage->ranges.size() > message->ranges.size()) {
				this->assembleInterleavedMessages(this->storedMessage, message,
						ranges);
			} else {
				this->assembleInterleavedMessages(message, this->storedMessage,
						ranges);
			}
			dataAvailable = false;
		}
	} else {

		for (int i = 0; i < message->ranges.size(); i++) {
			float val = message->ranges.at(i);
			if (isnanf(val) || isinff(val) || val<ROBOT_FOOTPRINT_RADIUS) {
				val = message->range_max;
			}
			ranges.push_back(val);
		}
	}

	// create correct timestamp
	uint32_t sec = message->header.stamp.sec;
	uint32_t nsec = message->header.stamp.nsec;
	uint32_t usec = nsec / 1000;
	uint64_t timestamp = sec * 1000000 + usec;

	// create rst type
	shared_ptr<vision::LaserScan> data = shared_ptr<vision::LaserScan>(new vision::LaserScan());
	data->set_scan_angle(abs(message->angle_max - message->angle_min));

	for (vector<float>::iterator it = ranges.begin(); it != ranges.end(); ++it) {
		float* d = data->mutable_scan_values()->Add();
		*d = *it;
	}

	rsb::EventPtr event = rsbInformer->createEvent();
	event->setData(data);
	event->mutableMetaData().setCreateTime(timestamp);
	this->publish(event);
}

void LaserDataPublisher::assembleInterleavedMessages(
		const sensor_msgs::LaserScan::ConstPtr firstMessage,
		const sensor_msgs::LaserScan::ConstPtr secondMessage,
		std::vector<float> &assembledMessage) {

	// assembledMessage.resize( (firstMessage->ranges.size() * 2) -1);
	assembledMessage.push_back(
			firstMessage->ranges.at(firstMessage->ranges.size() - 1));

	size_t messageSizeMOne = (firstMessage->ranges.size() * 2) - 2;
	for (int i = 0; i < secondMessage->ranges.size(); i++) {
		float valFirst = firstMessage->ranges.at(i);
		if (isnanf(valFirst) || isinff(valFirst) || valFirst<ROBOT_FOOTPRINT_RADIUS) {
			valFirst = firstMessage->range_max;
		}
		float valSecond = secondMessage->ranges.at(i);
		if (isnanf(valSecond) || isinff(valSecond) || valSecond<ROBOT_FOOTPRINT_RADIUS) {
			valSecond = secondMessage->range_max;
		}
		assembledMessage.push_back(valFirst);
		assembledMessage.push_back(valSecond);
	}
}
}
