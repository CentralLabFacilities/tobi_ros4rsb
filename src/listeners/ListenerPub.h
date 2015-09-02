#pragma once

#include "Listener.h"

#include <ros/ros.h>

/**
 * This is a abstract class defining the outline of every Publisher that wraps ROS topics to RST types.
 */
namespace ros4rsb {

template<class RsbType, class RosType>
class ListenerPub: public ListenerImpl<RsbType> {
public:

    ListenerPub(const std::string &scopeIn, const std::string &topicOut, ros::NodeHandle node) :
        ListenerImpl<RsbType>(scopeIn) {
		this->node = node;
		this->rosPublisher = node.advertise<RosType>(topicOut, 10);

	}

	virtual ~ListenerPub() {
		rosPublisher.shutdown();
	}

	virtual void publish(const RosType &data) {
		rosPublisher.publish(data);
	}

private:
	ros::NodeHandle node;

protected:
	ros::Publisher rosPublisher;
};

}
