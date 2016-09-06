/* 
 * File:   GlobalPlan.h
 * Author: biron
 *
 * Created on March 29, 2012, 9:38 AM
 */

#pragma once

#include "Publisher.h"

// ROS
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>

// RST
#include <rst/navigation/Path.pb.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

namespace ros4rsb {

/**
 * This class reads sensor messages from the laser and publishes the data
 * on a publisher stream.
 *
 * @author prenner, pdressel, lziegler
 */
class GlobalPlanPublisher: public PublisherImpl<rst::navigation::Path> {
public:
	/**
	 * Creates a new instance of this class.
	 *
	 * @param name The name of the publisher stream.
	 * @param node The ros node providing laser data.
	 * @param isInterleavedMode Does the laser data come in as separate
	 * 			interleaved messages?
	 */
	GlobalPlanPublisher(const std::string &topicIn, std::string name, ros::NodeHandle node);
	virtual ~GlobalPlanPublisher();
        
	/**
	 * Callback method which will be called by the ros node when new data is
	 * available.
	 *
	 * @param message The new incoming message.
	 */
	void callback(const nav_msgs::PathConstPtr &message);
        
private:
    tf::TransformListener *tfListener;
	uint64_t timestamp;
        
};

}
