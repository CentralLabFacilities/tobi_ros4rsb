/* 
 * File:   LaserDataPublisher.h
 * Author: biron
 *
 * Created on March 13, 2012, 2:54 PM
 */
#pragma once

#include "Publisher.h"

// ROS
#include <sensor_msgs/LaserScan.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <string>
#include <rst/vision/LaserScan.pb.h>

namespace ros4rsb {

/**
 * This class reads sensor messages from the laser and publishes the data
 * on a xcf publisher stream.
 *
 * @author prenner, pdressel, lziegler
 */
class LaserDataPublisher: public PublisherImpl<rst::vision::LaserScan> {
public:
	/**
	 * Creates a new instance of this class.
	 *
	 * @param name The name of the xcf publisher stream.
	 * @param node The ros node providing laser data.
	 * @param isInterleavedMode Does the laser data come in as separate
	 * 			interleaved messages?
	 */
	LaserDataPublisher(const std::string &topicIn,std::string name, ros::NodeHandle node);
	virtual ~LaserDataPublisher();
        
	/**
	 * Callback method which will be called by the ros node when new data is
	 * available.
	 *
	 * @param message The new incoming message.
	 */
	void callback(const sensor_msgs::LaserScan::ConstPtr &message);

	CREATE_PUBLISHER_BUILDER_NESTED(LaserDataPublisher)
private:

	/**
	 * Assemble two interleaved messages into one consistent complete reading.
	 *
	 * @param firstMessage First message
	 * @param secondMessage Second message
	 * @param assembledMessage output parameter: the assembled data structure.
	 */
	void assembleInterleavedMessages(
			const sensor_msgs::LaserScan::ConstPtr firstMessage,
			const sensor_msgs::LaserScan::ConstPtr secondMessage,
			std::vector<float> &assembledMessage);
        static const float ROBOT_FOOTPRINT_RADIUS = 0.25f;
	bool isInterleavedMode;
	bool dataAvailable;
	sensor_msgs::LaserScan::ConstPtr storedMessage;
	boost::mutex myMutex;
};

}
