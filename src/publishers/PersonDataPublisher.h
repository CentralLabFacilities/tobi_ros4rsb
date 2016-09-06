/* 
 * File:   PersonDataPublisher.h
 * Author: biron
 *
 * Created on March 13, 2012, 2:54 PM
 */
#pragma once

#include "Publisher.h"

// ROS
//#include "bayes_people_tracker/PeopleTracker.h"
#include <people_msgs/People.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <string>
#include <rst/hri/PersonHypotheses.pb.h>

namespace ros4rsb {

/**
 * This class reads sensor messages from the laser and publishes the data
 * on a xcf publisher stream.
 *
 * @author prenner, pdressel, lziegler
 */
class PersonDataPublisher: public PublisherImpl<rst::hri::PersonHypotheses> {
public:
	/**
	 * Creates a new instance of this class.
	 *
	 * @param name The name of the rsb publisher stream.
	 * @param node The ros node providing person data.
	 */
	PersonDataPublisher(const std::string &topicIn,std::string name, ros::NodeHandle node);
	virtual ~PersonDataPublisher();
        
	/**
	 * Callback method which will be called by the ros node when new data is
	 * available.
	 *
	 * @param message The new incoming message.
	 */
	void callback(const people_msgs::People::ConstPtr &message);

private:

	bool dataAvailable;
	people_msgs::People::ConstPtr storedMessage;
	boost::mutex myMutex;
};

}
