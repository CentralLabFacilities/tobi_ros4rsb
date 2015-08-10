/* 
 * File:   StallDataPublisher.h
 * Author: alangfel
 *
 * Created on April 25, 2014, 1:43 PM
 */

#ifndef STALLDATAPUBLISHER_H
#define	STALLDATAPUBLISHER_H

#define SKIPCONVERTER

#include "Publisher.h"

// ROS
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/KeyValue.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

namespace ros4rsb {

    class StallDataPublisher : public Publisher<void>{
    public:
	StallDataPublisher(std::string name, ros::NodeHandle node);
	virtual ~StallDataPublisher();
        void callback(const diagnostic_msgs::DiagnosticArray::ConstPtr &message);
    };

}

#endif	/* STALLDATAPUBLISHER_H */

