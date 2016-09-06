/* 
 * File:   StallDataPublisher.h
 * Author: alangfel
 *
 * Created on April 25, 2014, 1:43 PM
 */

#pragma once

#define SKIPCONVERTER
#include "Publisher.h"

// ROS
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/KeyValue.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>

namespace ros4rsb {

    class StallDataPublisher : public PublisherImpl<void>{
    public:
        StallDataPublisher(const std::string &topicIn,std::string name, ros::NodeHandle node);
        virtual ~StallDataPublisher();
        void callback(const diagnostic_msgs::DiagnosticArray::ConstPtr &message);

    };
}
