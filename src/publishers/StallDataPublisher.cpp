/* 
 * File:   StallDataPublisher.cpp
 * Author: alangfel
 * 
 * Created on April 25, 2014, 1:43 PM
 */

#include "StallDataPublisher.h"

using namespace ros;
using namespace boost;

#define STR_LEFT_WHEEL_STALL "left wheel stall"
#define STR_RIGHT_WHEEL_STALL "right wheel stall"
#define STR_TRUE "True"

namespace ros4rsb {
    
    StallDataPublisher::StallDataPublisher(const std::string &topicIn,std::string name, NodeHandle node) :
        PublisherImpl(name, node) {

        function<void(const diagnostic_msgs::DiagnosticArray::ConstPtr&) > m0 = bind(
                mem_fn(&StallDataPublisher::callback), this, _1);

        rosSubscriber = node.subscribe(topicIn, 1000, m0);
        ROS_INFO_STREAM("SpeedDataPublisher: " << name << " is subscribing to topic " << topicIn);
    }

    StallDataPublisher::~StallDataPublisher() {
    }

    void StallDataPublisher::callback(const diagnostic_msgs::DiagnosticArray::ConstPtr &message) {

        bool isStalled = false;

        diagnostic_msgs::DiagnosticStatus status;
        diagnostic_msgs::KeyValue keyVal;

        for (int i = 0; i < message->status.size(); ++i) {
            status = (diagnostic_msgs::DiagnosticStatus) message->status[i];

            for (int j = 0; j < status.values.size(); ++j) {
                //cout << "Keyname: " << keyVal.key << "\t Value: " << keyVal.value << endl;

                keyVal = ((diagnostic_msgs::KeyValue) status.values[j]);
                // TODO: Check defined keynames for values
                if ((keyVal.key.compare(STR_LEFT_WHEEL_STALL) == 0) && (keyVal.value.compare(STR_TRUE) == 0))
                    isStalled = true;
                if ((keyVal.key.compare(STR_RIGHT_WHEEL_STALL) == 0) && (keyVal.value.compare(STR_TRUE) == 0))
                    isStalled = true;
            }
        }
     
        // create correct timestamp
        uint32_t sec = message->header.stamp.sec;
        uint32_t nsec = message->header.stamp.nsec;
        uint32_t usec = nsec / 1000;
        uint64_t timestamp = sec * 1000000 + usec;

        // send only when 'true'
        if (isStalled) {
            rsb::VoidPtr data;
            rsb::EventPtr event = rsbInformer->createEvent();
            event->setData(data);
            this->publish(event);
        }
    }
}
