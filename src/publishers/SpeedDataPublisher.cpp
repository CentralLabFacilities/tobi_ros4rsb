/* 
 * File:   SpeedDataPublisher.cpp
 * Author: alangfel
 * 
 * Created on April 25, 2014, 1:02 PM
 */

#include "SpeedDataPublisher.h"
#include <rsb/MetaData.h>

using namespace std;
using namespace ros;
using namespace boost;
using namespace rst;

namespace ros4rsb {

    SpeedDataPublisher::SpeedDataPublisher(const string &topicIn,string name, NodeHandle node) :
        PublisherImpl(name, node) {

        function<void(const nav_msgs::Odometry::ConstPtr&) > m0 = bind(
                mem_fn(&SpeedDataPublisher::callback), this, _1);

        rosSubscriber = node.subscribe(topicIn, 1000, m0);
        ROS_INFO_STREAM("SpeedDataPublisher: " << name << " is subscribing to topic " << topicIn);
    }

    SpeedDataPublisher::~SpeedDataPublisher() {
    }

    void SpeedDataPublisher::callback(const nav_msgs::Odometry::ConstPtr& message) {
        float translationSpeed = message->twist.twist.linear.x;
        float yawSpeed = message->twist.twist.angular.z;

        // create correct timestamp
        uint32_t sec = message->header.stamp.sec;
        uint32_t nsec = message->header.stamp.nsec;
        uint32_t usec = nsec / 1000;
        uint64_t timestamp = sec * 1000000 + usec;

        shared_ptr<kinematics::Twist> data = shared_ptr<kinematics::Twist>(new kinematics::Twist());

        data->mutable_angular()->set_a(message->twist.twist.angular.x);
        data->mutable_angular()->set_b(message->twist.twist.angular.y);
        data->mutable_angular()->set_c(yawSpeed);
        
        data->mutable_linear()->set_x(translationSpeed);
        data->mutable_linear()->set_y(message->twist.twist.linear.y);
        data->mutable_linear()->set_z(message->twist.twist.linear.z);

        rsb::EventPtr event = rsbInformer->createEvent();
        event->setData(data);
        
        event->mutableMetaData().setUserTime("created", timestamp);
        event->mutableMetaData().setUserTime("updated", timestamp);
        
        this->publish(event);
    }

}

