/* 
 * File:   OdometryDataPublisher.cpp
 * Author: alangfel
 * 
 * Created on April 25, 2014, 11:01 AM
 */

#include "OdometryDataPublisher.h"
#include <math.h>
#include <tf2/LinearMath/Scalar.h>
#include <rsb/MetaData.h>

using namespace ros;
using namespace boost;
using namespace rst;

namespace ros4rsb {

    OdometryDataPublisher::OdometryDataPublisher(const std::string &topicIn,std::string name, ros::NodeHandle node) :
        PublisherImpl(name, node) {

        function<void(const nav_msgs::Odometry::ConstPtr&) > m0 = bind(
                mem_fn(&OdometryDataPublisher::callback), this, _1);

        rosSubscriber = node.subscribe(topicIn, 1000, m0);
        ROS_INFO_STREAM("OdometryDataPublisher: " << name << " is subscribing to topic " << topicIn);
    }

    OdometryDataPublisher::~OdometryDataPublisher() {
    }

    void OdometryDataPublisher::callback(const nav_msgs::Odometry::ConstPtr& message) {

        float x = message->pose.pose.position.x;
        float y = message->pose.pose.position.y;
        tf2Scalar roll, pitch, yaw;
        tf2::Quaternion q = tf2::Quaternion(
                message->pose.pose.orientation.x,
                message->pose.pose.orientation.y,
                message->pose.pose.orientation.z,
                message->pose.pose.orientation.w);
        tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

        // create correct timestamp
        uint32_t sec = message->header.stamp.sec;
        uint32_t nsec = message->header.stamp.nsec;
        uint32_t usec = nsec / 1000;
        timeval t;
        t.tv_sec = sec;
        t.tv_usec = usec;

        //create rst type
        shared_ptr<geometry::Pose> odoData = shared_ptr<geometry::Pose>(new geometry::Pose());

        odoData->mutable_translation()->set_x(message->pose.pose.position.x);
        odoData->mutable_translation()->set_y(message->pose.pose.position.y);
        odoData->mutable_translation()->set_z(message->pose.pose.position.z);

        odoData->mutable_rotation()->set_qx(message->pose.pose.orientation.x);
        odoData->mutable_rotation()->set_qy(message->pose.pose.orientation.y);
        odoData->mutable_rotation()->set_qz(message->pose.pose.orientation.z);
        odoData->mutable_rotation()->set_qw(message->pose.pose.orientation.w);


        
	uint64_t timestamp = message->header.stamp.sec * 1000000 + message->header.stamp.nsec
				/ 1000;
        rsb::EventPtr event = rsbInformer->createEvent();
        
        event->setData(odoData);
        event->mutableMetaData().setUserTime("created", timestamp);
        event->mutableMetaData().setUserTime("updated", timestamp);

        this->publish(event);


    }


}
