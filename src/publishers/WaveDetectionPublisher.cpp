/* 
 * File:   WaveDetectionPublisher.cpp
 * Author: alangfel
 * 
 * Created on April 25, 2014, 2:37 PM
 */

#include "WaveDetectionPublisher.h"
#include <rsb/MetaData.h>
using namespace rst;
using namespace boost;

namespace ros4rsb {

    WaveDetectionPublisher::WaveDetectionPublisher(const std::string &topicIn,const std::string &name, ros::NodeHandle &node) : PublisherImpl(name, node) {
        boost::function<void(const geometry_msgs::PointStampedConstPtr&) > m0 =
                bind(mem_fn(&WaveDetectionPublisher::handCreateCallback), this, _1);
        handCreateSub = node.subscribe("/HandCreate", 1000, m0);
        boost::function<void(const geometry_msgs::PointStampedConstPtr&) > m1 =
                bind(mem_fn(&WaveDetectionPublisher::handDestroyCallback), this, _1);
        handDestroySub = node.subscribe("/HandDestroy", 1000, m1);
        boost::function<void(const geometry_msgs::PointStampedConstPtr&) > m2 =
                bind(mem_fn(&WaveDetectionPublisher::handUpdateCallback), this, _1);
        handUpdateSub = node.subscribe("/HandUpdate", 1000, m2);
        tfListener = new tf::TransformListener(node);

        ROS_INFO("WaveDetectionPublisher created");
    }

    void WaveDetectionPublisher::handCreateCallback(const geometry_msgs::PointStampedConstPtr& msg) {
        transformAndPublish(msg->header.stamp.toNSec(), msg->point.x, msg->point.y, msg->point.z);
    }

    void WaveDetectionPublisher::handDestroyCallback(const geometry_msgs::PointStampedConstPtr& msg) {
        //transformAndPublish(msg->header.stamp.toNSec());    
    }

    void WaveDetectionPublisher::handUpdateCallback(const geometry_msgs::PointStampedConstPtr& msg) {
        transformAndPublish(msg->header.stamp.toNSec(), msg->point.x, msg->point.y, msg->point.z);
    }

    void WaveDetectionPublisher::transformAndPublish(unsigned long long int nsecTime, double x, double y, double z) {
        geometry_msgs::PointStamped pointOut;
        geometry_msgs::PointStamped pointIn;
        // Robot's x is Kinect's z and robot's y is Kinect's x
        //TODO change datatype to rst
        
        shared_ptr<geometry::Pose> data = shared_ptr<geometry::Pose>(new geometry::Pose());
        
        data->mutable_rotation()->set_qx(0);
        data->mutable_rotation()->set_qy(0);
        data->mutable_rotation()->set_qz(0);
        data->mutable_rotation()->set_qw(0);
        
        
        data->mutable_translation()->set_x(z/1000.0);
        data->mutable_translation()->set_y(-x/1000.0);
        data->mutable_translation()->set_z(y/1000.0);

        rsb::EventPtr event = rsbInformer->createEvent();
		event->setData(data);
		this->publish(event);
    }
}
