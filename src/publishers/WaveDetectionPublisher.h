/* 
 * File:   WaveDetectionPublisher.h
 * Author: alangfel
 *
 * Created on April 25, 2014, 2:37 PM
 */

#ifndef WAVEDETECTIONPUBLISHER_H
#define	WAVEDETECTIONPUBLISHER_H

#include <rst/geometry/Pose.pb.h>
#include "Publisher.h"
#include "waving_detection/HandCreate.h"
#include "waving_detection/HandUpdate.h"
#include "waving_detection/HandDestroy.h"
#include <tf/transform_listener.h>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

namespace ros4rsb {

    class WaveDetectionPublisher : public Publisher<rst::geometry::Pose>{
    public:
       WaveDetectionPublisher(std::string name, ros::NodeHandle node);

    void handCreateCallback(const waving_detection::HandCreateConstPtr& msg);
    
    void handUpdateCallback(const waving_detection::HandUpdateConstPtr& msg);
    
    void handDestroyCallback(const waving_detection::HandDestroyConstPtr& msg);
    
private:
    ros::Subscriber handCreateSub;
    ros::Subscriber handDestroySub;
    ros::Subscriber handUpdateSub;
    
    tf::TransformListener *tfListener;
    
    void transformAndPublish(unsigned long long int time, 
        double x,
        double y,
        double z);

    };

}
#endif	/* WAVEDETECTIONPUBLISHER_H */

