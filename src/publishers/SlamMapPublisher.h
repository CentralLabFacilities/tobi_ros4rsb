/* 
 * File:   SlamMapPublisher.h
 * Author: alangfel
 *
 * Created on April 25, 2014, 2:02 PM
 */

#ifndef SLAMMAPPUBLISHER_H
#define	SLAMMAPPUBLISHER_H

#include "Publisher.h"

// ROS
#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"

// Boost
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <rst/navigation/OccupancyGrid2DInt.pb.h>

namespace ros4rsb {

    class SlamMapPublisher : public PublisherImpl<rst::navigation::OccupancyGrid2DInt> {
    public:
        SlamMapPublisher(const std::string &topicIn,const std::string &name, ros::NodeHandle &node);
        virtual ~SlamMapPublisher();
        // TODO: Change the message this Publisher is listening to
        void callback(const nav_msgs::OccupancyGridConstPtr &message);

        void publishThread();

        CREATE_PUBLISHER_BUILDER_NESTED(SlamMapPublisher)

    private:
        boost::mutex mutex;
        boost::thread runner;
        bool hasData;
        ros::ServiceClient serviceClient;
        boost::shared_ptr<rst::navigation::OccupancyGrid2DInt> slamMap;
    };
}
#endif	/* SLAMMAPPUBLISHER_H */

