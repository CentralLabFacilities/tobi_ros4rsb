#pragma once

#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <ros/ros.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rst/geometry/Pose.pb.h>
#include <string>

#include <rsb/Informer.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

/**
 * This is a abstract class defining the outline of every Publisher that wraps ROS topics to RST types.
 */
namespace ros4rsb {

class Server {
public:
    typedef boost::shared_ptr<Server> Ptr;
    virtual ~Server() {
    }
private:
};

}
