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

class ServerBuilder {
public:
    typedef boost::shared_ptr<ServerBuilder> Ptr;
    ServerBuilder(const std::string &serverName): serverName(serverName) {
    }
    virtual std::string getServerName() const {
        return serverName;
    }
    virtual Server::Ptr build(const std::string &topicIn, const std::string &scopeOut, ros::NodeHandle &node) const = 0;
    virtual ~ServerBuilder() {
    }
protected:
    std::string serverName;
};

#define CREATE_SERVER_BUILDER_NESTED(SERVER_NAME) class Builder: public ServerBuilder {\
public:\
    Builder(const std::string &publisherName) :\
        ServerBuilder(publisherName) {\
    }\
    virtual Server::Ptr build(const std::string &topicIn, const std::string &scope, ros::NodeHandle &node) const {\
        ROS_INFO_STREAM("Building server " << serverName << ", topic: " << topicIn << ", scope: " << scope);\
        return Server::Ptr(new SERVER_NAME(scope, node));\
    }\
};

}
