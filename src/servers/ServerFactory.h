/*
 * ServerFactory.h
 *
 *  Created on: Sep 6, 2016
 *      Author: plueckin
 */
#pragma once

#include "ControllerServer.h"
#include "NavigationServer.h"

namespace ros4rsb {

class ServerFactory {
public:
   static Server::Ptr build(const std::string name, const std::string &topicIn, const std::string &scopeOut, ros::NodeHandle &node) {

       ROS_INFO_STREAM("Building server " << name << ", topic: " << topicIn << ", scope: " << scopeOut);

       if(name == "NavigationServer")
           return NavigationServer::Ptr(new NavigationServer(topicIn, node));
       if(name == "ControllerServer")
           return ControllerServer::Ptr(new ControllerServer(topicIn, node));

       ROS_ERROR_STREAM("No server found.");

       return Server::Ptr();
    }

};

}

