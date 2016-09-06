/*
 * ListenerFactory.h
 *
 *  Created on: Sep 6, 2016
 *      Author: plueckin
 */
#pragma once

#include "BoxListener.h"
#include "CollisionBoxListener.h"
#include "CollisionSurfaceListener.h"

namespace ros4rsb {

class ListenerFactory {
public:
   static Listener::Ptr build(const std::string name, const std::string &scopeIn, const std::string &topicOut, ros::NodeHandle &node) {

       ROS_INFO_STREAM("Building listener " << name << ", scope: " << scopeIn << ", topic: " << topicOut);

       if(name == "BoxListener")
           return BoxListener::Ptr(new BoxListener(scopeIn, topicOut, node));
       if(name == "CollisionBoxListener")
           return CollisionBoxListener::Ptr(new CollisionBoxListener(scopeIn, topicOut, node));
       if(name == "CollisionSurfaceListener")
           return CollisionSurfaceListener::Ptr(new CollisionSurfaceListener(scopeIn, topicOut, node));

       ROS_ERROR_STREAM("No listener found.");

       return Listener::Ptr();
    }

};

}

