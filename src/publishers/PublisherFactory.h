/*
 * PublisherFactory.h
 *
 *  Created on: Sep 6, 2016
 *      Author: plueckin
 */
#pragma once

#include "SlamPosPublisher.h"
#include "GlobalPlanPublisher.h"
#include "LaserDataPublisher.h"
#include "PersonDataPublisher.h"
#include "OdometryDataPublisher.h"
#include "SpeedDataPublisher.h"
#include "StallDataPublisher.h"
#include "SlamMapPublisher.h"
#include "GlobalPlannerStatePublisher.h"
#include "WaveDetectionPublisher.h"

namespace ros4rsb {

class PublisherFactory {
public:
   static Publisher::Ptr build(const std::string name, const std::string &topicIn, const std::string &scopeOut, ros::NodeHandle &node) {

       ROS_INFO_STREAM("Building publisher " << name << ", topic: " << topicIn << ", scope: " << scopeOut);

       if(name == "WaveDetectionPublisher")
           return WaveDetectionPublisher::Ptr(new WaveDetectionPublisher(topicIn, scopeOut, node));
       if(name == "GlobalPlanPublisher")
           return GlobalPlanPublisher::Ptr(new GlobalPlanPublisher(topicIn, scopeOut, node));
       if(name == "LaserDataPublisher")
           return LaserDataPublisher::Ptr(new LaserDataPublisher(topicIn, scopeOut, node));
       if(name == "OdometryDataPublisher")
           return OdometryDataPublisher::Ptr(new OdometryDataPublisher(topicIn, scopeOut, node));
       if(name == "SlamMapPublisher")
           return SlamMapPublisher::Ptr(new SlamMapPublisher(topicIn, scopeOut, node));
       if(name == "SlamPosPublisher")
           return SlamPosPublisher::Ptr(new SlamPosPublisher(topicIn, scopeOut, node));
       if(name == "SpeedDataPublisher")
           return SpeedDataPublisher::Ptr(new SpeedDataPublisher(topicIn, scopeOut, node));
       if(name == "StallDataPublisher")
           return StallDataPublisher::Ptr(new StallDataPublisher(topicIn, scopeOut, node));
       if(name == "PersonDataPublisher")
           return PersonDataPublisher::Ptr(new PersonDataPublisher(topicIn, scopeOut, node));

       ROS_ERROR_STREAM("No publisher found.");

       return Publisher::Ptr();
    }

};

}

