#pragma once

#include "Listener.h"

#include <ros/ros.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

/**
 * This is a abstract class defining the outline of every Publisher that wraps ROS topics to RST types.
 */
namespace ros4rsb {

template<class RsbType>
class ListenerScene: public ListenerImpl<RsbType> {
public:

    ListenerScene(const std::string &scopeIn) :
        ListenerImpl<RsbType>(scopeIn) {
	}

	virtual ~ListenerScene() {
	}

protected:
	moveit::planning_interface::PlanningSceneInterface sceneInterface;
};

}
