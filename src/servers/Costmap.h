/*
 * NavigationServer.h
 *
 *  Created on: 18.03.2012
 *      Author: leon, pdressel, prenner
 */

#ifndef COSTMAP_H
#define COSTMAP_H

// Ros
 #include <costmap_2d/costmap_2d_ros.h>
#include "nav_msgs/OccupancyGrid.h"
#include <ros/ros.h>
#include <costmap_2d/Costmap2DConfig.h>

namespace ros4rsb {
    class Costmap {
    public:
        Costmap(std::string name, ros::NodeHandle node);
        void CostmapFullUpdate(const nav_msgs::OccupancyGridConstPtr &message);
        void CostmapPartialUpdate(const nav_msgs::OccupancyGridConstPtr &message);
        //~Costmap();
        costmap_2d::Costmap2D* costmap;
        std::string name;
        ros::NodeHandle node;
        ros::Subscriber rosSubscriberFull;
        ros::Subscriber rosSubscriberPartial;
    };
}
#endif 
