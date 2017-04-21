/*
 * NavigationServer.h
 *
 *  Created on: 18.03.2012
 *      Author: leon, pdressel, prenner
 */

#pragma once

#include "ServerException.h"

#include <vector>
#include <boost/cstdint.hpp>
#include "../actuators/VelocityCommander.h"
#include <costmap_2d/costmap_2d_ros.h>

#include <rsb/Factory.h>
#include <rst/navigation/CommandResult.pb.h>
#include <rst/navigation/LabelCommand.pb.h>
#include <rst/navigation/CoordinateCommand.pb.h>
#include <rst/navigation/Path.pb.h>
#include <rst/navigation/PlatformCapabilities.pb.h>
#include <rst/scene/SceneObjects.pb.h>
#include <rst/scene/SceneObject.pb.h>
#include <rst/generic/Value.pb.h>
#include <rst/generic/KeyValuePair.pb.h>

// Ros
#include <ros/ros.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/simple_client_goal_state.h>
#include <move_base/MoveBaseConfig.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <costmap_2d/Costmap2DConfig.h>
#include <costmap_2d/InflationPluginConfig.h>
#include <costmap_2d/VoxelPluginConfig.h>
#include <base_local_planner/BaseLocalPlannerConfig.h>
#include <dynamic_reconfigure/BoolParameter.h>

// Boost
#include <boost/thread/mutex.hpp>

#include "Costmap.h"
#include "Server.h"

namespace ros4rsb {

class NavigationServer: public Server {
public:
    NavigationServer(const std::string &name, ros::NodeHandle &node);
    ~NavigationServer();

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* getMoveBaseClient() {
        return this->moveBaseClient;
    }
    void stop();
    void trainTarget(boost::shared_ptr<std::string> target);
    void defineTarget(boost::shared_ptr<rst::scene::SceneObject> target);
    void deleteTarget(boost::shared_ptr<std::string> target);
    boost::shared_ptr<rst::navigation::CommandResult> moveTo(
            boost::shared_ptr<rst::navigation::CoordinateCommand> corr, bool relative);
    boost::shared_ptr<rst::navigation::CommandResult> navigateTo(
            boost::shared_ptr<rst::navigation::CoordinateCommand> corr, bool relative);
    boost::shared_ptr<rst::navigation::Path> getPathTo(
            boost::shared_ptr<rst::navigation::CoordinateCommand> coor, bool relative);
    boost::shared_ptr<rst::navigation::CommandResult> reconfigureNode(
            boost::shared_ptr<std::string> node, boost::shared_ptr<rst::generic::KeyValuePair> key);
    boost::shared_ptr<int64_t> getCostGlobal(
            boost::shared_ptr<rst::navigation::CoordinateCommand> corr);
    bool isLocalized();
    boost::shared_ptr<rst::navigation::PlatformCapabilities> getCapabilities();

    void poseCallback(const nav_msgs::Odometry::ConstPtr &message);

private:
    bool stopping;
    std::string name;
    ros::NodeHandle node;
    bool isGoalActive;
    double defaultTurnSpeed;
    double defaultMoveSpeed;
    ros::Time lastTime;
    tf::TransformListener *tfListener;
    ros4rsb::Costmap* costmap;
    VelocityCommander *velocityCommander;
    rsb::patterns::LocalServerPtr server;
    ros::Subscriber rosSubscriber;
    ros::ServiceClient clientGetPlan;
    ros::ServiceClient clientClearCostmap;
    ros::ServiceClient drcclient_local_costmap;
    ros::ServiceClient drcclient_global_costmap;
    ros::ServiceClient drcclient_move_base;
    ros::ServiceClient drcclient_trajectory_planner;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *moveBaseClient;
    boost::shared_ptr<rst::navigation::CommandResult> reconfigureBool(
    boost::shared_ptr<std::string> nodeName,
    boost::shared_ptr<rst::generic::KeyValuePair> key);

    geometry_msgs::Pose currentPose;
    boost::mutex poseMutex;

};
}
