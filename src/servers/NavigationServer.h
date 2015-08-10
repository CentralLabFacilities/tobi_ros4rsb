/*
 * NavigationServer.h
 *
 *  Created on: 18.03.2012
 *      Author: leon, pdressel, prenner
 */

#ifndef NAVIGATIONSERVER_H_
#define NAVIGATIONSERVER_H_

#include "ServerException.h"

#include <vector>
#include <boost/cstdint.hpp>
#include "../publishers/SlamPosPublisher.h"
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

namespace ros4rsb {

    class NavigationServer {
    public:
        NavigationServer(
                std::string name,
                ros::NodeHandle node,
                SlamPosPublisher *slamPosPublisher,
                bool isLocalNavigation = false);
        ~NavigationServer();

        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>* getMoveBaseClient() {
            return this->moveBaseClient;
        }
        void stop();
        void trainTarget(boost::shared_ptr<std::string> target);
        void defineTarget(boost::shared_ptr<rst::scene::SceneObject> target);
        void deleteTarget(boost::shared_ptr<std::string> target);
        boost::shared_ptr<rst::navigation::CommandResult> moveTo(boost::shared_ptr<rst::navigation::CoordinateCommand> corr, bool relative);
        boost::shared_ptr<rst::navigation::CommandResult> navigateTo(boost::shared_ptr<rst::navigation::CoordinateCommand> corr, bool relative);
        boost::shared_ptr<rst::navigation::Path> getPathTo(boost::shared_ptr<rst::navigation::CoordinateCommand> coor, bool relative);
        boost::shared_ptr<rst::navigation::CommandResult> reconfigureNode(boost::shared_ptr<std::string> node, boost::shared_ptr<rst::generic::KeyValuePair> key);
        boost::shared_ptr<int64_t> getCostGlobal(boost::shared_ptr<rst::navigation::CoordinateCommand> corr);
        bool isLocalized();
        boost::shared_ptr<rst::navigation::PlatformCapabilities> getCapabilities();



    private:
        bool stopping;
        std::string name;
        ros::NodeHandle node;
        bool isLocalNavigation;
 	bool isGoalActive;
        double defaultTurnSpeed;
        double defaultMoveSpeed;
        ros::Time lastTime;
        ros4rsb::Costmap* costmap;
        SlamPosPublisher *slamPosPublisher;
        VelocityCommander *velocityCommander;
        rsb::patterns::LocalServerPtr server;
        ros::ServiceClient clientGetPlan;
        ros::ServiceClient drcclient_local_costmap;
        ros::ServiceClient drcclient_global_costmap;
        ros::ServiceClient drcclient_move_base;
        ros::ServiceClient drcclient_trajectory_planner;
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *moveBaseClient;
        boost::shared_ptr<rst::navigation::CommandResult> reconfigureBool(boost::shared_ptr<std::string> nodeName, boost::shared_ptr<rst::generic::KeyValuePair> key);

        /*
	Old functions:  
        void localCostmapConfigUpdateCallback(const dynamic_reconfigure::Config::ConstPtr& updated_config);
        void globalCostmapConfigUpdateCallback(const dynamic_reconfigure::Config::ConstPtr& updated_config);
        void moveBaseConfigUpdateCallback(const dynamic_reconfigure::Config::ConstPtr& updated_config);
        void trajectoryPlannerConfigUpdateCallback(const dynamic_reconfigure::Config::ConstPtr& updated_config);
        void drcSetLocalCostmap(const std::string &in, std::string &out);
        void drcGetLocalCostmap(const std::string &in, std::string &out);
        void drcSetGlobalCostmap(const std::string &in, std::string &out);
        void drcGetGlobalCostmap(const std::string &in, std::string &out);
        void drcSetMoveBase(const std::string &in, std::string &out);
        void drcGetMoveBase(const std::string &in, std::string &out);
        void drcSetTrajectoryPlanner(const std::string &in, std::string &out);
	*/
        };
    }
#endif /* NAVIGATIONSERVER_H_ */
