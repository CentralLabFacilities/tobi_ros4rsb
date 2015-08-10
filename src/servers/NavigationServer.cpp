/*
 * NavigationServer.cpp
 *
 *  Created on: 18.03.2012 
 *      Author: leon, pdressel, prenner, cklarhor (rewrite on 25.04.14)
 * 
 *  SOMEONE SHOULD UPDATE THIS FILE TO USE LOGGING INSTEAD OF PRINTF
 */
#include "NavigationServer.h"
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Quaternion.h> 
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Geometry>

#define WAYPOINTS_PER_MESSAGE 10

using namespace boost;
using namespace std;
using namespace ros;
using namespace tf;
using namespace rsb::patterns;
using namespace rst::navigation;
using namespace rst::scene;
using namespace rst::generic;
using namespace actionlib;

namespace ros4rsb {

    class StopCallback : public LocalServer::Callback<void, void> {
        NavigationServer *server;
    public:
        StopCallback(NavigationServer *server) {
            this->server = server;
        }
        void call(const std::string&) {
            server->stop();
        }
    };

    class BlockingCallback : public LocalServer::Callback<void, void> { //This is only a test callback to check the RSB parallel functionality
        NavigationServer *server;
    public:
        BlockingCallback(NavigationServer *server) {
            this->server = server;
        }
        void call(const std::string&) {
            printf("call BlockingCallback\n");
            usleep(100000000);
            printf("finish BlockingCallback\n");
        }
    };

    class ReconfigureNodeCallback : public LocalServer::Callback<KeyValuePair, CommandResult> {
        NavigationServer *server;
    public:
        ReconfigureNodeCallback(NavigationServer *server) {
            this->server = server;
        }
        shared_ptr<CommandResult> call(const std::string&, shared_ptr<KeyValuePair> input) {
            shared_ptr<string> tmp(new string("/move_base"));
            return server->reconfigureNode(tmp, input);
        }
    };

    class MoveRelativeCallback : public LocalServer::Callback<CoordinateCommand, CommandResult> {
        NavigationServer *server;
    public:
        MoveRelativeCallback(NavigationServer *server) {
            this->server = server;
        }
        shared_ptr<CommandResult> call(const std::string&, shared_ptr<CoordinateCommand> input) {
            return server->moveTo(input,true);
        }
    };

    class NavigateRelativeCallback : public LocalServer::Callback<CoordinateCommand, CommandResult> {
        NavigationServer *server;
    public:
        NavigateRelativeCallback(NavigationServer *server) {
            this->server = server;
        }
        shared_ptr<CommandResult> call(const std::string&, shared_ptr<CoordinateCommand> input) {
            return server->navigateTo(input,true);
        }
    };

    class MoveToCoordinateCallback : public LocalServer::Callback<CoordinateCommand, CommandResult> {
        NavigationServer *server;
    public:
        MoveToCoordinateCallback(NavigationServer *server) {
            this->server = server;
        }
        shared_ptr<CommandResult> call(const std::string&, shared_ptr<CoordinateCommand> input) {
            return server->moveTo(input,false);
        }
    };

    class NavigateToCoordinateCallback : public LocalServer::Callback<CoordinateCommand, CommandResult> {
        NavigationServer *server;
    public:
        NavigateToCoordinateCallback(NavigationServer *server) {
            this->server = server;
        }
        shared_ptr<CommandResult> call(const std::string&, shared_ptr<CoordinateCommand> input) {
            return server->navigateTo(input,false);
        }
    };

    class GetPathToCoordinate : public LocalServer::Callback<CoordinateCommand, Path> {
        NavigationServer *server;
    public:
        GetPathToCoordinate(NavigationServer *server) {
            this->server = server;
        }
        shared_ptr<Path> call(const std::string&, shared_ptr<CoordinateCommand> input) {
            return server->getPathTo(input,false);
        }
    };
    
     class GetCostCallback : public LocalServer::Callback<CoordinateCommand, int64_t> {
        NavigationServer *server;
    public:
        GetCostCallback(NavigationServer *server) {
            this->server = server;
        }
        shared_ptr<int64_t> call(const std::string&, shared_ptr<CoordinateCommand> input) {
            return server->getCostGlobal(input);
        }
    };
    
    class GetPathToCoordinateLocal : public LocalServer::Callback<CoordinateCommand, Path> {
        NavigationServer *server;
    public:
        GetPathToCoordinateLocal(NavigationServer *server) {
            this->server = server;
        }
        shared_ptr<Path> call(const std::string&, shared_ptr<CoordinateCommand> input) {
            return server->getPathTo(input,false);
        }
    };

    NavigationServer::NavigationServer(std::string name, ros::NodeHandle node, SlamPosPublisher* slamPosPublisher, bool isLocalNavigation) {
        this->name = name;
        this->node = node;
        this->slamPosPublisher = slamPosPublisher;
        this->velocityCommander = new VelocityCommander("VelocityCommander", node);
        this->isLocalNavigation = isLocalNavigation;
        this->defaultMoveSpeed = 0.5;
        this->defaultTurnSpeed = 0.5;
        this->isGoalActive = false;
        this->stopping = false;
        lastTime = ros::Time::now();
        ROS_INFO("starting navigation server");
        //tell the action client that we want to spin a thread by default
        moveBaseClient = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>(node, "move_base", true);
        //wait for the action server to come up
        while (!moveBaseClient->waitForServer(ros::Duration(5.0))) {
            ROS_INFO("Waiting for the move_base action server to come up");
            sleep(1);
        }
        ROS_INFO("move_base action server ready\n");
        costmap = new Costmap("/move_base/local_costmap",node);
        //costmap->start();
        // Create a client for communicating with the move_base dynamic_reconfiguration server
        drcclient_local_costmap = node.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/local_costmap/set_parameters");
        drcclient_global_costmap = node.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/global_costmap/set_parameters");
        drcclient_move_base = node.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/set_parameters");
        drcclient_trajectory_planner = node.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/TrajectoryPlannerROS/set_parameters");
        clientGetPlan = node.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");
        //register rsb converters
        rsb::Factory& factory = rsb::getFactory();
        boost::shared_ptr< rsb::converter::ProtocolBufferConverter<CoordinateCommand> >
                converter(new rsb::converter::ProtocolBufferConverter<CoordinateCommand>());
        rsb::converter::converterRepository<std::string>()->registerConverter(converter);
        
        boost::shared_ptr< rsb::converter::ProtocolBufferConverter<CommandResult> >
                converter1(new rsb::converter::ProtocolBufferConverter<CommandResult>());
        rsb::converter::converterRepository<std::string>()->registerConverter(converter1);

        boost::shared_ptr< rsb::converter::ProtocolBufferConverter<KeyValuePair> >
                converter2(new rsb::converter::ProtocolBufferConverter<KeyValuePair>());
        rsb::converter::converterRepository<std::string>()->registerConverter(converter2);
        
        //set parallel handler hack
        rsb::ParticipantConfig config = factory.getDefaultParticipantConfig();
        rsc::runtime::Properties &properties = config.mutableEventReceivingStrategy().mutableOptions();
        properties.set<string>("parallelhandlercalls", "1");
        
        server = factory.createLocalServer(this->name, config); // config);

        if (this->isLocalNavigation)
            ROS_INFO("publishing in base_link frame!\n");
        else
            ROS_INFO("publishing in map frame!\n");
        std::cout << name << std::endl;
        server->registerMethod("stop", rsb::patterns::LocalServer::CallbackPtr(new StopCallback(this)));
        server->registerMethod("blocking", rsb::patterns::LocalServer::CallbackPtr(new BlockingCallback(this)));
        server->registerMethod("moveRelative", rsb::patterns::LocalServer::CallbackPtr(new MoveRelativeCallback(this)));
        server->registerMethod("moveToCoordinate", rsb::patterns::LocalServer::CallbackPtr(new MoveToCoordinateCallback(this)));
        server->registerMethod("navigateToCoordinate", rsb::patterns::LocalServer::CallbackPtr(new NavigateToCoordinateCallback(this)));
        server->registerMethod("navigateRelative", rsb::patterns::LocalServer::CallbackPtr(new NavigateRelativeCallback(this)));
        server->registerMethod("getPathToCoordinate", rsb::patterns::LocalServer::CallbackPtr(new GetPathToCoordinate(this)));
        server->registerMethod("getPathToCoordinateLocal", rsb::patterns::LocalServer::CallbackPtr(new GetPathToCoordinateLocal(this)));
        server->registerMethod("reconfigureNode", rsb::patterns::LocalServer::CallbackPtr(new ReconfigureNodeCallback(this)));
        server->registerMethod("getCostGlobal", rsb::patterns::LocalServer::CallbackPtr(new GetCostCallback(this)));
        ROS_INFO("registered all methods\n");

    }

    void NavigationServer::stop() {
        ROS_INFO("call stop\n");
        if (this->moveBaseClient->getState()==SimpleClientGoalState::ACTIVE) {
	    stopping = true;
        moveBaseClient->cancelAllGoals();
        while (moveBaseClient->getState()==SimpleClientGoalState::ACTIVE) {
			ROS_INFO("Wait for stop!!!");
			usleep(2500);
		}
        
            
	} else {
            ROS_WARN("called stop but there is no goal running? stopping velocity commander");
            this->velocityCommander->stop();
            
        }
        ROS_INFO("called stop\n");

    }

    shared_ptr<CommandResult> NavigationServer::navigateTo(shared_ptr<CoordinateCommand> coor, bool relative) {
        ROS_INFO("call navigate\n");
        stopping = false;
        move_base_msgs::MoveBaseGoal mbGoal;
        mbGoal.target_pose.header.stamp = ros::Time::now();
        if (relative)
            // set map coordinates
            mbGoal.target_pose.header.frame_id = "base_link";
        else 
            mbGoal.target_pose.header.frame_id = "map";
        cout << "Driving to x:" << coor->mutable_goal()->mutable_translation()->x() << ",y: " << coor->mutable_goal()->mutable_translation()->y() << "relative: " <<relative;
        mbGoal.target_pose.pose.position.x = coor->mutable_goal()->mutable_translation()->x();
        mbGoal.target_pose.pose.position.y = coor->mutable_goal()->mutable_translation()->y();
        mbGoal.target_pose.pose.orientation.x = coor->mutable_goal()->mutable_rotation()->qx();
        mbGoal.target_pose.pose.orientation.y = coor->mutable_goal()->mutable_rotation()->qy();
        mbGoal.target_pose.pose.orientation.z = coor->mutable_goal()->mutable_rotation()->qz();
        mbGoal.target_pose.pose.orientation.w = coor->mutable_goal()->mutable_rotation()->qw();
        //moveBaseClient->cancelGoalsAtAndBeforeTime(lastTime - ros::Duration(1,0));
        //lastTime = ros::Time::now();
        
        actionlib::SimpleClientGoalState state = moveBaseClient->sendGoalAndWait(mbGoal, ros::Duration(0.0), ros::Duration(0.0));
        shared_ptr<CommandResult> result(new CommandResult());
        std::cout<<state.text_<<std::endl;
        switch (state.state_) {
            case SimpleClientGoalState::SUCCEEDED:
                result->set_type(CommandResult_Result_SUCCESS);
                break;
            case SimpleClientGoalState::PENDING:
				result->set_type(CommandResult_Result_PATH_BLOCKED);
                break;
            case SimpleClientGoalState::LOST:
                result->set_type(CommandResult_Result_CUSTOM_ERROR);
                result->set_code(1);
                break;
            case SimpleClientGoalState::ABORTED:
                if (stopping) {
                    result->set_type(CommandResult_Result_CANCELLED);
                    stopping = false;
                } else {
                    result->set_type(CommandResult_Result_PATH_BLOCKED);
                }
                break;
            case SimpleClientGoalState::PREEMPTED: //is that right?
                result->set_type(CommandResult_Result_SUPERSEDED);
                break;
            default:
                result->set_type(CommandResult_Result_UNKNOWN_ERROR);
                result->set_description(state.getText());
        }
        printf("called navigate\n");
        return result;
    }

    shared_ptr<CommandResult> NavigationServer::moveTo(shared_ptr<CoordinateCommand> coor, bool relative) {
        printf("call move\n");
        stop();
        if (!relative) {
            printf("fail not impl");
            shared_ptr<CommandResult> result(new CommandResult());
            result->set_type(CommandResult_Result_CUSTOM_ERROR);
            result->set_description("not impl");
            return result;
        }
        move_base::MoveBaseConfig move_base_config;
        //TODO: setAcceleration
        //TODO: use theta acceleration instead of velocity
        double v_theta;
        if (coor->has_motion_parameters() && coor->motion_parameters().has_max_velocity()) {
            v_theta = coor->motion_parameters().max_velocity();
	}
        else {
            v_theta = this->defaultTurnSpeed;
	}
        double v_x;
        if (coor->has_motion_parameters() && coor->motion_parameters().has_max_velocity()) {
            v_x = coor->motion_parameters().max_velocity();
	}
        else {
            v_x = this->defaultMoveSpeed;
	}
        const rst::geometry::Rotation r = coor->mutable_goal()->rotation();
        tfScalar roll, pitch, yaw;
        tf::Quaternion q = tf::Quaternion(r.qx(), r.qy(), r.qz(), r.qw());
        tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
        std::cout << "yaw: " <<yaw << ",pitch: " << pitch << ",roll: " << roll;
        //our angle is pitch its around y axis
        printf("before turn\n");
        ExitStatus status = this->velocityCommander->turn(yaw, v_theta);
        printf("after turn\n");
        printf("before drive\n");
        if (status == SUCCESS) {
            status = this->velocityCommander->drive(coor->mutable_goal()->mutable_translation()->x(), v_x);
        }
        printf("after drive\n");
        shared_ptr<CommandResult> result(new CommandResult());
        switch (status) {
            case CANCELLED:
                result->set_type(CommandResult_Result_CANCELLED);
                break;
            case NODE_BROKEN:
                result->set_type(CommandResult_Result_CUSTOM_ERROR);
                result->set_description("Node is broken");
                break;
            case SUCCESS:
                result->set_type(CommandResult_Result_SUCCESS);
                break;
            case SUPERSEDED:
                result->set_type(CommandResult_Result_SUPERSEDED);
                break;
            case TRANSFORM_ERROR:
                result->set_type(CommandResult_Result_CUSTOM_ERROR);
                result->set_description("Transformation exception see ROS_ERROR log");
                break;
            default:
                result->set_type(CommandResult_Result_UNKNOWN_ERROR);
                result->set_code(status);
                result->set_description("see error code");
        }
        printf("called move\n");
        return result;
    }

     shared_ptr<Path> NavigationServer::getPathTo(shared_ptr<CoordinateCommand> coor, bool relative) {
        printf("call getPathTo\n");
        nav_msgs::GetPlan srv;
        srv.request.start.pose = slamPosPublisher->getPose();
        cout << "start pose: " << srv.request.start.pose << endl;
        if (relative) 
            srv.request.goal.header.frame_id = "base_link";
        else
            srv.request.goal.header.frame_id = "map";
        srv.request.goal.pose.position.x = coor->mutable_goal()->mutable_translation()->x();
        srv.request.goal.pose.position.y = coor->mutable_goal()->mutable_translation()->y();
        srv.request.goal.pose.position.z = srv.request.start.pose.position.z;
        srv.request.goal.pose.orientation.x = coor->mutable_goal()->mutable_rotation()->qx();
        srv.request.goal.pose.orientation.y = coor->mutable_goal()->mutable_rotation()->qy();
        srv.request.goal.pose.orientation.z = coor->mutable_goal()->mutable_rotation()->qz();
        srv.request.goal.pose.orientation.w = coor->mutable_goal()->mutable_rotation()->qw();
        srv.request.tolerance = coor->mutable_motion_parameters()->translation_accuracy();
        std::cout << "search for plan with tol: " << srv.request.tolerance << ", x:" << srv.request.goal.pose.position.x << ",y=" << srv.request.goal.pose.position.y << std::endl;
        shared_ptr<Path> path(new Path());
        std::cout << "calling getPlan..." << std::endl;
        if (clientGetPlan.call(srv)) {
            // skip some points for efficiency
            unsigned int step = ceil(
                    srv.response.plan.poses.size() / WAYPOINTS_PER_MESSAGE);
            if (step == 0)
                ++step;
            for (unsigned int i = 0; i < srv.response.plan.poses.size(); i += step) {
                std::cout << "loop " << i << std::endl;
                geometry_msgs::Pose pose = srv.response.plan.poses[i].pose;
                rst::geometry::Pose *waypoint = path->mutable_poses()->Add();
                waypoint->mutable_translation()->set_x(pose.position.x);
                waypoint->mutable_translation()->set_y(pose.position.y);
                waypoint->mutable_translation()->set_z(pose.position.z);
                waypoint->mutable_rotation()->set_qw(pose.orientation.w);
                waypoint->mutable_rotation()->set_qx(pose.orientation.x);
                waypoint->mutable_rotation()->set_qy(pose.orientation.y);
                waypoint->mutable_rotation()->set_qz(pose.orientation.z);
            }
            std::cout << "got plan..." << std::endl;
        } else {
            std::cout << "no plan..." << std::endl;
        }
        return path;
        printf("called getPathTo\n");
    }
    
     shared_ptr<int64_t> NavigationServer::getCostGlobal(shared_ptr<CoordinateCommand> coor) {
        printf("call getCostGlobal\n");
        unsigned int mapX;
        unsigned int mapY;
        if (!costmap->costmap->worldToMap(coor->mutable_goal()->mutable_translation()->x(),coor->mutable_goal()->mutable_translation()->y(),mapX,mapY)) {
            std::cout << "Error: request get cost for out of costmap coord!!! -> return no costs (X: " << mapX << ",Y:" << mapY;
            shared_ptr<int64_t> costPtr(new int64_t((int64_t)0));
            return costPtr;
        }
        unsigned char cost = costmap->costmap->getCost(mapX,mapY);
        std::cout << "cell coor: X:" << mapX << ", Y: " << mapY << ", cost:" << cost;
        shared_ptr<int64_t> costPtr(new int64_t((int64_t)cost));
        printf("called getCostGlobal\n");
        return costPtr;
    }
   
    NavigationServer::~NavigationServer() {
        delete moveBaseClient;
        delete this->velocityCommander;
    }

    shared_ptr<CommandResult> NavigationServer::reconfigureNode(shared_ptr<string> node, shared_ptr<KeyValuePair> key) {
        std::cout << "call reconfigureNode with node=" << node << " key=" << key << std::endl;
        /*if (!(node->compare("/move_base"))) {
            shared_ptr<CommandResult> result(new CommandResult);
            result->set_type(CommandResult_Result_CUSTOM_ERROR);
            result->set_code(0);
            result->set_description("Node name is not in our list");
            return result;
        }*/

        switch (key->mutable_value()->type()) {
            case Value_Type_BOOL:
                return reconfigureBool(node, key);
            default:
                shared_ptr<CommandResult> result(new CommandResult);
                result->set_type(CommandResult_Result_CUSTOM_ERROR);
                result->set_code(1);
                result->set_description("There is no implementation that can set your type sry");
                return result;
        }
    }

    shared_ptr<CommandResult> NavigationServer::reconfigureBool(shared_ptr<string> nodeName, shared_ptr<KeyValuePair> key) {
        string nodeID(nodeName->data());
        nodeID.append("/set_parameters");
        ros::ServiceClient nodeClient = this->node.serviceClient<dynamic_reconfigure::Reconfigure>(nodeID);

        shared_ptr<CommandResult> result(new CommandResult);
        //get all config variables
        dynamic_reconfigure::Reconfigure reconfigure;
        nodeClient.call(reconfigure);

        //search for the key we want
        bool foundKey = false;
        for (uint i = 0; i < reconfigure.response.config.bools.size(); i++) {
            dynamic_reconfigure::BoolParameter param = reconfigure.response.config.bools[i];
            if (param.name.compare(key->key()) == 0) {
                foundKey = true;
                break;
            }
        }

        if (!foundKey) {
            result->set_type(CommandResult_Result_CUSTOM_ERROR);
            result->set_code(2); //TODO: change to enum
            result->set_description("Key not found");
            return result;
        }

        //set request
        dynamic_reconfigure::Reconfigure request;
        dynamic_reconfigure::BoolParameter paramToSet;
        paramToSet.name = key->key();
        paramToSet.value = key->value().bool_();
        request.request.config.bools.push_back(paramToSet);
        nodeClient.call(request);

        //Search again for key (now with our new value)
        foundKey = false;
        dynamic_reconfigure::BoolParameter param;
        for (uint i = 0; i < request.response.config.bools.size(); i++) {
            param = request.response.config.bools[i];
            if (param.name.compare(key->key()) == 0) {
                foundKey = true;
                break;
            }
        }
        if (!foundKey) {
            result->set_type(CommandResult_Result_CUSTOM_ERROR);
            result->set_code(3); //TODO: change to enum
            result->set_description("Key not found after set");
            return result;
        }

        //check that its the new value
        if (param.value != key->value().bool_()) {
            result->set_type(CommandResult_Result_CUSTOM_ERROR);
            result->set_code(4);
            result->set_description("Value was not set");
            return result;
        }

        result->set_type(CommandResult_Result_SUCCESS);
        return result;
    }

/* Currently unimplemented functions */
   
    void NavigationServer::deleteTarget(shared_ptr<string> target) {
        printf("call deleteTarget\n");
        printf("called deleteTarget\n");
    }

    bool NavigationServer::isLocalized() {
        printf("call isLocalized\n");
        printf("called isLocalized\n");
        return true;
    }

    shared_ptr<PlatformCapabilities> NavigationServer::getCapabilities() {
        printf("call getCapabilities\n");
        shared_ptr<PlatformCapabilities> result(new PlatformCapabilities());
        printf("called getCapabilities\n");
        return result;
    }

    void NavigationServer::trainTarget(shared_ptr<string> target) {
        printf("call trainTarget\n");
        printf("called trainTarget\n");
    }

    void NavigationServer::defineTarget(shared_ptr<SceneObject> target) {
        printf("call defineTarget\n");
        printf("called defineTarget\n");
    }

}
/*
NavigationServer::~NavigationServer() {
        delete moveBaseClient;

        this->drcclient_local_costmap.shutdown();
        this->drcclient_global_costmap.shutdown();
        this->drcclient_move_base.shutdown();
        this->drcclient_trajectory_planner.shutdown();

        delete this->velocityCommander;
        server->destroy();
}




void NavigationServer::goalReached(const std::string &in, std::string &out) {
        boost::mutex::scoped_lock l(goalLock);

        GoalReachedData grd;
        grd.setGenerator("ros4xcf");

        bool isGoalReached = false;
        if (this->activeGoal) {
                actionlib::SimpleClientGoalState state =
                                this->moveBaseClient->getState();
                isGoalReached = state == actionlib::SimpleClientGoalState::SUCCEEDED;
        }

        grd.setGoalReached(isGoalReached);

        // Print debug information
        if (isGoalReached) {
                ROS_INFO("Goal is reached!\n");
                this->moveBaseClient->stopTrackingGoal();
                this->activeGoal = false;
        }

        xmltio::Location locgrd(GoalReachedData::getTemplate(),
                        GoalReachedData::getBaseTag());
        xmltio::convert(grd, locgrd);
        out = locgrd.getDocumentText();
}





/*
 // As string [[x1,y1],..,[xn,yn]]
 void NavigationServer::setFootprint(const std::string &in, std::string &out) {

 // Extract the footprint data from xcf message
 //..

 // Wrap the new footprint data into a dynamic_reconfigure::Config
 dynamic_reconfigure::Config config;

 costmap_2d::Costmap2DConfig costmap_config;
 costmap_config.footprint = string.basic_string();

 costmap_config.__toMessage__(config);

 // Chose the client which is capable of sending the configuration to the coorect
 // dynamic_reconfigure server
 sendDynamicReconfiguration(this->dynamicReconfigurationClient, config);
 }

 // In meters [0.0 .. 50.0] (doubles)
 void NavigationServer::setInflationRadius(const std::string &in, std::string &out) {

 // Extract the inflation radius data from xcf message
 //..

 // Wrap the new inflation radius data into a dynamic_reconfigure::Config
 dynamic_reconfigure::Config config;

 costmap_2d::Costmap2DConfig costmap_config;
 costmap_config.inflation_radius = 2.0;

 costmap_config.__toMessage__(config);

 // Chose the client which is capable of sending the configuration to the coorect
 // dynamic_reconfigure server
 sendDynamicReconfiguration(this->dynamicReconfigurationClient, config);
 }

 // Escape velocity [-2.0 .. 2.0] (double)
 void NavigationServer::setEscapeVelocity(const std::string &in, std::string &out) {

 // Extract the escape velocity data from xcf message
 //..

 // Wrap the new escape velocity data into a dynamic_reconfigure::Config
 dynamic_reconfigure::Config config;

 base_local_planner::BaseLocalPlannerConfig base_local_planner_config;
 movebase_config. = 2.0;

 movebase_config.__toMessage__(config);

 // Chose the client which is capable of sending the configuration to the coorect
 // dynamic_reconfigure server
 sendDynamicReconfiguration(this->dynamicReconfigurationClient, config);
 }
 */
/*
void NavigationServer::wrapReconfiguration(
                const dynamic_reconfigure::Config &config,
                dynamic_reconfigure::Reconfigure &reconfigure) {
        // Wrap the configuration in a Reconfigure object
        dynamic_reconfigure::ReconfigureRequest request;
        dynamic_reconfigure::ReconfigureResponse response;

        request.config = config;

        reconfigure.request = request;
        reconfigure.response = response;
}

// Called once when the goal completes
void NavigationServer::doneCb(const actionlib::SimpleClientGoalState& state,
                const move_base_msgs::MoveBaseResultConstPtr &result) {
        //ROS_INFO("Finished in state [%s]", state.toString().c_str());
        //ROS_INFO("Answer: %i", result->status);
}

// Called once when the goal becomes active
void NavigationServer::activeCb() {
        this->activeGoal = true;
        ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void NavigationServer::feedbackCb(
                const move_base_msgs::MoveBaseFeedbackConstPtr &feedback) {
        //ROS_INFO("Got Feedback");
}

 
}
 */
