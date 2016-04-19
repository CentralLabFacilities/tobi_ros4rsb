/*
 * NavigationServer.cpp
 *
 *  Created on: 18.03.2012 
 *      Author: leon, pdressel, prenner, cklarhor (rewrite on 25.04.14)
 * 
 */
#include "NavigationServer.h"
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Quaternion.h> 
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Geometry>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

#define WAYPOINTS_PER_MESSAGE 10

using namespace boost;
using namespace std;
using namespace ros;
using namespace tf;
using namespace rsb;
using namespace rsb::patterns;
using namespace rsb::converter;
using namespace rst::navigation;
using namespace rst::scene;
using namespace rst::generic;
using namespace actionlib;
using namespace move_base_msgs;

namespace ros4rsb {

class StopCb: public LocalServer::Callback<void, void> {
    NavigationServer *server;
public:
    StopCb(NavigationServer *server) {
        this->server = server;
    }
    void call(const std::string&) {
        server->stop();
    }
};

class BlockingCb: public LocalServer::Callback<void, void> { //This is only a test callback to check the RSB parallel functionality
    NavigationServer *server;
public:
    BlockingCb(NavigationServer *server) {
        this->server = server;
    }
    void call(const std::string&) {
        ROS_INFO("call BlockingCallback\n");
        usleep(100000000);
        ROS_INFO("finish BlockingCallback\n");
    }
};

class ReconfigureNodeCb: public LocalServer::Callback<KeyValuePair, CommandResult> {
    NavigationServer *server;
public:
    ReconfigureNodeCb(NavigationServer *server) {
        this->server = server;
    }
    shared_ptr<CommandResult> call(const std::string&, shared_ptr<KeyValuePair> input) {
        shared_ptr<string> tmp(new string("/move_base"));
        return server->reconfigureNode(tmp, input);
    }
};

class MoveRelativeCb: public LocalServer::Callback<CoordinateCommand, CommandResult> {
    NavigationServer *server;
public:
    MoveRelativeCb(NavigationServer *server) {
        this->server = server;
    }
    shared_ptr<CommandResult> call(const std::string&, shared_ptr<CoordinateCommand> input) {
	ROS_INFO("CALLBACK: MoveRelative called");
        return server->moveTo(input, true);
    }
};

class NavigateRelativeCb: public LocalServer::Callback<CoordinateCommand, CommandResult> {
    NavigationServer *server;
public:
    NavigateRelativeCb(NavigationServer *server) {
        this->server = server;
    }
    shared_ptr<CommandResult> call(const std::string&, shared_ptr<CoordinateCommand> input) {
        return server->navigateTo(input, true);
    }
};

class MoveToCoordinateCb: public LocalServer::Callback<CoordinateCommand, CommandResult> {
    NavigationServer *server;
public:
    MoveToCoordinateCb(NavigationServer *server) {
        this->server = server;
    }
    shared_ptr<CommandResult> call(const std::string&, shared_ptr<CoordinateCommand> input) {
        return server->moveTo(input, false);
    }
};

class NavigateToCoordinateCb: public LocalServer::Callback<CoordinateCommand, CommandResult> {
    NavigationServer *server;
public:
    NavigateToCoordinateCb(NavigationServer *server) {
        this->server = server;
    }
    shared_ptr<CommandResult> call(const std::string&, shared_ptr<CoordinateCommand> input) {
        return server->navigateTo(input, false);
    }
};

class GetPathToCb: public LocalServer::Callback<CoordinateCommand, Path> {
    NavigationServer *server;
public:
    GetPathToCb(NavigationServer *server) {
        this->server = server;
    }
    shared_ptr<Path> call(const std::string&, shared_ptr<CoordinateCommand> input) {
        return server->getPathTo(input, false);
    }
};

class GetCostCb: public LocalServer::Callback<CoordinateCommand, int64_t> {
    NavigationServer *server;
public:
    GetCostCb(NavigationServer *server) {
        this->server = server;
    }
    shared_ptr<int64_t> call(const std::string&, shared_ptr<CoordinateCommand> input) {
        return server->getCostGlobal(input);
    }
};

class GetPathToCoordinateLocal: public LocalServer::Callback<CoordinateCommand, Path> {
    NavigationServer *server;
public:
    GetPathToCoordinateLocal(NavigationServer *server) {
        this->server = server;
    }
    shared_ptr<Path> call(const std::string&, shared_ptr<CoordinateCommand> input) {
        return server->getPathTo(input, false);
    }
};

NavigationServer::NavigationServer(const std::string &name, ros::NodeHandle &node) {
    this->name = name;
    this->node = node;
    this->tfListener = new tf::TransformListener(node);
    this->velocityCommander = new VelocityCommander("VelocityCommander", node);
    this->defaultMoveSpeed = 0.5;
    this->defaultTurnSpeed = 0.5;
    this->isGoalActive = false;
    this->stopping = false;
    lastTime = ros::Time::now();
    ROS_INFO("starting navigation server");

    //tell the action client that we want to spin a thread by default
    moveBaseClient = new SimpleActionClient<MoveBaseAction>("/move_base", true);

    function<void(const nav_msgs::Odometry::ConstPtr&)> m1 = bind(
            mem_fn(&NavigationServer::poseCallback), this, _1);
    rosSubscriber = node.subscribe("/pose", 1000, m1);

    //wait for the action server to come up
    ROS_INFO("waiting for move_base action server...");
    for (int attempt = 0; attempt < 5; attempt++) {
        if (!moveBaseClient->waitForServer(ros::Duration(5.0))) {
            ROS_WARN("move_base action server is not present yet");
        } else {
            break;
        }
    }
    if (moveBaseClient->isServerConnected()) {
        ROS_INFO("move_base action server ready");
    } else {
        ROS_FATAL("move_base action server failed to connect!");
        throw ros::Exception("move_base action server failed to connect!");
    }
    costmap = new Costmap("/move_base/local_costmap", node);

    // Create a client for communicating with the move_base dynamic_reconfiguration server
    drcclient_local_costmap = node.serviceClient<dynamic_reconfigure::Reconfigure>(
            "/move_base/local_costmap/set_parameters");
    drcclient_global_costmap = node.serviceClient<dynamic_reconfigure::Reconfigure>(
            "/move_base/global_costmap/set_parameters");
    drcclient_move_base = node.serviceClient<dynamic_reconfigure::Reconfigure>(
            "/move_base/set_parameters");
    drcclient_trajectory_planner = node.serviceClient<dynamic_reconfigure::Reconfigure>(
            "/move_base/TrajectoryPlannerROS/set_parameters");
    clientGetPlan = node.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan");

    //register rsb converters
    Factory& factory = rsb::getFactory();

    boost::shared_ptr<ProtocolBufferConverter<CoordinateCommand> > converter(
            new ProtocolBufferConverter<CoordinateCommand>());
    boost::shared_ptr<ProtocolBufferConverter<CommandResult> > converter1(
            new ProtocolBufferConverter<CommandResult>());
    boost::shared_ptr<ProtocolBufferConverter<KeyValuePair> > converter2(
            new ProtocolBufferConverter<KeyValuePair>());

    converterRepository<string>()->registerConverter(converter);
    converterRepository<std::string>()->registerConverter(converter1);
    converterRepository<std::string>()->registerConverter(converter2);

    //set parallel handler hack
    ParticipantConfig config = factory.getDefaultParticipantConfig();
    rsc::runtime::Properties &properties = config.mutableEventReceivingStrategy().mutableOptions();
    properties.set<string>("parallelhandlercalls", "1");

    server = factory.createLocalServer(this->name, config); // config);

    ROS_INFO_STREAM("Server name: " << name);
    server->registerMethod("stop", LocalServer::CallbackPtr(new StopCb(this)));
    server->registerMethod("blocking", LocalServer::CallbackPtr(new BlockingCb(this)));
    server->registerMethod("moveRelative", LocalServer::CallbackPtr(new MoveRelativeCb(this)));
    server->registerMethod("moveToCoordinate", LocalServer::CallbackPtr(new MoveToCoordinateCb(this)));
    server->registerMethod("navigateToCoordinate", LocalServer::CallbackPtr(new NavigateToCoordinateCb(this)));
    server->registerMethod("navigateRelative", LocalServer::CallbackPtr(new NavigateRelativeCb(this)));
    server->registerMethod("getPathToCoordinate", LocalServer::CallbackPtr(new GetPathToCb(this)));
    server->registerMethod("getPathToCoordinateLocal", LocalServer::CallbackPtr(new GetPathToCoordinateLocal(this)));
    server->registerMethod("reconfigureNode", LocalServer::CallbackPtr(new ReconfigureNodeCb(this)));
    server->registerMethod("getCostGlobal", LocalServer::CallbackPtr(new GetCostCb(this)));
    ROS_INFO("registered all methods\n");

}

void NavigationServer::poseCallback(
        const nav_msgs::Odometry::ConstPtr &message) {

    boost::mutex::scoped_lock lock(poseMutex);

    geometry_msgs::PoseStamped poseOut;
    geometry_msgs::PoseStamped poseIn;
    poseIn.pose = message->pose.pose;
    poseIn.header.frame_id = "/odom";
    try {
        tfListener->transformPose("/map", poseIn, poseOut);
        this->currentPose = poseOut.pose;
    } catch (tf::TransformException &e) {
        ROS_ERROR("No transform!");
    }
}

void NavigationServer::stop() {
    ROS_INFO("call stop\n");
    if (this->moveBaseClient->getState() == SimpleClientGoalState::ACTIVE) {
        stopping = true;
        moveBaseClient->cancelAllGoals();
        while (moveBaseClient->getState() == SimpleClientGoalState::ACTIVE) {
            ROS_INFO("Wait for stop!!!");
            usleep(2500);
        }

    } else {
        ROS_WARN("called stop but there is no goal running? stopping velocity commander");
        this->velocityCommander->stop();
		usleep(500);

    }
    ROS_INFO("called stop\n");

}

shared_ptr<CommandResult> NavigationServer::navigateTo(shared_ptr<CoordinateCommand> coor,
        bool relative) {
    ROS_INFO("call navigate\n");
    stopping = false;
    move_base_msgs::MoveBaseGoal mbGoal;
    mbGoal.target_pose.header.stamp = ros::Time::now();
    if (relative)
        // set map coordinates
        mbGoal.target_pose.header.frame_id = "base_link";
    else
        mbGoal.target_pose.header.frame_id = "map";
    ROS_INFO_STREAM("Driving to x:" << coor->mutable_goal()->mutable_translation()->x() << ",y: "
            << coor->mutable_goal()->mutable_translation()->y() << "relative: " << relative);
    mbGoal.target_pose.pose.position.x = coor->mutable_goal()->mutable_translation()->x();
    mbGoal.target_pose.pose.position.y = coor->mutable_goal()->mutable_translation()->y();
    mbGoal.target_pose.pose.orientation.x = coor->mutable_goal()->mutable_rotation()->qx();
    mbGoal.target_pose.pose.orientation.y = coor->mutable_goal()->mutable_rotation()->qy();
    mbGoal.target_pose.pose.orientation.z = coor->mutable_goal()->mutable_rotation()->qz();
    mbGoal.target_pose.pose.orientation.w = coor->mutable_goal()->mutable_rotation()->qw();

    SimpleClientGoalState state = moveBaseClient->sendGoalAndWait(mbGoal, ros::Duration(0.0), ros::Duration(0.0));
    shared_ptr<CommandResult> result(new CommandResult());

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
    ROS_INFO("called navigate\n");
    return result;
}

shared_ptr<CommandResult> NavigationServer::moveTo(shared_ptr<CoordinateCommand> coor,
        bool relative) {
    ROS_INFO_STREAM("called moveTo, relative:" << relative);
    ROS_INFO_STREAM("coor:" << coor);
    stop();
    if (!relative) {
        ROS_INFO("fail not impl");
        shared_ptr<CommandResult> result(new CommandResult());
        result->set_type(CommandResult_Result_CUSTOM_ERROR);
        result->set_description("not impl");
        return result;
    }
    move_base::MoveBaseConfig move_base_config;
    double v_theta;
	if (coor->has_motion_parameters() && coor->motion_parameters().has_max_acceleration()) {
		v_theta = coor->motion_parameters().max_acceleration();
    } else {
        v_theta = this->defaultTurnSpeed;
    }
    double v_x;
    if (coor->has_motion_parameters() && coor->motion_parameters().has_max_velocity()) {
        v_x = coor->motion_parameters().max_velocity();
    } else {
        v_x = this->defaultMoveSpeed;
    }
    const rst::geometry::Rotation r = coor->mutable_goal()->rotation();
    tfScalar roll, pitch, yaw;
    tf::Quaternion q = tf::Quaternion(r.qx(), r.qy(), r.qz(), r.qw());
    tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
    ROS_INFO_STREAM("yaw: " << yaw << ",pitch: " << pitch << ",roll: " << roll);
    //our angle is pitch its around y axis
    ROS_INFO("moveTo: before turn\n");
    ExitStatus status = this->velocityCommander->turn(yaw, v_theta);
    ROS_INFO("moveTo: after turn\n");
	if (status == SUCCESS && coor->mutable_goal()->mutable_translation()->x() != 0) {
		ROS_INFO("before drive\n");
        status = this->velocityCommander->drive(coor->mutable_goal()->mutable_translation()->x(),
                v_x);
    }
    ROS_INFO("moveTo: after drive\n");
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
    ROS_INFO("moveTo:  finished\n");
    return result;
}

shared_ptr<Path> NavigationServer::getPathTo(shared_ptr<CoordinateCommand> coor, bool relative) {
    ROS_INFO("call getPathTo\n");
    nav_msgs::GetPlan srv;
    {
        boost::mutex::scoped_lock lock(poseMutex);
        srv.request.start.pose = currentPose;
    }
    ROS_INFO_STREAM("start pose: " << srv.request.start.pose);
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
    ROS_INFO_STREAM("search for plan with tol: " << srv.request.tolerance << ", x:"
            << srv.request.goal.pose.position.x << ",y=" << srv.request.goal.pose.position.y);
    shared_ptr<Path> path(new Path());
    ROS_INFO("calling getPlan...");
    if (clientGetPlan.call(srv)) {
        // skip some points for efficiency
        unsigned int step = ceil(srv.response.plan.poses.size() / WAYPOINTS_PER_MESSAGE);
        if (step == 0)
            ++step;
        for (unsigned int i = 0; i < srv.response.plan.poses.size(); i += step) {
            ROS_INFO_STREAM("loop " << i);
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
        ROS_INFO_STREAM("got plan...");
    } else {
        ROS_INFO_STREAM("no plan...");
    }
    return path;
    ROS_INFO("called getPathTo\n");
}

shared_ptr<int64_t> NavigationServer::getCostGlobal(shared_ptr<CoordinateCommand> coor) {
    ROS_INFO("call getCostGlobal\n");
    unsigned int mapX;
    unsigned int mapY;
    if (!costmap->costmap->worldToMap(coor->mutable_goal()->mutable_translation()->x(),
            coor->mutable_goal()->mutable_translation()->y(), mapX, mapY)) {
        ROS_ERROR_STREAM("Error: request get cost for out of costmap coord!!! -> return HIGH costs (X: "
                << mapX << ",Y:" << mapY);
        shared_ptr<int64_t> costPtr(new int64_t((int64_t) 99));
        return costPtr;
    }
    unsigned char cost = costmap->costmap->getCost(mapX, mapY);
    ROS_INFO_STREAM("cell coor: X:" << mapX << ", Y: " << mapY << ", cost:" << cost);
    shared_ptr<int64_t> costPtr(new int64_t((int64_t) cost));
    ROS_INFO("called getCostGlobal\n");
    return costPtr;
}

NavigationServer::~NavigationServer() {
    delete moveBaseClient;
    delete this->velocityCommander;
}

shared_ptr<CommandResult> NavigationServer::reconfigureNode(shared_ptr<string> node,
        shared_ptr<KeyValuePair> key) {
    ROS_INFO_STREAM("call reconfigureNode with node=" << node << " key=" << key);

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

shared_ptr<CommandResult> NavigationServer::reconfigureBool(shared_ptr<string> nodeName,
        shared_ptr<KeyValuePair> key) {
    string nodeID(nodeName->data());
    nodeID.append("/set_parameters");
    ros::ServiceClient nodeClient = this->node.serviceClient<dynamic_reconfigure::Reconfigure>(
            nodeID);

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
    ROS_INFO("call deleteTarget\n");
    ROS_INFO("called deleteTarget\n");
}

bool NavigationServer::isLocalized() {
    ROS_INFO("call isLocalized\n");
    ROS_INFO("called isLocalized\n");
    return true;
}

shared_ptr<PlatformCapabilities> NavigationServer::getCapabilities() {
    ROS_INFO("call getCapabilities\n");
    shared_ptr<PlatformCapabilities> result(new PlatformCapabilities());
    ROS_INFO("called getCapabilities\n");
    return result;
}

void NavigationServer::trainTarget(shared_ptr<string> target) {
    ROS_INFO("call trainTarget\n");
    ROS_INFO("called trainTarget\n");
}

void NavigationServer::defineTarget(shared_ptr<SceneObject> target) {
    ROS_INFO("call defineTarget\n");
    ROS_INFO("called defineTarget\n");
}

}
