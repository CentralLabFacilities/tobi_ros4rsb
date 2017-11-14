//
// Created by lruegeme on 31.05.16.
//

#include "ControllerServer.h"

//RSB
#include <rsb/Factory.h>
#include <rst/generic/Dictionary.pb.h>

typedef rst::generic::Dictionary Dict;
typedef rst::generic::KeyValuePair Kvp;

namespace ros4rsb {

    class ZliftCb : public rsb::patterns::LocalServer::Callback<float, bool> {
        ControllerServer *server;
    public:
        ZliftCb(ControllerServer *server) {
            this->server = server;
        }

        boost::shared_ptr<bool> call(const std::string &, boost::shared_ptr<float> input) {
            return boost::make_shared<bool>(server->zliftGoto(*input));
        }
    };

    class HeadCb : public rsb::patterns::LocalServer::Callback<Dict, bool> {
        ControllerServer *server;
    public:
        HeadCb(ControllerServer *server) {
            this->server = server;
        }

        boost::shared_ptr<bool> call(const std::string &, boost::shared_ptr<Dict> input) {
            float j0, j1;
            Dict in = *input;

            ROS_DEBUG_STREAM("got head input");
            in.PrintDebugString();

            for (Kvp kv : in.entries()) {
                if (kv.key() == "j0") {
                    j0 = kv.value().double_();
                } else {
                    j1 = kv.value().double_();
                }
            }
            return boost::make_shared<bool>(server->headGoto(j0, j1));
        }
    };

    ControllerServer::ControllerServer(const std::string &name, ros::NodeHandle &node) {
        this->name = name;
        this->node = node;

        ROS_INFO("starting controller server");

        //tell the action client that we want to spin a thread by default
        zliftClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(
                "/meka_roscontrol/zlift_position_trajectory_controller/follow_joint_trajectory", true);

        headClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(
                "/meka_roscontrol/head_position_trajectory_controller/follow_joint_trajectory", true);

        ROS_INFO("waiting for ros_control action server...");
        for (int attempt = 0; attempt < 5; attempt++) {
            if (!zliftClient->waitForServer(ros::Duration(5.0))) {
                ROS_WARN("zlift action server is not present yet");
            }
            if (!headClient->waitForServer(ros::Duration(5.0))) {
                ROS_WARN("head action server is not present yet");
            }
        }

        if (zliftClient->isServerConnected()) {
            ROS_INFO("zlift action server ready");
        } else {
            ROS_FATAL("zlift action server failed to connect!");
            //throw ros::Exception("zlift action server failed to connect!");
        }

        if (headClient->isServerConnected()) {
            ROS_INFO("head action server ready");
        } else {
            ROS_FATAL("head action server failed to connect!");
            //throw ros::Exception("head action server failed to connect!");
        }

        rsb::Factory &factory = rsb::getFactory();
        boost::shared_ptr<rsb::converter::ProtocolBufferConverter<Dict> > converter(
                new rsb::converter::ProtocolBufferConverter<Dict>());

        rsb::converter::converterRepository<std::string>()->registerConverter(converter);


        server = factory.createLocalServer(this->name);

        ROS_INFO_STREAM("Server name: " << name);
        server->registerMethod("setZliftPosition", rsb::patterns::LocalServer::CallbackPtr(new ZliftCb(this)));
        server->registerMethod("setHeadAngles", rsb::patterns::LocalServer::CallbackPtr(new HeadCb(this)));
        ROS_INFO("registered all methods\n");

    }

    ControllerServer::~ControllerServer() {

    }

    bool ControllerServer::zliftGoto(float in) {
        control_msgs::FollowJointTrajectoryGoal goal;

        goal.trajectory.joint_names.push_back("zlift_j0");

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(in);

        //TODO: make dependend on distance to move (in is only target not distance)
        point.time_from_start = ros::Duration(2.0);

        goal.trajectory.points.push_back(point);

        zliftClient->sendGoal(goal);
        bool finished_before_timeout = zliftClient->waitForResult(ros::Duration(30.0));

        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = zliftClient->getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
            return state.state_ == state.SUCCEEDED;
        } else {
            ROS_INFO("Action did not finish before the time out.");
            return false;
        }

    }

    bool ControllerServer::headGoto(float j0, float j1) {
        control_msgs::FollowJointTrajectoryGoal goal;

        goal.trajectory.joint_names.push_back("head_j0");
        goal.trajectory.joint_names.push_back("head_j1");

        trajectory_msgs::JointTrajectoryPoint p0;
        p0.positions.push_back(j0);
        p0.positions.push_back(j1);
        p0.time_from_start = ros::Duration(2.0);
        goal.trajectory.points.push_back(p0);


        headClient->sendGoal(goal);
        bool finished_before_timeout = headClient->waitForResult(ros::Duration(30.0));

        if (finished_before_timeout) {
            actionlib::SimpleClientGoalState state = headClient->getState();
            ROS_INFO("Action finished: %s", state.toString().c_str());
            return state.state_ == state.SUCCEEDED;
        } else {
            ROS_INFO("Action did not finish before the time out.");
            return false;
        }
    }


}
