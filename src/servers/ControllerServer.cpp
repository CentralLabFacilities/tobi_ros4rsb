//
// Created by lruegeme on 31.05.16.
//

#include "ControllerServer.h"

//RSB
#include <rsb/Factory.h>


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

    ControllerServer::ControllerServer(const std::string &name, ros::NodeHandle &node) {
        this->name = name;
        this->node = node;

        ROS_INFO("starting controller server");

        //tell the action client that we want to spin a thread by default
        zliftClient = new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>(
                "/meka_roscontrol/zlift_position_trajectory_controller", true);

        ROS_INFO("waiting for zlift action server...");
        for (int attempt = 0; attempt < 5; attempt++) {
            if (!zliftClient->waitForServer(ros::Duration(5.0))) {
                ROS_WARN("zlift action server is not present yet");
            } else {
                break;
            }
        }
        if (zliftClient->isServerConnected()) {
            ROS_INFO("zlift action server ready");
        } else {
            ROS_FATAL("zlift action server failed to connect!");
            throw ros::Exception("zlift action server failed to connect!");
        }

        rsb::Factory &factory = rsb::getFactory();
        server = factory.createLocalServer(this->name);

        ROS_INFO_STREAM("Server name: " << name);
        server->registerMethod("goTo", rsb::patterns::LocalServer::CallbackPtr(new ZliftCb(this)));
        ROS_INFO("registered all methods\n");

    }

    ControllerServer::~ControllerServer() {

    }

    bool ControllerServer::zliftGoto(float in) {
        control_msgs::FollowJointTrajectoryGoal goal;

        goal.trajectory.joint_names.push_back("zlift_j0");

        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.push_back(in);
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


}