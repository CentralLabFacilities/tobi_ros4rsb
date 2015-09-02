
#include "GlobalPlannerStatePublisher.h"

using namespace std;
using namespace ros;
using namespace boost;
using namespace rst::navigation;

namespace ros4rsb {

    GlobalPlannerStatePublisher::GlobalPlannerStatePublisher(const string &topicIn,
            const std::string &name,
            ros::NodeHandle &node) {
        function<void(const actionlib_msgs::GoalStatusArrayConstPtr&) > m0 = bind(
                mem_fn(&GlobalPlannerStatePublisher::callback), this, _1);

        rosSubscriber = node.subscribe("/move_base/status", 1000, m0);
        ROS_INFO_STREAM(name << " subscribed to /move_base/status topic.");
    }
    
    void setActiveGoal(actionlib_msgs::GoalID goalID) {
        
    }
    
    void GlobalPlannerStatePublisher::publishPlannerState() {
        // create correct timestamp
        uint32_t sec = 0; //message->header.stamp.sec;
        uint32_t nsec = 0; //message->header.stamp.nsec;
        uint32_t usec = nsec / 1000;
	uint64_t timestamp = sec * 1000000 + usec;

        shared_ptr<string> data = shared_ptr<string>(new string());

        actionlib::SimpleClientGoalState::StateEnum state = this->lastState;

        bool publish = true;
        switch (state) {
            case actionlib::SimpleClientGoalState::PENDING:
//                globalPlannerStateChanged.setNewState(
//                        GlobalPlannerStateChange::PLANNING);
                data->append("PLANNING");
                //ROS_INFO("PENDING");
                break;
            case actionlib::SimpleClientGoalState::ACTIVE:
//                globalPlannerStateChanged.setNewState(
//                        GlobalPlannerStateChange::FOLLOWING_PLAN);
                data->append("FOLLOWING_PLAN");
                //ROS_INFO("ACTIVE");
                break;
            case actionlib::SimpleClientGoalState::REJECTED:
                publish = false;
                //ROS_INFO("REJECTED");
                break;
            case actionlib::SimpleClientGoalState::LOST:
//                globalPlannerStateChanged.setNewState(
//                        GlobalPlannerStateChange::PLANNING);
                data->append("PLANNING");
                //ROS_INFO("LOST");
                break;
            case actionlib::SimpleClientGoalState::ABORTED:
                //ROS_INFO("ABORTED");
                publish = false;
                break;
            case actionlib::SimpleClientGoalState::PREEMPTED:
                //ROS_INFO("PREEMPTED");
//                globalPlannerStateChanged.setNewState(
//                        GlobalPlannerStateChange::PATH_BLOCKED);
                data->append("BLOCKED");
                break;
            case actionlib::SimpleClientGoalState::RECALLED:
                //ROS_INFO("RECALLED");
                publish = false;
                break;
            case actionlib::SimpleClientGoalState::SUCCEEDED:
                //ROS_INFO("SUCCEEDED");
                publish = false;
                break;
            default:
                publish = false;
        }

        if (publish) {
            rsb::EventPtr event = rsbInformer->createEvent();
            event->setData(data);
            event->mutableMetaData().setCreateTime(timestamp);
            this->publish(event);
        }
    }
    
    void callback(const actionlib_msgs::GoalStatusArrayConstPtr &message) {
//        long newestGoalTime = LONG_MAX;
//        string id;
//        for (unsigned int i = 0; i < message->status_list.size(); ++i) {
//            long time = message->status_list[i].goal_id.stamp.sec * 1000
//                    + message->status_list[i].goal_id.stamp.nsec / 1000000;
//            if (time < newestGoalTime) {
//                newestGoalTime = time;
//                this->lastState = (actionlib::SimpleClientGoalState::StateEnum) message->status_list.back().status;
//                id = message->status_list.back().goal_id.id;
//            }
//        }
////        cout << "Publishing status of goal " << id << endl;
//        publishPlannerState();
    }
    
}
