#include "ros/ros.h"

#include "publishers/PublisherFactory.h"
#include "listeners/ListenerFactory.h"
#include "servers/ServerFactory.h"

#include <ros/ros.h>
#include <ros/param.h>
#include <rsb/Exception.h>

using namespace ros;
using namespace ros4rsb;
using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros4rsb");

    ros::NodeHandle n("~");
    XmlRpc::XmlRpcValue pub_list, lis_list, serv_list;

    // check params && get plugin lists from params
    if (!n.hasParam("publisher_list")) {
        ROS_WARN_STREAM("No controller_list specified.");
    } else {
        n.getParam("publisher_list", pub_list);
        if (pub_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("Parameter publisher_list should be specified as an array");
            return 1;
        }
    }
    if (!n.hasParam("listener_list")) {
        ROS_WARN_STREAM("No listener_list specified.");
    } else {
        n.getParam("listener_list", lis_list);
        if (lis_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("Parameter listener_list should be specified as an array");
            return 1;
        }
    }
    if (!n.hasParam("server_list")) {
        ROS_WARN_STREAM("No server_list specified.");
    } else {
        n.getParam("server_list", serv_list);
        if (serv_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_ERROR("Parameter server_list should be specified as an array");
            return 1;
        }
    }

    // Initialize publishers and servers
   try {

        /* actually create each publisher */
        vector<ros4rsb::Publisher::Ptr> publishers;
        if(pub_list.getType() != XmlRpc::XmlRpcValue::TypeInvalid) {
            for (int i = 0; i < pub_list.size(); ++i) {
                if (!pub_list[i].hasMember("name") || !pub_list[i].hasMember("topic") || !pub_list[i].hasMember("scope")) {
                    ROS_ERROR("Name, topic and scope must be specified for each publisher");
                    continue;
                }
                string name = string(pub_list[i]["name"]);
                string topic = string(pub_list[i]["topic"]);
                string scope = string(pub_list[i]["scope"]);

                ros4rsb::Publisher::Ptr pub = PublisherFactory::build(name, topic, scope, n);
                publishers.push_back(pub);
            }
        }

        ROS_INFO_STREAM(publishers.size() << " publishers created");

        /* actually create each listener */
        vector<ros4rsb::Listener::Ptr> listeners;
        if(lis_list.getType() != XmlRpc::XmlRpcValue::TypeInvalid) {
            for (int i = 0; i < lis_list.size(); ++i) {
                if (!lis_list[i].hasMember("name") || !lis_list[i].hasMember("topic") || !lis_list[i].hasMember("scope")) {
                    ROS_ERROR("Name, topic and scope must be specified for each listener");
                    continue;
                }
                string name = string(lis_list[i]["name"]);
                string topic = string(lis_list[i]["topic"]);
                string scope = string(lis_list[i]["scope"]);

                ros4rsb::Listener::Ptr listener = ListenerFactory::build(name, scope, topic, n);
                listeners.push_back(listener);
            }
        }

        ROS_INFO_STREAM(listeners.size() << " listener created");

        /* actually create each server */
        vector<ros4rsb::Server::Ptr> servers;
        if(serv_list.getType() != XmlRpc::XmlRpcValue::TypeInvalid) {
            for (int i = 0; i < serv_list.size(); ++i) {
                if (!serv_list[i].hasMember("name") || !serv_list[i].hasMember("topic") || !serv_list[i].hasMember("scope")) {
                    ROS_ERROR("Name, topic and scope must be specified for each server");
                    continue;
                }
                string name = string(serv_list[i]["name"]);
                string topic = string(serv_list[i]["topic"]);
                string scope = string(serv_list[i]["scope"]);

                ros4rsb::Server::Ptr server = ServerFactory::build(name, scope, n);
                servers.push_back(server);
            }
        }

        ROS_INFO_STREAM(servers.size() << " server created");

        cout << endl << "ROS4RSB is RUNNING (:" << endl;

        ros::spin();

    } catch (rsb::Exception &e) {
        ROS_FATAL_STREAM("Can not initialize rsb communication!" << endl);
        exit(1);
    }

    return 0;
}
