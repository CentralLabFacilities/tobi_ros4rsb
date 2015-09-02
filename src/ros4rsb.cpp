#include "ros/ros.h"

#include "publishers/SlamPosPublisher.h"
#include "publishers/GlobalPlanPublisher.h"
#include "publishers/LaserDataPublisher.h"
#include "publishers/OdometryDataPublisher.h"
#include "publishers/SpeedDataPublisher.h"
#include "publishers/StallDataPublisher.h"
#include "publishers/SlamMapPublisher.h"
#include "publishers/GlobalPlannerStatePublisher.h"
#include "publishers/WaveDetectionPublisher.h"
#include "listeners/BoxListener.h"
#include "listeners/CollisionBoxListener.h"
#include "listeners/CollisionSurfaceListener.h"
#include "servers/NavigationServer.h"
#include <ros/ros.h>
#include <ros/param.h>
#include <rsb/Exception.h>

using namespace ros;
using namespace ros4rsb;
using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros4rsb");

    vector<PublisherBuilder::Ptr> publisherBuilders;
    vector<ListenerBuilder::Ptr> listenerBuilders;
    vector<ServerBuilder::Ptr> serverBuilders;

    // register publishers
    publisherBuilders.push_back(PublisherBuilder::Ptr(new WaveDetectionPublisher::Builder("WaveDetectionPublisher")));
    publisherBuilders.push_back(PublisherBuilder::Ptr(new GlobalPlanPublisher::Builder("GlobalPlanPublisher")));
    publisherBuilders.push_back(PublisherBuilder::Ptr(new LaserDataPublisher::Builder("LaserDataPublisher")));
    publisherBuilders.push_back(PublisherBuilder::Ptr(new OdometryDataPublisher::Builder("OdometryDataPublisher")));
    publisherBuilders.push_back(PublisherBuilder::Ptr(new SlamMapPublisher::Builder("SlamMapPublisher")));
    publisherBuilders.push_back(PublisherBuilder::Ptr(new SlamPosPublisher::Builder("SlamPosPublisher")));
    publisherBuilders.push_back(PublisherBuilder::Ptr(new SpeedDataPublisher::Builder("SpeedDataPublisher")));
    publisherBuilders.push_back(PublisherBuilder::Ptr(new StallDataPublisher::Builder("StallDataPublisher")));

    // register listeners
    listenerBuilders.push_back(ListenerBuilder::Ptr(new BoxListener::Builder("BoxListener")));
    listenerBuilders.push_back(ListenerBuilder::Ptr(new CollisionBoxListener::Builder("CollisionBoxListener")));
    listenerBuilders.push_back(ListenerBuilder::Ptr(new CollisionSurfaceListener::Builder("CollisionSurfaceListener")));

    // register servers
    serverBuilders.push_back(ServerBuilder::Ptr(new NavigationServer::Builder("NavigationServer")));

    ros::NodeHandle n("~");

    // check params
    if (!n.hasParam("publisher_list")) {
        ROS_WARN_STREAM("No controller_list specified.");
    }
    if (!n.hasParam("listener_list")) {
        ROS_WARN_STREAM("No listener_list specified.");
    }
    if (!n.hasParam("server_list")) {
        ROS_WARN_STREAM("No server_list specified.");
    }

    // get plugin lists from params
    XmlRpc::XmlRpcValue pub_list, lis_list, serv_list;
    n.getParam("publisher_list", pub_list);
    n.getParam("listener_list", lis_list);
    n.getParam("server_list", serv_list);
    if (pub_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Parameter publisher_list should be specified as an array");
        return 1;
    }
    if (lis_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Parameter listener_list should be specified as an array");
        return 1;
    }
    if (serv_list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Parameter server_list should be specified as an array");
        return 1;
    }

    // Initialize publishers and servers
   try {

        /* actually create each publisher */
        vector<ros4rsb::Publisher::Ptr> publishers;
        for (int i = 0; i < pub_list.size(); ++i) {
            if (!pub_list[i].hasMember("name") || !pub_list[i].hasMember("topic") || !pub_list[i].hasMember("scope")) {
                ROS_ERROR("Name, topic and scope must be specified for each publisher");
                continue;
            }
            string name = string(pub_list[i]["name"]);
            string topic = string(pub_list[i]["topic"]);
            string scope = string(pub_list[i]["scope"]);
            bool success = false;
            for (vector<PublisherBuilder::Ptr>::iterator i = publisherBuilders.begin(); i !=  publisherBuilders.end(); ++i) {
                if ((**i).getPublisherName() == name) {
                    publishers.push_back((**i).build(topic, scope, n));
                    success = true;
                    break;
                }
            }
            if (!success) {
                ROS_ERROR_STREAM("No publisher found with name \"" << name << "\"");
                return 1;
            }
        }

        /* actually create each listener */
        vector<ros4rsb::Listener::Ptr> listeners;
        for (int i = 0; i < lis_list.size(); ++i) {
            if (!lis_list[i].hasMember("name") || !lis_list[i].hasMember("topic") || !lis_list[i].hasMember("scope")) {
                ROS_ERROR("Name, topic and scope must be specified for each listener");
                continue;
            }
            string name = string(lis_list[i]["name"]);
            string topic = string(lis_list[i]["topic"]);
            string scope = string(lis_list[i]["scope"]);
            bool success = false;
            for (vector<ListenerBuilder::Ptr>::iterator i = listenerBuilders.begin(); i !=  listenerBuilders.end(); ++i) {
                if ((**i).getListenerName() == name) {
                    listeners.push_back((**i).build(scope, topic, n));
                    success = true;
                    break;
                }
            }
            if (!success) {
                ROS_ERROR_STREAM("No listener found with name \"" << name << "\"");
                return 1;
            }
        }

        /* actually create each server */
        vector<ros4rsb::Server::Ptr> servers;
        for (int i = 0; i < serv_list.size(); ++i) {
            if (!serv_list[i].hasMember("name") || !serv_list[i].hasMember("topic") || !serv_list[i].hasMember("scope")) {
                ROS_ERROR("Name, topic and scope must be specified for each server");
                continue;
            }
            string name = string(serv_list[i]["name"]);
            string topic = string(serv_list[i]["topic"]);
            string scope = string(serv_list[i]["scope"]);
            bool success = false;
            for (vector<ServerBuilder::Ptr>::iterator i = serverBuilders.begin(); i !=  serverBuilders.end(); ++i) {
                if ((**i).getServerName() == name) {
                    servers.push_back((**i).build(topic, scope, n));
                    success = true;
                    break;
                }
            }
            if (!success) {
                ROS_ERROR_STREAM("No server found with name \"" << name << "\"");
                return 1;
            }
        }

        cout << endl << "ROS4RSB is RUNNING (:" << endl;

        ros::spin();

    } catch (rsb::Exception &e) {
        ROS_FATAL_STREAM("Can not initialize rsb communication!" << endl);
        exit(1);
    }

    return 0;
}
