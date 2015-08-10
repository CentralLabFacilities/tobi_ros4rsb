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
#include "servers/NavigationServer.h"
#include <ros/ros.h>
#include <ros/param.h>
#include <rsb/Exception.h>

using namespace ros;
using namespace ros4rsb;
using namespace std;

int main(int argc, char **argv) {

    ros::init(argc, argv, "ros4rsb");

    ros::NodeHandle n;
    bool isLocalNavigation;
    bool isInterleavedLaserData = false;

    ros::param::param<bool>("/isLocalNavigation", isLocalNavigation, true);
    //ros::param::param<bool>("/isInterleavedLaserData", isInterleavedLaserData, false);
    cout << "This version sets isInterleaved to false!!!" << endl;

    // Initialize publishers and servers
    SlamPosPublisher* slamPosPublisher;
    try {
        // Publisher
        LaserDataPublisher laserPublisher("/ros4rsb/laserscan", n,isInterleavedLaserData);
        OdometryDataPublisher odometryPublisher("/ros4rsb/odometryData", n);
        SpeedDataPublisher speedPublisher("/ros4rsb/speedData", n);
        StallDataPublisher stallPublisher("/ros4rsb/stallData", n);
        SlamMapPublisher slamMapPublisher("/ros4rsb/slamMap", n, isLocalNavigation);
        slamPosPublisher = new SlamPosPublisher("/ros4rsb/slampose", n, isLocalNavigation);
        WaveDetectionPublisher waveDetectionPublisher("/ros4rsb/handPos", n);
        GlobalPlanPublisher globalPlanPublisher("/ros4rsb/globalplan", n);

        // Listeners
        BoxListener boxListener("/boxes", "box_marker", n);

        // Server
        NavigationServer navigationServer("/nav_server", n, slamPosPublisher, isLocalNavigation);

        cout << endl << "ROS4RSB is RUNNING (:" << endl;

        ros::spin();

    } catch (rsb::Exception &e) {
        ROS_FATAL_STREAM("Can not initialize rsb communication!" << endl);
        exit(1);
    }

    delete slamPosPublisher;

    return 0;
}
