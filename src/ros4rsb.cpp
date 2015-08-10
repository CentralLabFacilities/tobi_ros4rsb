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
#include "servers/NavigationServer.h"
#include <ros/ros.h>
#include <ros/param.h>
#include <rsb/Exception.h>

int main(int argc, char **argv) {
        
        /**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line. For programmatic
	 * remappings you can use a different version of init() which takes remappings
	 * directly, but for most command-line programs, passing argc and argv is the easiest
	 * way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "ros4rsb");

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle n;
        bool isLocalNavigation;
        bool isInterleavedLaserData=false;
        
        ros::param::param<bool>("/isLocalNavigation", isLocalNavigation, true);
        //ros::param::param<bool>("/isInterleavedLaserData", isInterleavedLaserData, false);
	std::cout << "This version sets isInterleaved to false!!!" << std::endl;
        ros4rsb::LaserDataPublisher *laserPublisher;
        ros4rsb::OdometryDataPublisher *odometryPublisher;
        ros4rsb::SpeedDataPublisher *speedPublisher;
        ros4rsb::StallDataPublisher *stallPublisher;
        ros4rsb::SlamMapPublisher *slamMapPublisher;
        ros4rsb::SlamPosPublisher *slamPosPublisher;
        ros4rsb::GlobalPlanPublisher *globalPlanPublisher;
        //ros4rsb::GlobalPlannerStatePublisher *globalPlannerStatePublisher;
        ros4rsb::WaveDetectionPublisher *waveDetectionPublisher;
        
        ros4rsb::NavigationServer *navigationServer;
        
	// Initialize publishers and servers
	try {
                // Publisher
		laserPublisher = new ros4rsb::LaserDataPublisher("/ros4rsb/laserscan", n, isInterleavedLaserData);
                odometryPublisher = new ros4rsb::OdometryDataPublisher("/ros4rsb/odometryData", n);
                speedPublisher = new ros4rsb::SpeedDataPublisher("/ros4rsb/speedData", n);
                stallPublisher = new ros4rsb::StallDataPublisher("/ros4rsb/stallData", n);
                slamMapPublisher = new ros4rsb::SlamMapPublisher("/ros4rsb/slamMap", n, isLocalNavigation);
                slamPosPublisher = new ros4rsb::SlamPosPublisher("/ros4rsb/slampose", n, isLocalNavigation);
//                globalPlannerStatePublisher = new ros4rsb::GlobalPlannerStatePublisher("/ros4rsb/globalPlannerState", n);
                waveDetectionPublisher = new ros4rsb::WaveDetectionPublisher("/ros4rsb/handPos", n);
                globalPlanPublisher = new ros4rsb::GlobalPlanPublisher("/ros4rsb/globalplan", n);
                
                // Server
                navigationServer = new ros4rsb::NavigationServer(
                        "/nav_server",
                        n,
                        slamPosPublisher,
                        isLocalNavigation);
                
                

	} catch (rsb::Exception &e) {

                ROS_FATAL_STREAM("Can not initialize rsb communication!" << std::endl);
//                ROS_FATAL_STREAM("Reason: " << e << std::endl);
		exit(1);
	}
        
        
        std::cout << std::endl << "ROS4RSB is RUNNING (:" << std::endl;
        
        
	/**
	 * ros::spin() will enter a loop, pumping callbacks.  With this version, all
	 * callbacks will be called from within this thread (the main one).  ros::spin()
	 * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
	 */
        ros::spin();

        delete laserPublisher;
        delete odometryPublisher;
        delete speedPublisher;
        delete stallPublisher;
        delete slamMapPublisher;
        delete slamPosPublisher;
        delete globalPlanPublisher;
        
        delete navigationServer;
        
        return 0;
}
