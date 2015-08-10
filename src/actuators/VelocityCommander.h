/* 
 * File:   VelocityCommander.h
 * Author: prenner, ruegeme, cklarhor
 *
 * Created on June 7, 2012, 6:17 PM
 */

#ifndef VELOCITYCOMMANDER_H
#define	 VELOCITYCOMMANDER_H

#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

enum ExitStatus { STILL_RUNNING=-1, SUCCESS=0, SUPERSEDED = 1, CANCELLED = 2, NODE_BROKEN = 3,TRANSFORM_ERROR=4};

class VelocityCommander {
public:
    VelocityCommander(std::string name, ros::NodeHandle node);
    virtual ~VelocityCommander();
    
    ExitStatus drive(double distance, double speed);
    ExitStatus turn(double angle, double speed);
    bool isRunning();
    void stop();
    
private:
    void killRunningAndSet();
    ExitStatus checkRunningAndExit();
    std::string name;
    ros::NodeHandle node;
    tf::TransformListener listener;
    ros::Publisher publisher;
    boost::mutex mutex;
    bool running;
    bool kill;
    bool should_stop;

};

#endif	/* VELOCITYCOMMANDER_H */

