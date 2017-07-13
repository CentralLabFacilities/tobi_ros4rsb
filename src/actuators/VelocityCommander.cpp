/* 
 * File:   VelocityCommander.cpp
 * Author: biron, cklarhor, lruegeme
 * 
 * Created on June 7, 2012, 6:17 PM
 */
#include <unistd.h>
#include "VelocityCommander.h"

VelocityCommander::VelocityCommander(std::string name, ros::NodeHandle node) {
    this->name = name;
    this->node = node;
    this->publisher = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    this->running = false;
    ROS_INFO("VelocityCommander: publishing to /cmd_vel.");
}

void VelocityCommander::stop() {
	boost::mutex::scoped_lock lock(mutex);
    if (running)
        should_stop = true;
	while(running) {
		usleep(100);
	}

}

bool VelocityCommander::isRunning() {
    return running;
}

void VelocityCommander::killRunningAndSet() {
    boost::mutex::scoped_lock lock(mutex);
    printf("killRunningAndSet()\n");
    if (running) {
        kill = true;
    }
    while(running) {
		usleep(100);
	}
    running = true;
    printf("now we can start\n");
}

ExitStatus VelocityCommander::checkRunningAndExit() {
    if (!running) { //should not happen! throw exception?
        cout << "Returning SUCCES while not running" << endl;
        return SUCCESS;
    }
    if (kill) {
        kill = false;
        running = false;
        cout << "Returning SUPERSEDED" << endl;
        return SUPERSEDED;
    }
    if (should_stop) {
        should_stop = false;
        running = false;
        cout << "Returning CANCELLED" << endl;
        return CANCELLED;
    }
    if (!node.ok()) {
        running = false;
        cout << "Returning NODE_BROKEN" << endl;
        return NODE_BROKEN;
    }
    cout << "Returning NODE_BROKEN" << endl;
    return STILL_RUNNING;
}

ExitStatus VelocityCommander::drive(double distance, double speed) {
    killRunningAndSet();
    printf("drive %e with %e speed\n",distance, speed);
    if (distance == 0) {
        running = false;
        return SUCCESS;
    }
    
    //wait for the listener to get the first message
    listener.waitForTransform("base_link", "odom", ros::Time::now(), ros::Duration(1.0));
    
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform current_transform;

    //record the starting transform from the odometry to the base frame
    listener.lookupTransform("base_link", "odom", ros::Time(0), start_transform);
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to go forward
    base_cmd.linear.y = base_cmd.angular.z = 0;
    if (distance > 0) {
        base_cmd.linear.x = speed;
    } else {
        base_cmd.linear.x = -speed;
    }
    
    ros::Time endTime = ros::Time::now() + ros::Duration(fabs(distance) /  fabs(base_cmd.linear.x) * 2.0 + 1.0);
    
    ros::Rate rate(15.0);
        
    ExitStatus result = checkRunningAndExit();
    while (result==STILL_RUNNING)
    {
      //send the drive command
      publisher.publish(base_cmd);
      rate.sleep();
      //get the current transform
      try
      {
        listener.lookupTransform("base_link", "odom", ros::Time(0), current_transform);
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("%s",ex.what());
        running = false;
        return TRANSFORM_ERROR;
      }
      //see how far we've traveled
      tf::Transform relative_transform = start_transform.inverse() * current_transform;
      double dist_moved = relative_transform.getOrigin().length();

      if (dist_moved >= std::fabs(distance) || ros::Time::now() >= endTime) {
          std::cout << "dist_moved: " << dist_moved << "distance" << distance <<std::endl;
          base_cmd.linear.x = 0;
          publisher.publish(base_cmd);
          running = false;
          return SUCCESS;
      }
      result = checkRunningAndExit();
    }
    base_cmd.linear.x = 0; //Stop on error
    publisher.publish(base_cmd);
    return result;
}

ExitStatus VelocityCommander::turn(double angle, double speed) {
    killRunningAndSet();
    printf("turn %e with %e speed\n",angle, speed);
    
    float sleeptime = 10.0; //times add up!
    ros::Rate rate(1000.0/sleeptime);
    
    if (angle == 0) {
        running = false;
        return SUCCESS;
    }
    
    bool clockwise = false;
    if (angle < 0) {
        angle = fabs(angle);
        clockwise = true;
    }
    uint skips = 0;
    
    //wait for the listener to get the first message
    bool waitTransform = listener.waitForTransform("base_link", "odom", ros::Time::now(), ros::Duration(1.0));
    std::cout << "waitTransform was: " << waitTransform << std::endl;
        
    //we will record transforms here
    tf::StampedTransform start_transform;
    tf::StampedTransform last_transform;
    tf::StampedTransform current_transform;
    
    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    //the command will be to turn
    base_cmd.linear.x = base_cmd.linear.y = 0.0;
 
 
    base_cmd.angular.z = speed;
    if (clockwise) base_cmd.angular.z = -speed;
    printf("setting speed to %e and sleep for %e ms\n", speed, sleeptime);
    
    //the axis we want to be rotating by
    tf::Vector3 desired_turn_axis(0,0,1);
    if (!clockwise) desired_turn_axis = -desired_turn_axis;
    
    std::cout << "get endtime" << std::endl;
    ros::Time endTime = ros::Time::now() + ros::Duration(fabs(angle) /  fabs(base_cmd.angular.z) * 2.0 + 1.0);
    std::cout << "got endtime" << std::endl;

    //record the starting transform from the odometry to the base frame    
    listener.lookupTransform("base_link", "odom", ros::Time(0), start_transform);
    last_transform = start_transform;    
    
    ExitStatus result = checkRunningAndExit();

    double angle_turned_total = 0.0;
    std::cout << "start polling" << std::endl;
    while (result==STILL_RUNNING)
    {
      //send the drive command
      base_cmd.angular.z = speed;
      if (clockwise) base_cmd.angular.z = -speed;
        printf("setting speed to %e and sleep for %e ms\n", speed, sleeptime);
      publisher.publish(base_cmd);
      rate.sleep();     
      
      //get the current transform
      try
      {
        listener.lookupTransform("base_link", "odom", ros::Time(0), current_transform);
        if (last_transform.stamp_!=current_transform.stamp_)  {
            double angle_turned_step = last_transform.getRotation().angleShortestPath(current_transform.getRotation());
            last_transform = current_transform;          
            angle_turned_total+=angle_turned_step;        
            std::cout<<"turn step was "<<angle_turned_step<< " sum:" << angle_turned_total << std::endl;
            skips = 0;
            //limit speed if close to target
            double angle_to_turn = angle - angle_turned_total;      
            if (angle_to_turn<0.6 && speed > 0.4) speed = 0.4;
            if (angle_to_turn<0.3 && speed > 0.1) speed = 0.1;
        } else {
            skips++;
            std::cout<<"skipped" << std::endl;
        }
      }
      catch (tf::TransformException &ex)
      {
        ROS_ERROR("%s",ex.what());
        running = false;
        return TRANSFORM_ERROR;
      }
      
      //calculate expected angle with turnspeed (odom updaterate is 10hz)
      float calculated_turn=( speed * sleeptime * 0.001 * (float) skips);      
      
      std::cout<<"I'm there returning angle_turned "<<angle_turned_total<< " calculated angle: " << calculated_turn << ", angle: "<<angle<<std::endl;
      
      if (angle_turned_total + calculated_turn >= angle || ros::Time::now() >= endTime) {
          base_cmd.angular.z = 0;
          publisher.publish(base_cmd);
          std::cout<<"I'm there returning angle_turned "<<angle_turned_total<< " calculated angle: " << calculated_turn << ", angle: "<<angle<<std::endl;
          std::cout<<"End because (time end 0/1: "<<(ros::Time::now() >= endTime)<<" angle: "<<angle<<std::endl;
          running = false;
          return SUCCESS;
      }
      result = checkRunningAndExit();
    }
    base_cmd.angular.z = 0; //Stop on error
    publisher.publish(base_cmd);
    return result;    
}

VelocityCommander::~VelocityCommander() {
    
}
