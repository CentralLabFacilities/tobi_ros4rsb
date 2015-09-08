/*
 * TransformerTF.h
 *
 *  Created on: Sep 8, 2015
 *      Author: lziegler
 */

#pragma once

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include <rst/geometry/Pose.pb.h>

namespace ros4rsb {

class TransformerTF {
public:
    TransformerTF();
    virtual ~TransformerTF();

    bool transform(const rst::geometry::Pose& poseIn, geometry_msgs::PoseStamped& poseOut, const std::string &target_frame);
    bool transform(const geometry_msgs::PoseStamped& poseIn, geometry_msgs::PoseStamped& poseOut, const std::string &target_frame);

private:
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

};

} /* namespace ros4rsb */
