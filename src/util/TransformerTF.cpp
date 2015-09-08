/*
 * TransformerTF.cpp
 *
 *  Created on: Sep 8, 2015
 *      Author: lziegler
 */

#include "TransformerTF.h"

#include <kdl/frames.hpp>

using namespace std;

namespace ros4rsb {

TransformerTF::TransformerTF() :
        tfListener(tfBuffer) {
}

TransformerTF::~TransformerTF() {
}

bool TransformerTF::transform(const rst::geometry::Pose& poseIn,
        geometry_msgs::PoseStamped& poseOut, const std::string &target_frame) {
    geometry_msgs::PoseStamped ps;

    ps.pose.position.x = poseIn.translation().x();
    ps.pose.position.y = poseIn.translation().y();
    ps.pose.position.z = poseIn.translation().z();
    ps.pose.orientation.w = poseIn.rotation().qw();
    ps.pose.orientation.x = poseIn.rotation().qx();
    ps.pose.orientation.y = poseIn.rotation().qy();
    ps.pose.orientation.z = poseIn.rotation().qz();
    ps.header.frame_id = poseIn.translation().frame_id();

    if (transform(ps, ps, target_frame)) {
        poseOut = ps;
        return true;
    } else {
        return false;
    }
}
bool TransformerTF::transform(const geometry_msgs::PoseStamped& poseIn,
        geometry_msgs::PoseStamped& poseOut, const std::string &target_frame) {
    try {
        geometry_msgs::PoseStamped myPose = poseIn;
        string from = myPose.header.frame_id;
        boost::algorithm::replace_all(from, "/", "");
        myPose.header.frame_id = from;
        ROS_DEBUG_STREAM("transform " << from << " to " << target_frame);
        tfBuffer.transform(myPose, poseOut, target_frame);
        poseOut.header.frame_id = target_frame;
        return true;
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
}

} /* namespace ros4rsb */

namespace tf2 {

KDL::Frame gmTransformToKDL(const geometry_msgs::TransformStamped& t) {
    return KDL::Frame(
            KDL::Rotation::Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                    t.transform.rotation.z, t.transform.rotation.w),
            KDL::Vector(t.transform.translation.x, t.transform.translation.y,
                    t.transform.translation.z));
}

// method to extract timestamp from object
template<>
const ros::Time& getTimestamp(const geometry_msgs::PoseStamped& t) {
    return t.header.stamp;
}

// method to extract frame id from object
template<>
const std::string& getFrameId(const geometry_msgs::PoseStamped& t) {
    return t.header.frame_id;
}

// this method needs to be implemented by client library developers
template<>
void doTransform(const geometry_msgs::PoseStamped& t_in, geometry_msgs::PoseStamped& t_out,
        const geometry_msgs::TransformStamped& transform) {
    KDL::Vector v(t_in.pose.position.x, t_in.pose.position.y, t_in.pose.position.z);
    KDL::Rotation r = KDL::Rotation::Quaternion(t_in.pose.orientation.x, t_in.pose.orientation.y,
            t_in.pose.orientation.z, t_in.pose.orientation.w);

    tf2::Stamped<KDL::Frame> v_out = tf2::Stamped<KDL::Frame>(
            gmTransformToKDL(transform) * KDL::Frame(r, v), transform.header.stamp,
            transform.header.frame_id);
    t_out.pose.position.x = v_out.p[0];
    t_out.pose.position.y = v_out.p[1];
    t_out.pose.position.z = v_out.p[2];
    v_out.M.GetQuaternion(t_out.pose.orientation.x, t_out.pose.orientation.y,
            t_out.pose.orientation.z, t_out.pose.orientation.w);
    t_out.header.stamp = v_out.stamp_;
    t_out.header.frame_id = v_out.frame_id_;
}
geometry_msgs::PoseStamped toMsg(const geometry_msgs::PoseStamped& in) {
    return in;
}
void fromMsg(const geometry_msgs::PoseStamped& msg, geometry_msgs::PoseStamped& out) {
    out = msg;
}
}
