#include <Eigen/Geometry>
#include <sstream>
#include <limits>

#include <boost/algorithm/string.hpp>

#include <kdl/frames.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "CollisionSurfaceListener.h"

using namespace std;
using namespace ros;
using namespace boost;
using namespace rst;

namespace ros4rsb {

const std::string CollisionSurfaceListener::DEFAULT_FRAME_ORIGIN_ARM = "katana_base_link";

CollisionSurfaceListener::CollisionSurfaceListener(const std::string &scopeIn, const std::string &topicOut, ros::NodeHandle &node) :
        ListenerScene(scopeIn),
        frameOriginArm(DEFAULT_FRAME_ORIGIN_ARM),
        tfListener(tfBuffer) {
}

CollisionSurfaceListener::~CollisionSurfaceListener() {
}

void CollisionSurfaceListener::callback(PatchesPtr input) {

    // cleanup scene
    vector<string> knownObjects = sceneInterface.getKnownObjectNames();
    vector<string> knownSurfaces;
    for (vector<string>::iterator i = knownObjects.begin(); i != knownObjects.end(); ++i) {
        if (boost::algorithm::contains(*i, "surface")) {
            knownSurfaces.push_back(*i);
        }
    }
    sceneInterface.removeCollisionObjects(knownSurfaces);

    // create new collision objects
    vector<moveit_msgs::CollisionObject> surfaces;

    int numPatches = input->patches_size();
    for (size_t i = 0; i < numPatches; i++) {
        const ::rst::geometry::PolygonalPatch3D& patch = input->patches(i);
        stringstream ss;
        ss << "surface" << i;
        string name = ss.str();

        geometry_msgs::PoseStamped pose;
        if (!transform(patch.base(), pose, frameOriginArm))
            continue;

        Eigen::Affine3d transform;
        transform.fromPositionOrientationScale(
                Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z),
                Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x,
                        pose.pose.orientation.y, pose.pose.orientation.z), Eigen::Vector3d::Ones());

        double xMax, yMax, zMax;
        xMax = yMax = zMax = -numeric_limits<double>::max();
        double xMin, yMin, zMin;
        xMin = yMin = zMin = numeric_limits<double>::max();

        int numBorder = patch.border_size();
        for (size_t j = 0; j < numBorder; j++) {
            const ::rst::math::Vec2DFloat& border = patch.border(j);
            Eigen::Vector3d vec(border.x(), border.y(), 0);
            vec = transform * vec;

            if (xMax < vec(0))
                xMax = vec(0);
            if (yMax < vec(1))
                yMax = vec(1);
            if (zMax < vec(2))
                zMax = vec(2);
            if (xMin > vec(0))
                xMin = vec(0);
            if (yMin > vec(1))
                yMin = vec(1);
            if (zMin > vec(2))
                zMin = vec(2);
        }

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = xMax - xMin;
        primitive.dimensions[1] = yMax - yMin;;
        primitive.dimensions[2] = zMax - zMin;
        pose.pose.position.x = xMax - (xMax - xMin) / 2.0;
        pose.pose.position.y = yMax - (yMax - yMin) / 2.0;
        pose.pose.position.z = zMax - (zMax - zMin) / 2.0;

        moveit_msgs::CollisionObject surface;
        surface.header.frame_id = frameOriginArm;
        surface.id = ss.str();
        surface.operation = surface.ADD;
        surface.primitive_poses.push_back(pose.pose);
        surface.primitives.push_back(primitive);

        surfaces.push_back(surface);
    }
    sceneInterface.addCollisionObjects(surfaces);
}

bool CollisionSurfaceListener::transform(const rst::geometry::Pose& poseIn,
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
bool CollisionSurfaceListener::transform(const geometry_msgs::PoseStamped& poseIn,
        geometry_msgs::PoseStamped& poseOut, const std::string &target_frame) {
    try {
        geometry_msgs::PoseStamped myPose = poseIn;
        string from = myPose.header.frame_id;
        boost::algorithm::replace_all(from, "/", "");
        myPose.header.frame_id = from;
        ROS_INFO_STREAM("transform " << from << " to " << target_frame);
        tfBuffer.transform(myPose, poseOut, target_frame);
        poseOut.header.frame_id = target_frame;
        return true;
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return false;
    }
}
}

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
