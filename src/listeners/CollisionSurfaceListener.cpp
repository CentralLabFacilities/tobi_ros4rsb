#include <Eigen/Geometry>
#include <sstream>
#include <limits>

#include <boost/algorithm/string.hpp>

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
        frameOriginArm(DEFAULT_FRAME_ORIGIN_ARM) {
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
    ROS_DEBUG_STREAM("CollisionSurfaceListener forwarding " << numPatches << " surfaces");
    for (size_t i = 0; i < numPatches; i++) {
        const ::rst::geometry::PolygonalPatch3D& patch = input->patches(i);
        stringstream ss;
        ss << "surface" << i;
        string name = ss.str();

        geometry_msgs::PoseStamped pose;
        if (!transformer.transform(patch.base(), pose, frameOriginArm))
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

        geometry_msgs::Pose poseNew;
        poseNew.position.x = xMax - (xMax - xMin) / 2.0;
        poseNew.position.y = yMax - (yMax - yMin) / 2.0;
        poseNew.position.z = zMax - (zMax - zMin) / 2.0;
        poseNew.orientation.w = 1.0;

        moveit_msgs::CollisionObject surface;
        surface.header.frame_id = frameOriginArm;
        surface.id = ss.str();
        surface.operation = surface.ADD;
        surface.primitive_poses.push_back(poseNew);
        surface.primitives.push_back(primitive);

        surfaces.push_back(surface);
    }
    sceneInterface.addCollisionObjects(surfaces);
}

}
