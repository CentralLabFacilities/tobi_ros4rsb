#include <Eigen/Geometry>
#include <sstream>
#include <limits>

#include <boost/algorithm/string.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_messages.h>
#include <geometric_shapes/shape_operations.h>
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

        double xMax, yMax;
        xMax = yMax = -numeric_limits<double>::max();
        double xMin, yMin;
        xMin = yMin = numeric_limits<double>::max();

        EigenSTL::vector_Vector3d vertices;
        int numBorder = patch.border_size();
        for (size_t j = 0; j < numBorder; j++) {
            const ::rst::math::Vec2DFloat& border = patch.border(j);
            Eigen::Vector3d p(border.x(), border.y(), 0.0);
            vertices.push_back(p);
            if (xMax < border.x())
                xMax = border.x();
            if (yMax < border.y())
                yMax = border.y();
            if (xMin > border.x())
                xMin = border.x();
            if (yMin > border.y())
                yMin = border.y();
        }

        double xCenter = xMin + (xMax - xMin) / 2.0;
        double yCenter = yMin + (yMax - yMin) / 2.0;

        //convert
//        shapes::Mesh* mesh = shapes::createMeshFromVertices(vertices);
//        shape_msgs::Mesh mesh_msg;
//        shapes::ShapeMsg shape_mesh_msg = mesh_msg;
//        shapes::constructMsgFromShape(mesh,shape_mesh_msg);

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = xMax - xMin;
        primitive.dimensions[1] = yMax - yMin;
        primitive.dimensions[2] = 0.01;


        geometry_msgs::PoseStamped poseOld;
        poseOld.pose.position.x = patch.base().translation().x();
        poseOld.pose.position.y = patch.base().translation().y();
        poseOld.pose.position.z = patch.base().translation().z();
        poseOld.pose.orientation.w = patch.base().rotation().qw();
        poseOld.pose.orientation.x = patch.base().rotation().qx();
        poseOld.pose.orientation.y = patch.base().rotation().qy();
        poseOld.pose.orientation.z = patch.base().rotation().qz();
        poseOld.header.frame_id = patch.base().translation().frame_id();

        // Attention: Ros assumes the applied pose to be the center of the
        // object. The RST type does not require the pose to be the center. Therefore we shift
        // shift in the x-y plane.
        geometry_msgs::PoseStamped poseNew;
        transformer.transform(poseOld, poseNew, "base_link");
//        poseNew.pose.position.x -= xCenter;
//        poseNew.pose.position.y -= yCenter;

        moveit_msgs::CollisionObject surface;
        surface.header.frame_id = poseNew.header.frame_id;
        surface.id = ss.str();
        surface.operation = surface.ADD;
        surface.primitive_poses.push_back(poseNew.pose);
        surface.primitives.push_back(primitive);
//        surface.mesh_poses.push_back(poseNew);
//        surface.meshes.push_back(mesh_msg);

        surfaces.push_back(surface);
    }
    sceneInterface.addCollisionObjects(surfaces);
}

}
