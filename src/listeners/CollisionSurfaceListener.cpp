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

const std::string CollisionSurfaceListener::DEFAULT_FRAME_ORIGIN_ARM = "base_link";

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

    moveit_msgs::CollisionObject surfaceBig;
    moveit_msgs::CollisionObject surfaceHigh;
    geometry_msgs::PoseStamped poseBig;
    geometry_msgs::PoseStamped poseHigh;
    shape_msgs::SolidPrimitive primitiveBig;
    shape_msgs::SolidPrimitive primitiveHigh;
    double yBig, xBig, zBig, yMaxB, yMinB, zMax;
    yBig = xBig = zBig = yMaxB = yMinB = zMax= 0;
    int numPatches = input->patches_size();
    ROS_DEBUG_STREAM("CollisionSurfaceListener forwarding " << numPatches << " surfaces");

    if(numPatches < 1) {
        return;
    }

    for (size_t i = 0; i < numPatches; i++) {
        const ::rst::geometry::PolygonalPatch3D& patch = input->patches(i);
        stringstream ss;
        ss << "surface" << i;
        string name = ss.str();

        double xMax, yMax;
        xMax = yMax = -numeric_limits<double>::max();
        double xMin, yMin;
        xMin = yMin = numeric_limits<double>::max();

        //gets the z Coordinate to detect the highest and lowest plane
        double zCoord = patch.base().translation().z();

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

        //Transform of the Plane itself
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = xMax - xMin;
        primitive.dimensions[1] = yMax - yMin;
        primitive.dimensions[2] = 0.01;
	
        if(numPatches > 1){
          primitive.dimensions[1] = 1.3;
        }

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

        //the important data for the biggest surface is stored.
        if((abs(yMax - yMin)) > yBig){
          yBig = abs(yMax - yMin);
          xBig = abs(xMax - xMin);
          zBig = zCoord;
          yMaxB = yMax;
          yMinB = yMin;
          surfaceBig = surface;
          primitiveBig = primitive;
          poseBig = poseNew;
        }
        
        
        //store highest plane
        if(zCoord > zMax){
            cout << "found plane at z = " << zCoord << endl;
            surfaceHigh = surface;
            poseHigh = poseNew;
            primitiveHigh = primitive;
            zMax = zCoord;
        }
    }

    if(numPatches > 1){
      zBig = 3.00;
    }

    //primitive is reused and parameters for left plane are used
    primitiveBig.dimensions[0] =  xBig; //length
    primitiveBig.dimensions[1] =  0.01; //depth
    primitiveBig.dimensions[2] =  zBig;//height

    moveit_msgs::CollisionObject surfaceLeft;
    surfaceLeft.header.frame_id = poseBig.header.frame_id;
    surfaceLeft.id = "surfaceBigLeft";
    surfaceLeft.operation = surfaceBig.ADD;
    surfaceLeft.primitive_poses.push_back(poseBig.pose);
    surfaceLeft.primitives.push_back(primitiveBig);
    surfaceLeft.primitive_poses[0].position.x = surfaceBig.primitive_poses[0].position.x;
    surfaceLeft.primitive_poses[0].position.y = yMinB;
    surfaceLeft.primitive_poses[0].position.z = surfaceBig.primitive_poses[0].position.z / 2;

    surfaces.push_back(surfaceLeft);

    //Transform of the right plane that is created; for comments look at leftplane
    primitiveBig.dimensions[0] =  xBig; //length
    primitiveBig.dimensions[1] =  0.01; //depth
    primitiveBig.dimensions[2] =  zBig;//height

    moveit_msgs::CollisionObject surfaceRight;
    surfaceRight.header.frame_id = poseBig.header.frame_id;
    surfaceRight.id = "surfaceBigRight";
    surfaceRight.operation = surfaceBig.ADD;
    surfaceRight.primitive_poses.push_back(poseBig.pose);
    surfaceRight.primitives.push_back(primitiveBig);
    surfaceRight.primitive_poses[0].position.x = surfaceBig.primitive_poses[0].position.x;
    surfaceRight.primitive_poses[0].position.y = yMaxB;
    surfaceRight.primitive_poses[0].position.z = surfaceBig.primitive_poses[0].position.z / 2;

    surfaces.push_back(surfaceRight);
    
    primitiveHigh.dimensions[0] =  surfaceHigh.primitives[0].dimensions[0];   //length
    primitiveHigh.dimensions[1] =  surfaceHigh.primitives[0].dimensions[1];  //depth
    primitiveHigh.dimensions[2] =  surfaceHigh.primitives[0].dimensions[2]; //height
    
    cout << "adding highest surface" << endl;
    //raise surface by 34cm
    moveit_msgs::CollisionObject surfaceUpper;
    surfaceUpper.header.frame_id = poseHigh.header.frame_id;
    surfaceUpper.id = "surfaceHigh";
    surfaceUpper.operation = surfaceHigh.ADD;
    surfaceUpper.primitive_poses.push_back(poseHigh.pose);
    surfaceUpper.primitives.push_back(primitiveHigh);
    surfaceUpper.primitive_poses[0].position.x = surfaceHigh.primitive_poses[0].position.x;
    surfaceUpper.primitive_poses[0].position.y = surfaceHigh.primitive_poses[0].position.y;
    surfaceUpper.primitive_poses[0].position.z = surfaceHigh.primitive_poses[0].position.z + 0.34;

    surfaces.push_back(surfaceUpper);

    sceneInterface.addCollisionObjects(surfaces);
}

}
