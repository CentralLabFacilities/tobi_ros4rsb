#include <Eigen/Geometry>
#include <sstream>
#include <limits>

#include <boost/algorithm/string.hpp>

#include <kdl/frames.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "CollisionBoxListener.h"

using namespace std;
using namespace ros;
using namespace boost;
using namespace rst;

namespace ros4rsb {

const string CollisionBoxListener::DEFAULT_OBJECT_PREFIX = "object";

CollisionBoxListener::CollisionBoxListener(const string &scopeIn, const string &topicOut, ros::NodeHandle &node) :
        ListenerScene(scopeIn) {
}

CollisionBoxListener::~CollisionBoxListener() {
}

void CollisionBoxListener::callback(BoxesPtr input) {

    // cleanup scene
    vector<string> knownObjects = sceneInterface.getKnownObjectNames();
    vector<string> knownBoxes;
    for (vector<string>::iterator i = knownObjects.begin(); i != knownObjects.end(); ++i) {
        if (boost::algorithm::contains(*i, DEFAULT_OBJECT_PREFIX)) {
            knownBoxes.push_back(*i);
        }
    }
    sceneInterface.removeCollisionObjects(knownBoxes);
    sceneInterface.removeCollisionObjects(lastObjects);

    // create new collision objects
    vector<moveit_msgs::CollisionObject> objects;

    int numPatches = input->region_size();

    ROS_DEBUG_STREAM("CollisionBoxListener forwarding " << numPatches << " boxes");
    for (size_t i = 0; i < numPatches; i++) {
        const ::rst::tracking::TrackedClassifiedRegion3D& box = input->region(i);

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = box.region().region().transformation().translation().x();
        pose.pose.position.y = box.region().region().transformation().translation().y();
        pose.pose.position.z = box.region().region().transformation().translation().z();
        pose.pose.orientation.w = box.region().region().transformation().rotation().qw();
        pose.pose.orientation.x = box.region().region().transformation().rotation().qx();
        pose.pose.orientation.y = box.region().region().transformation().rotation().qy();
        pose.pose.orientation.z = box.region().region().transformation().rotation().qz();
        pose.header.frame_id = box.region().region().transformation().translation().frame_id();

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[0] = box.region().region().width();
        primitive.dimensions[1] = box.region().region().depth();
        primitive.dimensions[2] = box.region().region().height();

        stringstream ss;
        ss << DEFAULT_OBJECT_PREFIX << box.info().id();
        string name = ss.str();

        moveit_msgs::CollisionObject object;
        object.header.frame_id = pose.header.frame_id;
        object.id = ss.str();
        object.operation = object.ADD;
        object.primitive_poses.push_back(pose.pose);
        object.primitives.push_back(primitive);

        objects.push_back(object);
    }
    sceneInterface.addCollisionObjects(objects);
}

}
