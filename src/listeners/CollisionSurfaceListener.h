#pragma onec

#include "ListenerScene.h"

#include "../util/TransformerTF.h"

#include <rst/geometry/PolygonalPatch3DSet.pb.h>
#include <rsb/Factory.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/converter/Repository.h>
#include <rsb/MetaData.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <moveit_msgs/PlanningScene.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>

namespace ros4rsb {

typedef rst::geometry::PolygonalPatch3DSet Patches;
typedef boost::shared_ptr<Patches> PatchesPtr;

class CollisionSurfaceListener : public ListenerScene<Patches> {
public:
    CollisionSurfaceListener(const std::string &scopeIn, const std::string &topicOut, ros::NodeHandle &node);
	virtual ~CollisionSurfaceListener();
	void callback(PatchesPtr data);

private:
	static const std::string DEFAULT_FRAME_ORIGIN_ARM;

	std::string frameOriginArm;

	TransformerTF transformer;
};
}
