#pragma onec

#include "ListenerScene.h"

#include "../util/TransformerTF.h"

#include <rst/tracking/TrackedClassifiedRegions3D.pb.h>
#include <rsb/Factory.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/converter/Repository.h>
#include <rsb/MetaData.h>

#include <ros/ros.h>

#include <moveit_msgs/PlanningScene.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>

namespace ros4rsb {

typedef rst::tracking::TrackedClassifiedRegions3D Boxes;
typedef boost::shared_ptr<Boxes> BoxesPtr;

class CollisionBoxListener : public ListenerScene<Boxes> {
public:
    CollisionBoxListener(const std::string &scopeIn, const std::string &topicOut, ros::NodeHandle &node);
	virtual ~CollisionBoxListener();
	void callback(BoxesPtr data);

	CREATE_LISTENER_BUILDER_NESTED(CollisionBoxListener)

private:
	std::vector<std::string> lastObjects;
	TransformerTF transformer;
	std::string frameOriginArm;
protected:
	 static const std::string DEFAULT_OBJECT_PREFIX;
	 static const std::string DEFAULT_FRAME_ORIGIN_ARM;
};
}
