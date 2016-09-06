#pragma onec

#include "ListenerPub.h"

#include <rst/geometry/BoundingBox3DFloat.pb.h>
#include <rsb/Factory.h>
#include <rsb/converter/ProtocolBufferConverter.h>
#include <rsb/converter/Repository.h>
#include <rsb/MetaData.h>

#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <ros/ros.h>

#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/program_options.hpp>

namespace ros4rsb {

typedef rst::geometry::BoundingBox3DFloat Box;
typedef visualization_msgs::Marker Marker;
typedef boost::shared_ptr<Box> BoxPtr;

class BoxListener : public ListenerPub<Box, Marker> {
public:
	BoxListener(const std::string &scope, const std::string &topic, ros::NodeHandle node);
	virtual ~BoxListener();
	void callback(BoxPtr data);

};
}
