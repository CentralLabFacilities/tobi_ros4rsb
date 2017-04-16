/* 
 * File:   PersonDataPublisher.cpp
 * Author: biron
 * 
 * Created on March 13, 2012, 2:54 PM
 */

#include "PersonDataPublisher.h"
#include <rsb/MetaData.h>
#include <rst/hri/PersonHypothesis.pb.h>
#include <stdio.h>
#include <stdlib.h>

#include <boost/uuid/nil_generator.hpp>
#include <boost/uuid/string_generator.hpp>
#include <boost/uuid/name_generator.hpp>
#include <boost/uuid/random_generator.hpp>


using namespace boost;
using namespace rst;

namespace ros4rsb {

PersonDataPublisher::PersonDataPublisher(const std::string &topicIn,std::string name, ros::NodeHandle node) :
		PublisherImpl(name, node), dataAvailable(false) {

	function<void(const people_msgs::People::ConstPtr&)> m0 = bind(
			mem_fn(&PersonDataPublisher::callback), this, _1);

	rosSubscriber = node.subscribe(topicIn, 1000, m0);
	ROS_INFO_STREAM("PersonDataPublisher: " << name << " is subscribing to topic " << topicIn);
}

PersonDataPublisher::~PersonDataPublisher() {
}

void PersonDataPublisher::callback(
		const people_msgs::People::ConstPtr &message) {
  
	ROS_DEBUG_STREAM("PersonDataPublisher message callback");
	
	boost::mutex::scoped_lock lock(myMutex/*, boost::try_to_lock*/);
	if (!lock.owns_lock()) {
		return;
	}
	ROS_DEBUG_STREAM("PersonDataPublisher message callback");
	
	// create correct timestamp
	uint32_t sec = message->header.stamp.sec;
	uint32_t nsec = message->header.stamp.nsec;
	uint32_t usec = nsec / 1000;
	uint64_t timestamp = sec * 1000000 + usec;
	
	//output list and iterator
	shared_ptr<rst::hri::PersonHypotheses> list(new rst::hri::PersonHypotheses());
	
	
	//get iterative vector
	std::vector<people_msgs::Person> plist = message->people;
	if (plist.size() == 0) {
	  return;
	}
	
	std::vector<people_msgs::Person>::iterator it;
	
	boost::uuids::string_generator gen;
	
	//fill rsb msg
	for (it = plist.begin(); it != plist.end(); ++it) {
	  people_msgs::Person pTmp = *it;

	  rst::hri::PersonHypothesis* p = list->add_persons();
	  
	  //TODO: find better way! critical
	  //set uuid
	  //memcpy(&guid, data, 16);
          
          
          //take characters 0 to 8 from the uuid, parse as hex(base 16) and assign to unsigned int.
          //123e4567-e89b-12d3-a456-426655440000 -> 123e4567 -> 306070887
          unsigned int x = strtoul(pTmp.name.substr(0, 8).c_str(), NULL, 16);
          
	  p->mutable_tracking_info()->set_id(x);
	  //boost::uuids::uuid uuid = gen(pTmp.name);
	  //int* id;
	  //char* pGuid = (char*) &uuid;
	  
	  //memcpy(pGuid, id, 4);
	  //p->mutable_tracking_info()->set_id(*id);  
	  
	  // internally we use millimeters. RST used METERS !!!!!!
	  //ProbPos xPos, yPos;
	  //pTmp->getPersGlobalX(xPos);
	  //pTmp->getPersGlobalY(yPos);
  
	  // internally we use millimeters. RST used METERS !!!!!!
	  p->mutable_body()->mutable_location()->set_x(pTmp.position.x);
	  p->mutable_body()->mutable_location()->set_y(pTmp.position.y);
	  p->mutable_body()->mutable_location()->set_z(0);
	  p->mutable_body()->mutable_location()->set_frame_id("/map");
	  
	  //ProbPos oPos;
	  //pTmp->getPersGlobalOrientation(oPos);
	  //Eigen::Quaternionf quat(Eigen::AngleAxisf(oPos.fPos, Eigen::Vector3f::UnitZ()));
        
	  p->mutable_body()->mutable_orientation()->set_qx(0);
	  p->mutable_body()->mutable_orientation()->set_qy(0);
	  p->mutable_body()->mutable_orientation()->set_qz(0);
	  p->mutable_body()->mutable_orientation()->set_qw(1);
	  p->mutable_body()->mutable_orientation()->set_frame_id("/map");

	  p->mutable_face()->mutable_face()->mutable_region()->set_height(0);
	  p->mutable_face()->mutable_face()->mutable_region()->set_width(0);
	  p->mutable_face()->mutable_face()->mutable_region()->set_image_height(0);
	  p->mutable_face()->mutable_face()->mutable_region()->set_image_width(0);
	  p->mutable_face()->mutable_face()->mutable_region()->mutable_top_left()->set_x(0);
	  p->mutable_face()->mutable_face()->mutable_region()->mutable_top_left()->set_y(0);

	  //LOG4CXX_DEBUG(logger,
          //      "publish person - xyz:" << p->body().location().x()<<","<<p->body().location().y() <<"," << p->body().location().z()<<" - orientation xyzw:" << p->body().orientation().qx()<<"," << p->body().orientation().qy()<<"," << p->body().orientation().qz()<<"," << p->body().orientation().qw());
    }
	rsb::EventPtr event = rsbInformer->createEvent();
	event->setData(list);
	event->mutableMetaData().setCreateTime(timestamp);
	this->publish(event);
}
}
