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

using namespace std;
using namespace boost;
using namespace rst;

namespace ros4rsb {

PersonDataPublisher::PersonDataPublisher(const string &topicIn,string name, ros::NodeHandle node) :
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

	boost::mutex::scoped_try_lock lock(myMutex/*, boost::try_to_lock*/);
	if (!lock.owns_lock()) {
		return;
	}
	
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
	
	
	//fill rsb msg
	for (it = plist.begin(); it != plist.end(); ++it) {
	  people_msgs::Person pTmp = *it;

	  rst::hri::PersonHypothesis* p = list->add_persons();
	  
	  //TODO: find better way! critical
	  //set uuid
	  p->mutable_tracking_info()->set_id(std::atoi(pTmp.name.c_str()));
	    
	  // internally we use millimeters. RST used METERS !!!!!!
	  //ProbPos xPos, yPos;
	  //pTmp->getPersGlobalX(xPos);
	  //pTmp->getPersGlobalY(yPos);
  
	  // internally we use millimeters. RST used METERS !!!!!!
	  p->mutable_body()->mutable_location()->set_x(pTmp.position.x);
	  p->mutable_body()->mutable_location()->set_y(pTmp.position.y);
	  p->mutable_body()->mutable_location()->set_z(0);
        
	  //ProbPos oPos;
	  //pTmp->getPersGlobalOrientation(oPos);
	  //Eigen::Quaternionf quat(Eigen::AngleAxisf(oPos.fPos, Eigen::Vector3f::UnitZ()));
        
	  p->mutable_body()->mutable_orientation()->set_qx(0);
	  p->mutable_body()->mutable_orientation()->set_qy(0);
	  p->mutable_body()->mutable_orientation()->set_qz(0);
	  p->mutable_body()->mutable_orientation()->set_qw(1);

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
