#pragma once

#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <ros/ros.h>
#include <rsb/Listener.h>
#include <rsb/Factory.h>
#include <string>

#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

/**
 * This is a abstract class defining the outline of every Publisher that wraps ROS topics to RST types.
 */
namespace ros4rsb {

template<class RsbType, class RosType>
class Listener {
public:

	Listener(const std::string &scopeIn, const std::string &topicOut, ros::NodeHandle node) :
		factory(rsb::getFactory()) {
		this->scope = scopeIn;
		this->node = node;
		this->rosPublisher = node.advertise<RosType>(topicOut, 10);

#ifndef SKIPCONVERTER
		try {
			boost::shared_ptr<rsb::converter::ProtocolBufferConverter<RsbType> >
							converter(new rsb::converter::ProtocolBufferConverter<RsbType>());

			rsb::converter::converterRepository<std::string>()->registerConverter(
							converter);
		} catch (std::invalid_argument &e) {
			std::cout << "Warning: trying to register two converters for same type" << std::endl;
		}
#endif

		rsbListener = factory.createListener(scope);
		boost::function<void(boost::shared_ptr<RsbType>)> cb(boost::bind(&Listener::callback, this, _1));
		rsbListener->addHandler(rsb::HandlerPtr(new rsb::DataFunctionHandler<RsbType>(cb)));
	}

	virtual ~Listener() {
		rosPublisher.shutdown();
	}

	virtual void callback(boost::shared_ptr<RsbType> data) = 0;
	virtual void publish(const RosType &data) {
		rosPublisher.publish(data);
	}

private:
	std::string scope;
	ros::NodeHandle node;

protected:
	ros::Publisher rosPublisher;
	rsb::Factory& factory;
	typename rsb::ListenerPtr rsbListener;
};

}
