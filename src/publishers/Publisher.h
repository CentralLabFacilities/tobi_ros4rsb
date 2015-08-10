#ifndef PUBLISHER_H_
#define PUBLISHER_H_

#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <ros/ros.h>
#include <rsb/Informer.h>
#include <rsb/Factory.h>
#include <rst/geometry/Pose.pb.h>
#include <string>

#include <rsb/Informer.h>
#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

/**
 * This is a abstract class defining the outline of every Publisher that wraps ROS topics to RST types.
 */
namespace ros4rsb {

template<class T>
class Publisher {
public:

	Publisher(std::string scope, ros::NodeHandle node) :
		factory(rsb::getFactory()) {
		this->scope = scope;
		this->node = node;

#ifndef SKIPCONVERTER
                try {
                    boost::shared_ptr<rsb::converter::ProtocolBufferConverter<T> >
                                    converter(new rsb::converter::ProtocolBufferConverter<T>());

                    rsb::converter::converterRepository<std::string>()->registerConverter(
                                    converter);
                } catch (std::invalid_argument &e) {
                    std::cout << "Warning: trying to register two converters for same type" << std::endl;
                }
#endif

		rsbInformer = factory.createInformer<T> (scope);

	}

	virtual ~Publisher() {
		rosSubscriber.shutdown();
	}

	void publish(typename rsb::Informer<T>::DataPtr msg) {
		rsbInformer->publish(msg);
	}
	void publish(rsb::EventPtr msg) {
		rsbInformer->publish(msg);
	}

private:
	std::string scope;
	ros::NodeHandle node;

protected:
	ros::Subscriber rosSubscriber;
	rsb::Factory& factory;
	typename rsb::Informer<T>::Ptr rsbInformer;
};

}

#endif /* PUBLISHER_H_ */
