#pragma once

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

class Publisher {
public:
    typedef boost::shared_ptr<Publisher> Ptr;
    virtual ~Publisher() {
    }
private:
};

template<class T>
class PublisherImpl: public Publisher {
public:

    PublisherImpl(const std::string &scope, ros::NodeHandle &node) :
		factory(rsb::getFactory()), scope(scope), node(node) {

#ifndef SKIPCONVERTER
                try {
                    boost::shared_ptr<rsb::converter::ProtocolBufferConverter<T> >
                                    converter(new rsb::converter::ProtocolBufferConverter<T>());

                    rsb::converter::converterRepository<std::string>()->registerConverter(
                                    converter);
                } catch (std::invalid_argument &e) {
                    ROS_WARN("Warning: trying to register two converters for same type");
                }
#endif

		rsbInformer = factory.createInformer<T> (scope);

	}

	virtual ~PublisherImpl() {
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

class PublisherBuilder {
public:
    typedef boost::shared_ptr<PublisherBuilder> Ptr;
    PublisherBuilder(const std::string &publisherName): publisherName(publisherName) {
    }
    virtual std::string getPublisherName() const {
        return publisherName;
    }
    virtual Publisher::Ptr build(const std::string &topicIn, const std::string &scopeOut, ros::NodeHandle &node) const = 0;
    virtual ~PublisherBuilder() {
    }
protected:
    std::string publisherName;
};

#define CREATE_PUBLISHER_BUILDER_NESTED(PUBLISHER_NAME) class Builder: public PublisherBuilder {\
public:\
    Builder(const std::string &publisherName) :\
            PublisherBuilder(publisherName) {\
    }\
    virtual Publisher::Ptr build(const std::string &topicIn, const std::string &scopeOut, ros::NodeHandle &node) const {\
        ROS_INFO_STREAM("Building publisher " << publisherName << ", topic: " << topicIn << ", scope: " << scopeOut);\
        return Publisher::Ptr(new PUBLISHER_NAME(topicIn, scopeOut, node));\
    }\
};

}
