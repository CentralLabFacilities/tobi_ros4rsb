#pragma once

#define BOOST_SIGNALS_NO_DEPRECATION_WARNING
#include <rsb/Listener.h>
#include <rsb/Factory.h>
#include <string>

#include <rsb/converter/Repository.h>
#include <rsb/converter/ProtocolBufferConverter.h>

#include <ros/ros.h>

/**
 * This is a abstract class defining the outline of every Publisher that wraps ROS topics to RST types.
 */
namespace ros4rsb {

class Listener {
public:
    typedef boost::shared_ptr<Listener> Ptr;
    virtual ~Listener() {
    }
private:
};

template<class RsbType>
class ListenerImpl: public Listener {
public:

    ListenerImpl(const std::string &scopeIn) :
		factory(rsb::getFactory()) {
		this->scope = scopeIn;

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
		boost::function<void(boost::shared_ptr<RsbType>)> cb(boost::bind(&ListenerImpl::callback, this, _1));
		rsbListener->addHandler(rsb::HandlerPtr(new rsb::DataFunctionHandler<RsbType>(cb)));
	}

	virtual ~ListenerImpl() {
	}

	virtual void callback(boost::shared_ptr<RsbType> data) = 0;

private:
	std::string scope;

protected:
	rsb::Factory& factory;
	typename rsb::ListenerPtr rsbListener;
};

class ListenerBuilder {
public:
    typedef boost::shared_ptr<ListenerBuilder> Ptr;
    ListenerBuilder(const std::string &listenerName): listenerName(listenerName) {
    }
    virtual std::string getListenerName() const {
        return listenerName;
    }
    virtual Listener::Ptr build(const std::string &scopeIn, const std::string &topicOut, ros::NodeHandle &node) const = 0;
    virtual ~ListenerBuilder() {
    }
protected:
    std::string listenerName;
};

#define CREATE_LISTENER_BUILDER_NESTED(LISTENER_NAME) class Builder: public ListenerBuilder {\
public:\
    Builder(const std::string &listenerName) :\
            ListenerBuilder(listenerName) {\
    }\
    virtual Listener::Ptr build(const std::string &scopeIn, const std::string &topicOut, ros::NodeHandle &node) const {\
        ROS_INFO_STREAM("Building listener " << listenerName << ", topic: " << topicOut << ", scope: " << scopeIn);\
        return Listener::Ptr(new LISTENER_NAME(scopeIn, topicOut, node));\
    }\
};

}
