/* 
 * File:   SlamMapPublisher.cpp
 * Author: alangfel
 * 
 * Created on April 25, 2014, 2:02 PM
 */

#include "SlamMapPublisher.h"

#include <tf2/LinearMath/Quaternion.h> 
#include <tf2/LinearMath/Matrix3x3.h>


using namespace std;
using namespace ros;
using namespace boost;
using namespace rst;

namespace ros4rsb {

    SlamMapPublisher::SlamMapPublisher(
            string name,
            NodeHandle node) :
    Publisher(name, node),
    hasData(false) {

        string topic = "map";

        function<void(const nav_msgs::OccupancyGrid::ConstPtr&) > m0 = bind(
                mem_fn(&SlamMapPublisher::callback), this, _1);

        rosSubscriber = node.subscribe(topic, 1000, m0);
        std::cout << name << " subscribed to topic: " << topic << std::endl;

        runner = boost::thread(&SlamMapPublisher::publishThread, this);
        //        xcfPublisher->setOnlySendLast();

    }

    SlamMapPublisher::~SlamMapPublisher() {
    }

    void SlamMapPublisher::publishThread() {

        while (true) {

            {
                boost::mutex::scoped_lock lock(mutex);

                if (hasData) {

                    rsb::EventPtr event = rsbInformer->createEvent();
                    event->setData(slamMap);
                    this->publish(event);
                    //hasData = false;
//                    cout << "Send data!" << endl;
                    
                } else {
//                    cout << "SlamMapPublisher did not receive data yet" << endl;
                }
            }
            usleep(3000 * 1000);
        }
    }

    void SlamMapPublisher::callback(
            const nav_msgs::OccupancyGrid::ConstPtr &message) {

        vector<int8_t> ints;


        shared_ptr<navigation::OccupancyGrid2DInt> data(new navigation::OccupancyGrid2DInt());

        //set origin        
        data->mutable_origin()->mutable_translation()->set_x(static_cast<int> (message->info.origin.position.x));
        data->mutable_origin()->mutable_translation()->set_y(static_cast<int> (message->info.origin.position.y));
        data->mutable_origin()->mutable_translation()->set_z(static_cast<int> (message->info.origin.position.z));
        
        data->mutable_origin()->mutable_rotation()->set_qw(static_cast<int> (message->info.origin.orientation.w));
        data->mutable_origin()->mutable_rotation()->set_qx(static_cast<int> (message->info.origin.orientation.x));
        data->mutable_origin()->mutable_rotation()->set_qy(static_cast<int> (message->info.origin.orientation.y));
        data->mutable_origin()->mutable_rotation()->set_qz(static_cast<int> (message->info.origin.orientation.z));

        data->set_resolution(message->info.resolution);
        data->set_height(message->info.height);
        data->set_width(message->info.width);

        for (unsigned int x = 0; x < message->info.width; x++) {
            for (unsigned int y = 0; y < message->info.height; y++) {
                int8_t rosVal = message->data[x + y * message->info.width];
                data->mutable_map()->push_back(rosVal);
//                if(rosVal != -1) {
//                    printf("%d, ", rosVal);
//                }
            }
            
        }

        //set data pointer to map
//        data->set_map((char*)ints.data());

        this->slamMap = data;
        
        hasData = true;
    }
}
