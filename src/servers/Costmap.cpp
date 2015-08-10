#include "Costmap.h"

using namespace boost;
using namespace std;
using namespace ros;
using namespace tf;

namespace ros4rsb {
    Costmap::Costmap(std::string name, NodeHandle node){
        this->name = name;
        this->node = node;
        function<void(const nav_msgs::OccupancyGrid::ConstPtr&)> m0 = bind(mem_fn(&Costmap::CostmapFullUpdate),this,_1);
        function<void(const nav_msgs::OccupancyGrid::ConstPtr&)> m1 = bind(mem_fn(&Costmap::CostmapPartialUpdate),this,_1);
        string topicFullMessage = "/move_base/local_costmap/costmap";
        string topicPartialMessage = "/move_base/local_costmap/costmap_update";
        rosSubscriberFull = node.subscribe(topicFullMessage, 1000, m0);
        std::cout << name << " subscribed to topic: " << topicFullMessage << std::endl;
        rosSubscriberPartial = node.subscribe(topicPartialMessage, 1000, m1);
        std::cout << name << " subscribed to topic: " << topicPartialMessage << std::endl;
        costmap = NULL;
    }
    
    void Costmap::CostmapFullUpdate(const nav_msgs::OccupancyGridConstPtr &message){
        if (costmap != NULL) {
            free(costmap);
        }
        costmap = new costmap_2d::Costmap2D(message->info.width,message->info.height,message->info.resolution,
                                 message->info.origin.position.x,message->info.origin.position.y);
        unsigned char* data = costmap->getCharMap();
        for (unsigned int i = 0; i < message->info.width*message->info.height; i++){
            data[i] = message->data[i];
        }
    }
    
    void Costmap::CostmapPartialUpdate(const nav_msgs::OccupancyGridConstPtr &message){
        std::cout << "got partial update" << endl;
    }
}