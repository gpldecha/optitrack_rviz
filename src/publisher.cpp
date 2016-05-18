#include "optitrack_rviz/publisher.h"

namespace opti_rviz{


Publisher::Publisher(ros::NodeHandle& nh, const std::string topic_name){
    publisher = nh.advertise<std_msgs::Float64MultiArray>(topic_name,5);
}

void Publisher::publish(const arma::vec& data){
    if(msg.data.size() != data.n_elem){
        msg.data.resize(data.n_elem);
    }

    for(std::size_t i = 0; i < data.n_elem;i++){
        msg.data[i] = data(i);
    }
    publisher.publish(msg);
}


}
