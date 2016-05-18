#ifndef OPTI_RVIZ_PUBLISHER_H_
#define OPTI_RVIZ_PUBLISHER_H_


#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <armadillo>


namespace opti_rviz{

class Publisher{

public:

    Publisher(ros::NodeHandle& nh, const std::string topic_name);

    void publish(const arma::vec& data);

private:

    ros::Publisher                  publisher;
    std_msgs::Float64MultiArray     msg;

};

}

#endif
