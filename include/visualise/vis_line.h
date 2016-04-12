#ifndef VIS_LINE_H_
#define VIS_LINE_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <tf/LinearMath/Vector3.h>

namespace opti_rviz {

class Lines{

public:

    Lines(ros::NodeHandle& nh, const std::string& topic_name);

    void initialise(const std::string& frame_id,const std::vector<double>& points);

    void update();

private:

    ros::Publisher                  publisher;
    visualization_msgs::MarkerArray marke_array;

};

}

#endif
