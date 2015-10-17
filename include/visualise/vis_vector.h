#ifndef VIS_VECTOR_H_
#define VIS_VECTOR_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <tf/LinearMath/Vector3.h>

#include <utility>

namespace opti_rviz{

class Arrow{
public:
    Arrow(){}
    Arrow(const tf::Vector3& origin,const tf::Vector3& direction):origin(origin),direction(direction){}
    tf::Vector3 origin;
    tf::Vector3 direction;
};

class Vis_vectors{

public:

    Vis_vectors(ros::NodeHandle& node,const std::string& topic_name);

    void set_color(const std::vector<tf::Vector3>& colors);

    void initialise(const std::string& frame_id, const std::vector<Arrow>& vectors);

    void update(const std::vector<Arrow> &vectors);

    void publish();

public:

    float                      scale;
    float                      r;
    float                      g;
    float                      b;

private:


    ros::Publisher                  vector_pub;
    visualization_msgs::MarkerArray vector_marke_array;
    std::vector<tf::Vector3>        colors;



};

}

#endif
