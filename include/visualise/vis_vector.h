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
    Arrow();

    Arrow(const tf::Vector3& origin,const tf::Vector3& direction, const std::string& name);

    void set_pos_dir(const tf::Vector3& origin,const tf::Vector3& direction);

    void set_rgba(double r, double g, double b, double a);

    void set_scale(double shaft_diameter, double head_diameter, double head_length);

    void print() const;

public:

    tf::Vector3                 origin;
    tf::Vector3                 direction;
    geometry_msgs::Vector3      scale;
    std_msgs::ColorRGBA         color;
    std::string                 name;

};

class Vis_vectors{

public:

    Vis_vectors(ros::NodeHandle& node,const std::string& topic_name);

    void initialise(const std::string& frame_id, const std::vector<Arrow>& vectors);

    void update(const std::vector<Arrow> &vectors);

    void publish();

private:


    ros::Publisher                  vector_pub;
    visualization_msgs::MarkerArray vector_marke_array;
    std::vector<tf::Vector3>        colors;
    std::string                     topic_name;
    bool                            bInitialised;



};

}

#endif
