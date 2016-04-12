#ifndef VIS_CYLINDER_H_
#define VIS_CYLINDER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

#include <armadillo>
#include <array>


namespace opti_rviz{

class Cylinder{

public:

    Cylinder();

    Cylinder(const tf::Vector3 &position, const tf::Quaternion orientation);

    void set_pos(const tf::Vector3& position,const tf::Quaternion orientation);

    void set_FR_tip();

    void set_rgba(double r, double g, double b, double a);

    void set_scale(double x, double y, double z);

public:

    tf::Vector3                 tf_pos,tf_pos_shift,tf_scale;
    tf::Quaternion              tf_orient;
    tf::Matrix3x3               tf_rot;
    geometry_msgs::Point        position;
    geometry_msgs::Quaternion   orientation;
    geometry_msgs::Vector3      scale;
    std_msgs::ColorRGBA         color;

};

class Vis_cylinder{

public:

    Vis_cylinder(ros::NodeHandle& node, const std::string& topic_name);

    void initialise(const std::string& frame_id,const std::vector<Cylinder>& cylinders);

    void update(const std::vector<Cylinder>& cylinders);

    void publish();

private:


    ros::Publisher                  publisher;
    visualization_msgs::MarkerArray marker_array;

};

}



#endif
