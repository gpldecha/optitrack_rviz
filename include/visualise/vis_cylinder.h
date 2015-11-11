#ifndef VIS_CYLINDER_H_
#define VIS_CYLINDER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <vector>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>

#include <armadillo>
#include <array>


namespace opti_rviz{

class Vis_cylinder{

    typedef std::vector<std::array<float,3> > colors;

public:

    Vis_cylinder(ros::NodeHandle& node, const std::string& topic_name, int marker_display_type=visualization_msgs::Marker::CYLINDER);

    void initialise(const std::string& frame_id);

    void update(const tf::Vector3& position,const tf::Quaternion& orientation);

    void publish();

private:

    void initialise_markers(const std::string& frame_id, const std::size_t num_points);

public:

    float                      scale;
    float                      alpha;
    float                      r;
    float                      g;
    float                      b;

private:


    ros::Publisher             cylinder_pub;
    visualization_msgs::Marker cylinder_m;

};

}



#endif
