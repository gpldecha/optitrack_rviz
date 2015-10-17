#ifndef VIS_POINTS_H_
#define VIS_POINTS_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <vector>
#include <tf/LinearMath/Vector3.h>

#include <armadillo>
#include <array>


namespace opti_rviz{

class Vis_points{

    typedef std::vector<std::array<float,3> > colors;

public:

    Vis_points(ros::NodeHandle& node, const std::string& topic_name, int marker_display_type=visualization_msgs::Marker::SPHERE_LIST);

    void initialise(const std::string& frame_id, const std::vector<tf::Vector3>& points);
    void initialise(const std::string& frame_id, const arma::fmat& points);

    void update(const std::vector<tf::Vector3>& points);
    void update(const arma::fmat &points, const colors& colors = colors(0));

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


    ros::Publisher             point_pub;
    visualization_msgs::Marker point_m;


};

}

#endif
