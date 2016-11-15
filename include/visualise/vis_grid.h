#ifndef VIS_GRID_H_
#define VIS_GRID_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <vector>
#include <tf/LinearMath/Vector3.h>

#include <armadillo>
#include <array>

#include "visualise/vis_utils.h"


namespace opti_rviz{

class Vis_gird{


public:

    Vis_gird(ros::NodeHandle& node, const std::string& topic_name);

    void initialise(const std::string& frame_id, const arma::mat &points, int marker_display_type=visualization_msgs::Marker::SPHERE_LIST);

    void update(const arma::mat &points, const arma::colvec& weights = arma::colvec());

    void update(const arma::mat &points, const acolor &colors);

    void update(const acolor & colors);

    void publish();

private:

    void update_2D(const arma::mat &points,const arma::colvec& weights);
    void update_3D(const arma::mat &points,const arma::colvec& weights);
    void update_2D(const arma::mat &points);
    void update_3D(const arma::mat &points);

public:

    float r, g, b;
    float scale;
    double default_z;
    double lifetime;



private:

    ros::Publisher                      publisher;
    visualization_msgs::MarkerArray     grid_msg;
    int                                 marker_display_type;
    bool                                bIs2D;



};



}


#endif
