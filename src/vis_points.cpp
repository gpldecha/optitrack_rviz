#include <visualise/vis_points.h>

namespace opti_rviz{

Vis_points::Vis_points(ros::NodeHandle& node,const std::string& topic_name,int marker_display_type){

    point_pub  = node.advertise<visualization_msgs::Marker>(topic_name, 10);

    scale = 1;
    alpha = 1;
    r     = 1;
    g     = 0;
    b     = 0;

    point_m.type            = marker_display_type;


}

void Vis_points::initialise_markers(const std::string& frame_id, const std::size_t num_points){

    point_m.points.resize(num_points);
    point_m.header.frame_id = frame_id;
    point_m.scale.x         = scale;
    point_m.scale.y         = scale;
    point_m.scale.z         = scale;
    point_m.color.a         = alpha;
    point_m.color.r         = r;
    point_m.color.g         = g;
    point_m.color.b         = b;
    point_m.lifetime        = ros::Duration(5);
    point_m.action          = visualization_msgs::Marker::ADD;
    point_m.header.stamp    = ros::Time::now();
    point_m.id              = 0;


}

void Vis_points::initialise(const std::string& frame_id,const std::vector<tf::Vector3>& points){

    initialise_markers(frame_id,points.size());

    for(std::size_t i = 0; i < points.size();i++){
          point_m.points[i].x    = points[i].x();
          point_m.points[i].y    = points[i].y();
          point_m.points[i].z    = points[i].z();
    }
}

void Vis_points::initialise(const std::string& frame_id, const arma::fmat& points){

    assert(points.n_cols >= 2);

    initialise_markers(frame_id,points.n_rows);

    for(std::size_t i = 0; i < points.n_rows;i++){
          point_m.points[i].x    = points(i,0);
          point_m.points[i].y    = points(i,1);
          point_m.points[i].z    = points(i,2);
    }
}


void Vis_points::update(const std::vector<tf::Vector3>& points){
    point_m.header.stamp    = ros::Time::now();
    for(std::size_t i = 0; i < points.size();i++){
          point_m.points[i].x    = points[i].x();
          point_m.points[i].y    = points[i].y();
          point_m.points[i].z    = points[i].z();
    }
}

void Vis_points::update(const arma::fmat& points,const colors& colors){
    point_m.header.stamp    = ros::Time::now();
    if (colors.size() == point_m.points.size()){
    }
        for(std::size_t i = 0; i < points.n_rows;i++){
            point_m.points[i].x    = points(i,0);
            point_m.points[i].y    = points(i,1);
            point_m.points[i].z    = points(i,2);
        }
}

void Vis_points::publish(){
    point_pub.publish(point_m);
}



}
