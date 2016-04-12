#include "visualise/vis_line.h"

namespace opti_rviz{


Lines::Lines(ros::NodeHandle& nh, const std::string& topic_name){

    publisher  = nh.advertise<visualization_msgs::MarkerArray>(topic_name, 10);
}

void Lines::initialise(const std::string& frame_id,const std::vector<double>& points){

    marke_array.markers.resize(1);

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.type            = visualization_msgs::Marker::LINE_LIST;
    marker.color.a         = 1.0f;

    marker.color.r         = 1;
    marker.color.g         = 0;
    marker.color.b         = 1;

    marker.lifetime        = ros::Duration(1);
    marker.scale.x         = 0.05;
    marker.points.resize(2);
    marker.points[1].x      = 1;

    marke_array.markers[0] = marker;


}

void Lines::update(){
    publisher.publish(marke_array);
}

}
