#include "visualise/vis_cylinder.h"

namespace opti_rviz{


Vis_cylinder::Vis_cylinder(ros::NodeHandle& node, const std::string&  topic_name, int  marker_display_type){

    cylinder_pub     = node.advertise<visualization_msgs::Marker>(topic_name, 10);
    cylinder_m.type  = marker_display_type;

    scale = 0.2;
    r     = 1;
    g     = 0;
    b     = 0;
    alpha = 0.2;

}

void Vis_cylinder::initialise(const std::string& frame_id){

    cylinder_m.header.frame_id = frame_id;
    cylinder_m.scale.x         = scale;
    cylinder_m.scale.y         = scale;
    cylinder_m.scale.z         = 1;
    cylinder_m.color.a         = alpha;
    cylinder_m.color.r         = r;
    cylinder_m.color.g         = g;
    cylinder_m.color.b         = b;

    cylinder_m.pose.position.x = 0;
    cylinder_m.pose.position.y = 0;
    cylinder_m.pose.position.z = 0;

    cylinder_m.pose.orientation.x = 0;
    cylinder_m.pose.orientation.y = 0;
    cylinder_m.pose.orientation.z = 0;
    cylinder_m.pose.orientation.w = 1;

    cylinder_m.action          = visualization_msgs::Marker::ADD;
    cylinder_m.header.stamp    = ros::Time::now();
    cylinder_m.id              = 0;
    cylinder_m.frame_locked    = false;

}

void Vis_cylinder::update(const tf::Vector3& position,const tf::Quaternion& orientation){
    cylinder_m.pose.position.x    = position.x();
    cylinder_m.pose.position.y    = position.y();
    cylinder_m.pose.position.z    = position.z();

    cylinder_m.pose.orientation.x = orientation.x();
    cylinder_m.pose.orientation.y = orientation.y();
    cylinder_m.pose.orientation.z = orientation.z();
    cylinder_m.pose.orientation.w = orientation.w();
}



void Vis_cylinder::publish(){
    cylinder_m.header.stamp    = ros::Time::now();
    cylinder_pub.publish(cylinder_m);
}

}
