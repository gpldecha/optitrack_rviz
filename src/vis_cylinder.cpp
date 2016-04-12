#include "visualise/vis_cylinder.h"

namespace opti_rviz{


Cylinder::Cylinder(){
    position.x = 0; position.y = 0; position.z = 0;
    orientation.w = 1; orientation.x = 0;  orientation.y = 0; orientation.z = 0;
    set_rgba(1,0,0,1);
    set_scale(0.1,0.1,0.1);
}

Cylinder::Cylinder(const tf::Vector3& position,const tf::Quaternion orientation)
{
    set_scale(0.1,0.1,0.1);
    set_pos(position,orientation);
    set_rgba(1,0,0,1);
}

void Cylinder::set_pos(const tf::Vector3& position,const tf::Quaternion orientation){
    tf_pos    = position;
    tf_orient = orientation;
    this->position.x    = position.getX();
    this->position.y    = position.getY();
    this->position.z    = position.getZ();
    this->orientation.w = orientation.getW();
    this->orientation.x = orientation.getX();
    this->orientation.y = orientation.getY();
    this->orientation.z = orientation.getZ();
}

void Cylinder::set_FR_tip(){
    tf_rot.setRotation(tf_orient);
    tf_pos_shift = (tf_rot * (tf_scale/2.0)) + tf_pos;
    position.x = tf_pos_shift.getX();
    position.y = tf_pos_shift.getY();
    position.z = tf_pos_shift.getZ();
}

void Cylinder::set_rgba(double r, double g, double b, double a){
    color.a = a; color.r = r; color.g = g; color.b = b;
}

void Cylinder::set_scale(double x, double y, double z){
    scale.x = x; scale.y = y; scale.z = z;
    tf_scale.setX(0);
    tf_scale.setY(0);
    tf_scale.setZ(scale.z);
}

Vis_cylinder::Vis_cylinder(ros::NodeHandle& node, const std::string&  topic_name){
    publisher     = node.advertise<visualization_msgs::MarkerArray>(topic_name, 10);
}

void Vis_cylinder::initialise(const std::string& frame_id,const std::vector<Cylinder>& cylinders){

    marker_array.markers.resize(cylinders.size());
    for(std::size_t i = 0; i < cylinders.size();i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id      = frame_id;
        marker.scale                = cylinders[i].scale;
        marker.pose.position        = cylinders[i].position;
        marker.pose.orientation     = cylinders[i].orientation;
        marker.color                = cylinders[i].color;
        marker.lifetime             = ros::Duration(1);
        marker.type                 = visualization_msgs::Marker::CYLINDER;
        marker.header.stamp         = ros::Time::now();
        marker.id                   = i;
        marker.frame_locked         = false;
        marker_array.markers[i]     = marker;
    }
}

void Vis_cylinder::update(const std::vector<Cylinder> &cylinders){

    for(std::size_t i = 0; i < cylinders.size();i++){
        marker_array.markers[i].scale                = cylinders[i].scale;
        marker_array.markers[i].pose.position        = cylinders[i].position;
        marker_array.markers[i].pose.orientation     = cylinders[i].orientation;
        marker_array.markers[i].color                = cylinders[i].color;
        marker_array.markers[i].header.stamp         = ros::Time::now();

    }
    publisher.publish(marker_array);
}

void Vis_cylinder::publish(){
    publisher.publish(marker_array);
}

}
