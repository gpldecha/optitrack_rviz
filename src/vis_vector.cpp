#include <visualise/vis_vector.h>

namespace opti_rviz{

Arrow::Arrow(){}

Arrow::Arrow(const tf::Vector3& origin,const tf::Vector3& direction, const std::string& name):
    origin(origin),
    direction(direction),
    name(name)
{
    scale.x = 0.01;
    scale.y = 0.015;
    scale.z = 0.015;
}

void Arrow::set_pos_dir(const tf::Vector3& origin,const tf::Vector3& direction){
    this->origin    = origin;
    this->direction = direction;
}

void Arrow::set_rgba(double r, double g, double b, double a){
    color.a = a;color.r = r; color.g = g; color.b = b;
}

void Arrow::set_scale(double shaft_diameter, double head_diameter, double head_length){
    scale.x = shaft_diameter;
    scale.y = head_diameter;
    scale.z = head_length;
}

void Arrow::print()const {
    std::cout<< "== Arrow ==" << std::endl;
    std::cout<< "origin: " << origin.getX() << " " << origin.getY() << " " << origin.getZ() << std::endl;
    std::cout<< "direction: " << direction.getX() << " " << direction.getY() << " " << direction.getZ() << std::endl;
    std::cout<< "name: " << name << std::endl;
}

Vis_vectors::Vis_vectors(ros::NodeHandle& node,const std::string& topic_name)
    :topic_name(topic_name)
{
    vector_pub      = node.advertise<visualization_msgs::MarkerArray>(topic_name, 10);
    bInitialised    = false;
}

void Vis_vectors::initialise(const std::string& frame_id,const std::vector<Arrow>& vectors){
    vector_marke_array.markers.resize(vectors.size());
    for(std::size_t i = 0; i < vectors.size();i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.type            = visualization_msgs::Marker::ARROW;
        marker.color           = vectors[i].color;
        marker.scale           = vectors[i].scale;
        marker.lifetime        = ros::Duration(1);
        marker.id              = i;
        marker.points.resize(2);
        vector_marke_array.markers[i] = marker;
    }
    bInitialised = true;
}

void Vis_vectors::update(const std::vector<Arrow>& vectors){

    if(!bInitialised){
        ROS_WARN_STREAM_THROTTLE(1.0,"Did not intialise: " << topic_name << " vis_vector object!");
        return;
    }

    assert(vectors.size() == vector_marke_array.markers.size());

    for(std::size_t i = 0; i < vectors.size();i++){

        vector_marke_array.markers[i].header.stamp = ros::Time::now();
        vector_marke_array.markers[i].id = i;

        vector_marke_array.markers[i].points[0].x = vectors[i].origin[0];
        vector_marke_array.markers[i].points[0].y = vectors[i].origin[1];
        vector_marke_array.markers[i].points[0].z = vectors[i].origin[2];

        vector_marke_array.markers[i].points[1].x = vectors[i].origin[0] + vectors[i].direction[0];
        vector_marke_array.markers[i].points[1].y = vectors[i].origin[1] + vectors[i].direction[1];
        vector_marke_array.markers[i].points[1].z = vectors[i].origin[2] + vectors[i].direction[2];
    }
}

void Vis_vectors::publish(){
    vector_pub.publish(vector_marke_array);
}


}
