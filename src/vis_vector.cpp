#include <visualise/vis_vector.h>

namespace opti_rviz{

Vis_vectors::Vis_vectors(ros::NodeHandle& node,const std::string& topic_name){

    vector_pub  = node.advertise<visualization_msgs::MarkerArray>(topic_name, 10);

    scale = 1;
    r     = 1;
    g     = 0;
    b     = 0;
}

void Vis_vectors::set_color(const std::vector<tf::Vector3>& colors){
    this->colors = colors;
}

void Vis_vectors::initialise(const std::string& frame_id,const std::vector<Arrow>& vectors){
    vector_marke_array.markers.resize(vectors.size());



    for(std::size_t i = 0; i < vectors.size();i++){
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id;
        marker.type            = visualization_msgs::Marker::ARROW;
        marker.color.a         = 1.0f;

        if(colors.size() == vectors.size()){
            marker.color.r         = colors[i][0];
            marker.color.g         = colors[i][1];
            marker.color.b         = colors[i][2];
        }else{
            marker.color.r         = r;
            marker.color.g         = g;
            marker.color.b         = b;
        }

        marker.scale.x         = scale;
        marker.scale.y         = scale;
        marker.scale.z         = scale;
        marker.points.resize(2);
        vector_marke_array.markers[i] = marker;
    }
}

void Vis_vectors::update(const std::vector<Arrow>& vectors){

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
