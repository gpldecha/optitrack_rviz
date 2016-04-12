#include <visualise/vis_grid.h>

namespace opti_rviz{

Vis_gird::Vis_gird(      ros::NodeHandle& node,
                   const std::string& topic_name
                   )
{
    publisher = node.advertise<visualization_msgs::MarkerArray>(topic_name, 10);
    r = 1;
    g = 0;
    b = 0;
    scale = 0.006;

}

void Vis_gird::initialise(const std::string& frame_id, const arma::mat& points,int marker_display_type){


    grid_msg.markers.resize(points.n_rows);
    visualization_msgs::Marker marker;

    for(std::size_t i = 0; i < points.n_rows;i++)
    {
        marker.points.resize(1);
        marker.header.frame_id          = frame_id;
        marker.type                     = marker_display_type;
      // marker.action                   = visualization_msgs::Marker::ADD;
        marker.color.a                  = 1.0f;
        marker.points[0].x      = points(i,0);
        marker.points[0].y      = points(i,1);
        marker.points[0].z      = points(i,2);
        marker.lifetime         = ros::Duration(1);
        marker.color.r          = r;
        marker.color.g          = g;
        marker.color.b          = b;
        marker.scale.x          = scale;
        marker.scale.y          = scale;
        marker.scale.z          = scale;
        marker.id               = i;
        marker.header.stamp     = ros::Time::now();
        grid_msg.markers[i]     = marker;
    }
}

void Vis_gird::update(const arma::mat &points, const arma::colvec& weights){
    std::cout<< "Vis_grid::update" << std::endl;
    if(weights.n_elem == grid_msg.markers.size()){
        for(std::size_t i = 0; i < points.n_rows;i++)
        {
            grid_msg.markers[i].points[0].x      = points(i,0);
            grid_msg.markers[i].points[0].y      = points(i,1);
            grid_msg.markers[i].points[0].z      = points(i,2);
            grid_msg.markers[i].header.stamp     = ros::Time::now();
            grid_msg.markers[i].color.a          = weights(i);
        }
    }else{
        for(std::size_t i = 0; i < points.n_rows;i++)
        {
            grid_msg.markers[i].points[0].x      = points(i,0);
            grid_msg.markers[i].points[0].y      = points(i,1);
            grid_msg.markers[i].points[0].z      = points(i,2);
            grid_msg.markers[i].header.stamp     = ros::Time::now();

        }
    }
}

void Vis_gird::publish(){
    std::cout<< "Vis_grid::publish()" << std::endl;
    publisher.publish(grid_msg);
}


}
