#include <visualise/vis_grid.h>

namespace opti_rviz{

Vis_gird::Vis_gird(ros::NodeHandle& node,
                   const std::string& topic_name)
{
    publisher = node.advertise<visualization_msgs::MarkerArray>(topic_name, 10);
    r = 1;
    g = 0;
    b = 0;
    scale = 0.006;
    bIs2D = false;
    default_z = 0.2;
    lifetime  = 0;

}

void Vis_gird::initialise(const std::string& frame_id, const arma::mat& points,int marker_display_type){


    if(points.n_cols == 2)
    {
        bIs2D = true;
    }else if(points.n_cols == 3){
        bIs2D = false;
    }else{
        ROS_WARN("Failed [Vis_gird::initialise] points should be 2D or 3D");
        return;
    }

    grid_msg.markers.resize(points.n_rows);
    visualization_msgs::Marker marker;

    for(std::size_t i = 0; i < points.n_rows;i++)
    {
        marker.points.resize(1);
        marker.header.frame_id          = frame_id;
        marker.type                     = marker_display_type;
        marker.points[0].x      = points(i,0);
        marker.points[0].y      = points(i,1);
        if(bIs2D){
            marker.points[0].z  = default_z;
        }else{
            marker.points[0].z  = points(i,2);
        }
        marker.lifetime         = ros::Duration(lifetime);
        marker.color.r          = r;
        marker.color.g          = g;
        marker.color.b          = b;
        marker.color.a          = 1.0f;
        marker.scale.x          = scale;
        marker.scale.y          = scale;
        marker.scale.z          = scale;
        marker.id               = i;
        marker.header.stamp     = ros::Time::now();
        grid_msg.markers[i]     = marker;
    }
}


void Vis_gird::update(const arma::mat &points, const arma::colvec& weights){
    if(weights.n_elem == grid_msg.markers.size()){

        if(bIs2D)
        {
            update_2D(points,weights);
        }else{
            update_3D(points,weights);
        }
    }else{

        if(bIs2D){
            update_2D(points);

        }else{
            update_3D(points);
        }
    }
}

void Vis_gird::update(const arma::mat &points, const acolor &colors){
    if(bIs2D)
    {
        for(std::size_t i = 0; i < points.n_rows;i++){
            grid_msg.markers[i].points[0].x      = points(i,0);
            grid_msg.markers[i].points[0].y      = points(i,1);
            grid_msg.markers[i].points[0].z      = default_z;
            grid_msg.markers[i].header.stamp     = ros::Time::now();
            grid_msg.markers[i].color.a          = colors[i][0];
            grid_msg.markers[i].color.r          = colors[i][1];
            grid_msg.markers[i].color.g          = colors[i][2];
            grid_msg.markers[i].color.b          = colors[i][3];
        }

    }else{
        for(std::size_t i = 0; i < points.n_rows;i++){
            grid_msg.markers[i].points[0].x      = points(i,0);
            grid_msg.markers[i].points[0].y      = points(i,1);
            grid_msg.markers[i].points[0].z      = points(i,2);
            grid_msg.markers[i].header.stamp     = ros::Time::now();
            grid_msg.markers[i].color.a          = colors[i][0];
            grid_msg.markers[i].color.r          = colors[i][1];
            grid_msg.markers[i].color.g          = colors[i][2];
            grid_msg.markers[i].color.b          = colors[i][3];
        }
    }
}

void Vis_gird::update(const acolor & colors){
    assert(colors.size() == grid_msg.markers.size());
    for(std::size_t i = 0; i < colors.size();i++){
        grid_msg.markers[i].header.stamp     = ros::Time::now();
        grid_msg.markers[i].color.a          = colors[i][0];
        grid_msg.markers[i].color.r          = colors[i][1];
        grid_msg.markers[i].color.g          = colors[i][2];
        grid_msg.markers[i].color.b          = colors[i][3];
    }
}




void Vis_gird::update_2D(const arma::mat &points, const arma::colvec &weights){
    for(std::size_t i = 0; i < points.n_rows;i++){
        grid_msg.markers[i].points[0].x      = points(i,0);
        grid_msg.markers[i].points[0].y      = points(i,1);
        grid_msg.markers[i].points[0].z      = default_z;
        grid_msg.markers[i].header.stamp     = ros::Time::now();
        grid_msg.markers[i].color.a          = weights(i);
    }
}

void Vis_gird::update_3D(const arma::mat &points, const arma::colvec &weights){
    for(std::size_t i = 0; i < points.n_rows;i++)
    {
        grid_msg.markers[i].points[0].x      = points(i,0);
        grid_msg.markers[i].points[0].y      = points(i,1);
        grid_msg.markers[i].points[0].z      = points(i,2);
        grid_msg.markers[i].header.stamp     = ros::Time::now();
        grid_msg.markers[i].color.a          = weights(i);
    }
}

void Vis_gird::update_2D(const arma::mat &points){
    for(std::size_t i = 0; i < points.n_rows;i++)
    {
        grid_msg.markers[i].points[0].x      = points(i,0);
        grid_msg.markers[i].points[0].y      = points(i,1);
        grid_msg.markers[i].points[0].z      = default_z;
        grid_msg.markers[i].header.stamp     = ros::Time::now();
    }
}

void Vis_gird::update_3D(const arma::mat &points){
    for(std::size_t i = 0; i < points.n_rows;i++)
    {
        grid_msg.markers[i].points[0].x      = points(i,0);
        grid_msg.markers[i].points[0].y      = points(i,1);
        grid_msg.markers[i].points[0].z      = points(i,2);
        grid_msg.markers[i].header.stamp     = ros::Time::now();
    }
}


void Vis_gird::publish(){
    publisher.publish(grid_msg);
}


}
