#include <visualise/vis_point_cloud.h>

namespace opti_rviz{

Vis_point_cloud::Vis_point_cloud(ros::NodeHandle& node,const std::string topic_name){
    cloud_pub       = node.advertise<sensor_msgs::PointCloud >(topic_name, 10);
    channel_type    = CHANNEL_TYPE::RGB;
}

void Vis_point_cloud::set_display_type(display_mode type){
    vtype = type;
}

void Vis_point_cloud::set_channel(CHANNEL_TYPE channel_type){
    this->channel_type = channel_type;
}

void Vis_point_cloud::initialise(const std::string& frame_id, const arma::mat& points){

    mPointCloud.header.frame_id     =   frame_id;
    mPointCloud.header.stamp        =   ros::Time::now();
    this->vtype                     =   vtype;

    mPointCloud.points.resize(points.n_rows);

    if(channel_type == CHANNEL_TYPE::RGB)
    {
        // use RGB
        mPointCloud.channels.resize(3);
        mPointCloud.channels[0].name = "r";
        mPointCloud.channels[0].values.resize(points.n_rows);
        mPointCloud.channels[1].name = "g";
        mPointCloud.channels[1].values.resize(points.n_rows);
        mPointCloud.channels[2].name = "b";
        mPointCloud.channels[2].values.resize(points.n_rows);
    }else if(channel_type == CHANNEL_TYPE::Intensity)
    {
        mPointCloud.channels.resize(1);
        mPointCloud.channels[0].name = "intensity";
        mPointCloud.channels[0].values.resize(points.n_rows);
    }
}

void Vis_point_cloud::update(const arma::mat &points, const double *weights){
    mPointCloud.header.stamp        =   ros::Time::now();

    if(mPointCloud.points.size() != points.n_rows){
        mPointCloud.points.resize(points.n_rows);
        mPointCloud.channels[0].values.resize(points.n_rows);
    }

    for(std::size_t i = 0; i < points.n_rows;i++){
        mPointCloud.points[i].x             = points(i,0);
        mPointCloud.points[i].y             = points(i,1);
        mPointCloud.points[i].z             = points(i,2);
        mPointCloud.channels[0].values[i]   = weights[i];
    }

}

void Vis_point_cloud::update(const arma::mat &points, const colors &colors, const double* weights, double threashod){

    mPointCloud.header.stamp        =   ros::Time::now();

    if(vtype == DEFAULT){
       // ROS_INFO_STREAM_THROTTLE(1.0,"DEFAULT  mPointCloud.points.size(): " <<  mPointCloud.points.size());


        if(channel_type == CHANNEL_TYPE::RGB){
            for(std::size_t i = 0; i < points.n_rows;i++){
                mPointCloud.points[i].x             = points(i,0);
                mPointCloud.points[i].y             = points(i,1);
                mPointCloud.points[i].z             = points(i,2);
                mPointCloud.channels[0].values[i]   = colors[i][0];
                mPointCloud.channels[1].values[i]   = colors[i][1];
                mPointCloud.channels[2].values[i]   = colors[i][2];
            }
        }else if(channel_type == CHANNEL_TYPE::Intensity){
            for(std::size_t i = 0; i < points.n_rows;i++){
                mPointCloud.points[i].x             = points(i,0);
                mPointCloud.points[i].y             = points(i,1);
                mPointCloud.points[i].z             = points(i,2);
                mPointCloud.channels[0].values[i]   = weights[i];
            }
        }

    }else if(vtype == ONLY_HIGH_WEIGHTS){
        ROS_INFO_STREAM_THROTTLE(1.0,"only weights");

        mPointCloud.points.clear();
        mPointCloud.channels[0].values.clear();
        mPointCloud.channels[1].values.clear();
        mPointCloud.channels[2].values.clear();
        geometry_msgs::Point32 point;
        for(std::size_t i = 0; i < points.n_rows;i++){
            if(weights[i] > threashod){
                point.x  = points(i,0);
                point.y  = points(i,1);
                point.z  = points(i,2);
                mPointCloud.points.push_back(point);
                mPointCloud.channels[0].values.push_back(colors[i][0]);
                mPointCloud.channels[1].values.push_back(colors[i][1]);
                mPointCloud.channels[2].values.push_back(colors[i][2]);
            }
        }
    }
}

void Vis_point_cloud::update(const arma::mat &points, const colors &colors, const arma::colvec& weights, double threashod){

    update(points,colors,weights.memptr(),threashod);


}

void Vis_point_cloud::publish(){
    cloud_pub.publish(mPointCloud);
}


}
