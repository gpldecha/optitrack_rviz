#ifndef VIS_POINT_CLOUD_H_
#define VIS_POINT_CLOUD_H_

#include <ros/ros.h>
#include <armadillo>

#include <sensor_msgs/PointCloud.h>

namespace opti_rviz{

class Vis_point_cloud{

public:
    typedef enum {DEFAULT,ONLY_HIGH_WEIGHTS} display_mode;

    enum class CHANNEL_TYPE{Intensity,RGB};

    class Intensity{
        //http://wiki.ros.org/rviz/DisplayTypes/PointCloudShared#channels
        public:
        Intensity(){
            min_i = 0; max_i = 1; min_c =0;max_c=1;
        }

        double min_i, max_i, min_c, max_c;
    };


    typedef std::vector<std::array<float,3> > colors;

public:

    Vis_point_cloud(ros::NodeHandle& node,const std::string topic_name);

    void initialise(const std::string& frame_id, const arma::mat& points);

    void set_display_type(display_mode type);

    void set_channel(CHANNEL_TYPE channel_type);

    void update(const arma::mat &points, const double* weights);

    void update(const arma::mat &points, const colors &colors, const double* weights, double threashod=0.9);

    void update(const arma::mat& points,const colors& colors,const arma::colvec& weights, double threashod=0.9);

    void publish();


private:

    sensor_msgs::PointCloud              mPointCloud;
    ros::Publisher                       cloud_pub;
    display_mode                         vtype;
    CHANNEL_TYPE                         channel_type;
    Intensity                            intensity;

};

}


#endif
