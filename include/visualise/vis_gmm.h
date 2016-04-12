#ifndef VIS_GMM_H_
#define VIS_GMM_H_

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <armadillo>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>

namespace opti_rviz{

class Vis_gmm{

public:

     Vis_gmm(ros::NodeHandle& node, const std::string& topic_name);

     void initialise(const std::string& frame_id,
                     const arma::vec& weights,
                     const std::vector<arma::vec>& means,
                     const std::vector<arma::mat>& covariances);

     void initialise(const std::string& frame_id,
                     const arma::vec& mean,
                     const arma::mat& covariance);

     void update(const arma::vec& weights,const std::vector<arma::vec>& means, const std::vector<arma::mat>& covariances);

     void update(const arma::vec& mean, const arma::mat& covariance);

     void publish();

public:

     float r,g,b,a;

private:

     inline float rescale(float x, float old_min, float old_max, float new_min, float new_max){
         return (((x - old_min) * (new_max - new_min)) / (old_max - old_min)) + new_min;
     }

private:

     ros::Publisher                  publisher;
     visualization_msgs::MarkerArray gmm_msg;
     arma::vec                       eigval;
     arma::mat                       eigvec;
     tf::Matrix3x3                   tmp;
     tf::Quaternion                  q;
     std::string                     frame_id;
};

}


#endif
