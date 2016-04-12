#include <visualise/vis_gmm.h>
#include <visualization_msgs/Marker.h>


namespace opti_rviz {

void tf2mat(arma::mat& m1,const tf::Matrix3x3& m2){
    m1(0,0) = m2[0][0];
    m1(1,0) = m2[1][0];
    m1(2,0) = m2[2][0];

    m1(0,1) = m2[0][1];
    m1(1,1) = m2[1][1];
    m1(2,1) = m2[2][1];

    m1(0,2) = m2[0][2];
    m1(1,2) = m2[1][2];
    m1(2,2) = m2[2][2];
}

void mat2tf(const arma::mat& m1,tf::Matrix3x3& m2){

    m2[0][0] =  m1(0,0);
    m2[1][0] =  m1(1,0);
    m2[2][0] =  m1(2,0);

    m2[0][1] =  m1(0,1);
    m2[1][1] =  m1(1,1);
    m2[2][1] =  m1(2,1);

    m2[0][2] =  m1(0,2);
    m2[1][2] =  m1(1,2);
    m2[2][2] =  m1(2,2);

}


Vis_gmm::Vis_gmm(ros::NodeHandle& node, const std::string& topic_name){
    publisher = node.advertise<visualization_msgs::MarkerArray>(topic_name, 10);
    a         = 1;
    r         = 1;
    g         = 0;
    b         = 0;

    eigvec.resize(3,3);
}


void Vis_gmm::initialise(const std::string& frame_id,
                         const arma::vec& weights,
                         const std::vector<arma::vec>& means,
                         const std::vector<arma::mat>& covariances
                         ){

    this->frame_id = frame_id;
    update(weights,means,covariances);
}

void Vis_gmm::initialise(const std::string& frame_id,
                         const arma::vec& mean,
                         const arma::mat& covariance){
    this->frame_id = frame_id;
    update(mean,covariance);
}


void Vis_gmm::update(const arma::vec& mean, const arma::mat& covariance){

    gmm_msg.markers.clear();
    gmm_msg.markers.resize(1);


    gmm_msg.markers[0].header.frame_id      = frame_id;
    gmm_msg.markers[0].type                 = visualization_msgs::Marker::SPHERE;
    gmm_msg.markers[0].lifetime             = ros::Duration(0.2);     //= visualization_msgs::Marker::ADD;


    gmm_msg.markers[0].color.a              = a;

    gmm_msg.markers[0].color.r              = r;
    gmm_msg.markers[0].color.g              = g;
    gmm_msg.markers[0].color.b              = b;
    gmm_msg.markers[0].points.resize(1);
    gmm_msg.markers[0].pose.position.x      = mean(0);
    gmm_msg.markers[0].pose.position.y      = mean(1);
    gmm_msg.markers[0].pose.position.z      = mean(2);

    mat2tf(eigvec,tmp);
    tmp.getRotation(q);

    eigvec.eye(3,3);


    gmm_msg.markers[0].pose.orientation.x   = q.x();
    gmm_msg.markers[0].pose.orientation.y   = q.y();
    gmm_msg.markers[0].pose.orientation.z   = q.z();
    gmm_msg.markers[0].pose.orientation.w   = q.w();

    gmm_msg.markers[0].id                   = 0;
    gmm_msg.markers[0].scale.x              = 3*sqrt(covariance(0,0));
    gmm_msg.markers[0].scale.y              = 3*sqrt(covariance(1,1));
    gmm_msg.markers[0].scale.z              = 3*sqrt(covariance(2,2));

}

void Vis_gmm::update(const arma::vec& weights,
                     const std::vector<arma::vec>& means,
                     const std::vector<arma::mat>& covariances)

{

    if(weights.n_elem > 0){

        //  std::cout<< "number elem: " << weights.n_elem << std::endl;

        gmm_msg.markers.clear();
        gmm_msg.markers.resize(weights.n_elem);

        float max_w = arma::max(weights);
        float min_w = arma::min(weights);

        for(std::size_t i = 0; i < weights.n_elem;i++){
            //  std::cout<< "("<<i<<")" << std::endl;
            //  covariances[i].print("converiance");
            //  means[i].print("means");

            gmm_msg.markers[i].header.frame_id      = frame_id;
            gmm_msg.markers[i].type                 = visualization_msgs::Marker::SPHERE;
            gmm_msg.markers[i].lifetime             = ros::Duration(0.2);     //= visualization_msgs::Marker::ADD;

            if(max_w == min_w){
                gmm_msg.markers[i].color.a          = 1;
            }else{
                gmm_msg.markers[i].color.a           = rescale(weights(i),max_w,min_w,0.1,0.95);
            }
            gmm_msg.markers[i].color.r              = r;
            gmm_msg.markers[i].color.g              = g;
            gmm_msg.markers[i].color.b              = b;
            gmm_msg.markers[i].points.resize(1);
            gmm_msg.markers[i].pose.position.x      = means[i](0);
            gmm_msg.markers[i].pose.position.y      = means[i](1);
            gmm_msg.markers[i].pose.position.z      = means[i](2);

            arma::eig_sym( eigval, eigvec, covariances[i] );
            mat2tf(eigvec,tmp);
            tmp.getRotation(q);

            // eigval.print("eigval(" + boost::lexical_cast<std::string>(i) + ")");

            gmm_msg.markers[i].pose.orientation.x   = q.x();
            gmm_msg.markers[i].pose.orientation.y   = q.y();
            gmm_msg.markers[i].pose.orientation.z   = q.z();
            gmm_msg.markers[i].pose.orientation.w   = q.w();

            gmm_msg.markers[i].id                   = i;
            gmm_msg.markers[i].scale.x              = 3*sqrt(eigval(0));
            gmm_msg.markers[i].scale.y              = 3*sqrt(eigval(1));
            gmm_msg.markers[i].scale.z              = 3*sqrt(eigval(2));

        }
    }
}

void Vis_gmm::publish(){
    publisher.publish(gmm_msg);
}


}
