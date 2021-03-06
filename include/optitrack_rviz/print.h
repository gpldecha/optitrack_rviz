#ifndef OPTI_RVIZ_PRINT_H_
#define OPTI_RVIZ_PRINT_H_

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <ros/ros.h>

namespace opti_rviz {

typedef enum {raw,angle} data_type;

inline float rad2deg(float rad){
    return rad * M_PI/180;
}

inline void print(const tf::Vector3& vec,const std::string& name = ""){
    std::cout << name << ": " << vec.x() << "\t" << vec.y() << "\t" << vec.z() << std::endl;
}

inline void print(const tf::Matrix3x3& mat,data_type dtype = raw,const std::string& name = ""){
    std::cout<< name << ":" << std::endl;
    if(dtype == raw){
        std::cout<< mat[0][0] <<  "\t" << mat[0][1] << "\t" << mat[0][2] << std::endl;
        std::cout<< mat[1][0] <<  "\t" << mat[1][1] << "\t" << mat[1][2] << std::endl;
        std::cout<< mat[2][0] <<  "\t" << mat[2][1] << "\t" << mat[2][2] << std::endl;
    }else{
        std::cout<< cos(mat[0][0]) <<  "\t" << cos(mat[0][1]) << "\t" << cos(mat[0][2]) << std::endl;
        std::cout<< cos(mat[1][0]) <<  "\t" << cos(mat[1][1]) << "\t" << cos(mat[1][2]) << std::endl;
        std::cout<< cos(mat[2][0]) <<  "\t" << cos(mat[2][1]) << "\t" << cos(mat[2][2]) << std::endl;
    }

}

inline void ros_stream(const arma::vec& v,const std::string& name,double throttle_time=1.0)
{
    if(v.n_elem == 1){
        ROS_INFO_STREAM_THROTTLE(throttle_time,name << ": " << v(0));
    }else if(v.n_elem > 1){

        for(std::size_t i = 0; i < v.n_elem -1;i++){

        }

    }


}



}

#endif
