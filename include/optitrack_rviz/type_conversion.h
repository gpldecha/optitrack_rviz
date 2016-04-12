#ifndef OPTI_RVIZ_TYPE_CONVERSION_H_
#define OPTI_RVIZ_TYPE_CONVERSION_H_

#include <armadillo>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
#include <geometry_msgs/Pose.h>
#include <cmath>

namespace opti_rviz {

inline double deg2rad(double radian){
    return radian * M_PI / 180.0;
}

inline double rad2deg(double degree)
{
    return degree * 180.8 / M_PI;
}


class type_conv{

public:

    static void tf2mat(const tf::Matrix3x3& m1, arma::mat& m2){

        m2(0,0)    = m1[0][0];
        m2(0,1)    = m1[0][1];
        m2(0,2)    = m1[0][2];

        m2(1,0)    = m1[1][0];
        m2(1,1)    = m1[1][1];
        m2(1,2)    = m1[1][2];

        m2(2,0)    = m1[2][0];
        m2(2,1)    = m1[2][1];
        m2(2,2)    = m1[2][2];
    }

    static void tf2mat(const tf::Matrix3x3& m1, arma::fmat& m2){

        m2(0,0)    = m1[0][0];
        m2(0,1)    = m1[0][1];
        m2(0,2)    = m1[0][2];

        m2(1,0)    = m1[1][0];
        m2(1,1)    = m1[1][1];
        m2(1,2)    = m1[1][2];

        m2(2,0)    = m1[2][0];
        m2(2,1)    = m1[2][1];
        m2(2,2)    = m1[2][2];
    }

    static void mat2tf(const arma::mat33& m1, tf::Matrix3x3& m2){
        m2[0][0] = m1(0,0);
        m2[0][1] = m1(0,1);
        m2[0][2] = m1(0,2);

        m2[1][0] = m1(1,0);
        m2[1][1] = m1(1,1);
        m2[1][2] = m1(1,2);

        m2[2][0] = m1(2,0);
        m2[2][1] = m1(2,1);
        m2[2][2] = m1(2,2);
    }

    static void mat2tf(const arma::fmat33& m1, tf::Matrix3x3& m2){
        m2[0][0] = m1(0,0);
        m2[0][1] = m1(0,1);
        m2[0][2] = m1(0,2);

        m2[1][0] = m1(1,0);
        m2[1][1] = m1(1,1);
        m2[1][2] = m1(1,2);

        m2[2][0] = m1(2,0);
        m2[2][1] = m1(2,1);
        m2[2][2] = m1(2,2);
    }

    static void tf2vec(const tf::Vector3& v1, arma::colvec3& v2){
            v2(0) = v1.x();
            v2(1) = v1.y();
            v2(2) = v1.z();
    }

    static void tf2vec(const tf::Vector3& v1, arma::fcolvec3& v2){
            v2(0) = v1.x();
            v2(1) = v1.y();
            v2(2) = v1.z();
    }

    static void tf2vec4(const tf::Quaternion& q1, arma::colvec4& q2){
        q2(0) = q1.getX();
        q2(1) = q1.getY();
        q2(2) = q1.getZ();
        q2(3) = q1.getW();

    }

    static void setTransform(tf::Transform& trans, const arma::colvec3& origin, const arma::colvec4& orientation){
            trans.setOrigin(tf::Vector3(origin(0),origin(1),origin(2)));
            trans.setRotation(tf::Quaternion(orientation(0),orientation(1),orientation(2),orientation(3)));
    }

    static void vec2tf(const arma::colvec3& v1, tf::Vector3& v2){
        v2.setX(v1(0));
        v2.setY(v1(1));
        v2.setZ(v1(2));
    }

    static void vec2tf(const arma::fcolvec3& v1, tf::Vector3& v2){
        v2.setX(v1(0));
        v2.setY(v1(1));
        v2.setZ(v1(2));
    }

    static void tf2geom(const tf::Vector3& position,const tf::Quaternion& orientation,geometry_msgs::Pose& pos){
       pos.position.x = position.getX();
       pos.position.y = position.getY();
       pos.position.z = position.getZ();

       pos.orientation.x = orientation.getX();
       pos.orientation.y = orientation.getY();
       pos.orientation.z = orientation.getZ();
       pos.orientation.w = orientation.getW();
    }

};


class flip{

public:

    static void col(tf::Matrix3x3& mat,std::size_t i, std::size_t j){
        tf::Vector3 col_i = tf::Vector3(mat[0][i],mat[1][i],mat[2][i]);
        tf::Vector3 col_j = tf::Vector3(mat[0][j],mat[1][j],mat[2][j]);

        mat[0][i] = col_j.x();
        mat[1][i] = col_j.y();
        mat[2][i] = col_j.z();

        mat[0][j] = col_i.x();
        mat[1][j] = col_i.y();
        mat[2][j] = col_i.z();


    }

};

}




#endif
