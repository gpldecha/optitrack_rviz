#ifndef OPTI_RVIZ_TYPE_DEBUG_H_
#define OPTI_RVIZ_TYPE_DEBUG_H_

#include <armadillo>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>
#include <ros/ros.h>
#include "optitrack_rviz/type_conversion.h"

namespace opti_rviz {

namespace debug{

    template<typename T>
    inline void tf_debuf(const arma::Col<T>& pos,const std::string& name,const std::string fixed_frame="world"){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(pos(0),pos(1),pos(2)) );
        tf::Quaternion q;
        q.setEuler(0,0,0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), fixed_frame, name));
    }

    template<typename T>
    inline void tf_debuf(const arma::Col<T>& pos,const tf::Quaternion& q,const std::string& name,const std::string fixed_frame="world"){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(pos(0),pos(1),pos(2)) );
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), fixed_frame, name));
    }

    template<typename T>
    inline void tf_debuf(const arma::Col<T>& pos, const tf::Matrix3x3& Rot,const std::string& name,const std::string fixed_frame="world"){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(pos(0),pos(1),pos(2)) );
        tf::Quaternion q;
        Rot.getRotation(q);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), fixed_frame, name));
    }


    template<typename T>
    inline void tf_debuf(const arma::Col<T>& pos, const arma::Mat<T>& Rot,const std::string& name,const std::string fixed_frame="world"){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(pos(0),pos(1),pos(2)) );
        tf::Quaternion q;
        tf::Matrix3x3 Rot_;
        type_conv::mat2tf<T>(Rot,Rot_);
        Rot_.getRotation(q);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), fixed_frame, name));
    }

    inline void tf_debuf(const arma::colvec2& pos, const arma::mat22& Rot,const std::string& name,const std::string fixed_frame="world"){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(pos(0),pos(1),0) );
        tf::Quaternion q;
        tf::Matrix3x3 Rot_;
        Rot_[0][0] = Rot(0,0);
        Rot_[0][1] = Rot(0,1);
        Rot_[0][2] = 0;

        Rot_[1][0] = Rot(1,0);
        Rot_[1][1] = Rot(1,1);
        Rot_[1][2] = 0;

        Rot_[1][0] = 0;
        Rot_[1][1] = 0;
        Rot_[1][2] = 1;

        Rot_.getRotation(q);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), fixed_frame, name));
    }

    inline void tf_debuf(const arma::vec& pos,const std::string& name,const std::string fixed_frame="world"){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(pos(0),pos(1),pos(2)) );
        tf::Quaternion q;
        q.setEuler(0,0,0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),fixed_frame, name));
    }

    inline void tf_debuf(const tf::Vector3& pos,const std::string& name,const std::string fixed_frame="world"){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( pos );
        tf::Quaternion q(0,0,0,1);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), fixed_frame, name));
    }

    inline void tf_debuf(const tf::Vector3& pos, const tf::Matrix3x3& Rot,const std::string& name,const std::string fixed_frame="world"){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( pos );
        tf::Quaternion q;
        Rot.getRotation(q);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), fixed_frame, name));
    }

    inline void tf_debuf(const tf::Vector3& pos, const tf::Quaternion q,const std::string& name,const std::string fixed_frame="world"){
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( pos );
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), fixed_frame, name));
    }


}

}

#endif
