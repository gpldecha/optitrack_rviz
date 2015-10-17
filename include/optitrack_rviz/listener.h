#ifndef LISTENER_H_
#define LISTENER_H_


// ROS

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Scalar.h>


//STL

#include <string>

namespace opti_rviz{

class Listener{

public:

    Listener(const std::string &fixed_frame,const std::string &target_frame);

    void update_opti2rviz(tf::Vector3& origin,tf::Matrix3x3& orientation);

    void update(tf::Vector3& origin,tf::Matrix3x3& orientation);

private:

    void optitrack_to_rviz(tf::Quaternion& q);

private:

    tf::TransformListener   tf_listener;
    tf::StampedTransform    tf_transform;
    std::string             fixed_frame, target_frame;
    tf::Matrix3x3           tmp,tmp2,R1;
    tf::Quaternion          q,q_tmp;
    tf::Vector3             origin_tmp;
    bool                    bFirst;

};

}


#endif
