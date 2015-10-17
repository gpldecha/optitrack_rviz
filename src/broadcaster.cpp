#include "optitrack_rviz/broadcaster.h"

namespace opti_rviz{

Broadcaster::Broadcaster(const std::string &fixed_frame, const std::string &target_frame_rviz)
{
    geom_message.header.frame_id = fixed_frame;
    geom_message.child_frame_id  = target_frame_rviz;

    q.setEuler(0,0,0);

}


void Broadcaster::update(const tf::Vector3& origin, const tf::Matrix3x3& R){

    R.getRotation(q);

    geom_message.transform.translation.x = origin.x();
    geom_message.transform.translation.y = origin.y();
    geom_message.transform.translation.z = origin.z();

    geom_message.transform.rotation.x = q.getX();
    geom_message.transform.rotation.y = q.getY();
    geom_message.transform.rotation.z = q.getZ();
    geom_message.transform.rotation.w = q.getW();

    geom_message.header.stamp = ros::Time::now();
    tf_broadcaster.sendTransform(geom_message);
}


}
