#ifndef BROADCASTER_H_
#define BROADCASTER_H_

// ROS

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>

#include <tf/transform_broadcaster.h>


// STL

#include <string>

namespace opti_rviz{

class Broadcaster{

public:

    Broadcaster(const std::string &fixed_frame, const std::string &target_frame_rviz);

    void update(const tf::Vector3& origin, const tf::Matrix3x3& R);

private:

    tf::TransformBroadcaster            tf_broadcaster;
    tf::Quaternion                      q;

    geometry_msgs::TransformStamped     geom_message;

};

}

#endif
