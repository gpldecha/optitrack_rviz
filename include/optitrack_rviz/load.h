#ifndef OPTITRACK_RVIZ_LOAD_H_
#define OPTITRACK_RVIZ_LOAD_H_

// ROS

#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Quaternion.h>

// STL

#include <string>

namespace opti_rviz{

class Load{

public:

   static bool load(tf::Vector3& origin, tf::Quaternion& q, std::string path_to_load);

};

}

#endif
