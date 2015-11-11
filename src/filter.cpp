#include "optitrack_rviz/filter.h"
#include <ros/ros.h>

namespace opti_rviz{

/// Kalman filter

Kalman::Kalman(float dt,float process_noise,float measurement_noise){
 /*   dt(dt),kalman_filter(3,process_noise,measurement_noise)
{
    k_position.resize(3);
    k_measurement.resize(3);*/
}

void Kalman::init(tf::Vector3 &origin){
 //   k_position(0) = origin.x();k_position(1) = origin.y(); k_position(2) = origin.z();

  //  kalman_filter.Init(k_position);
}

void Kalman::update(tf::Vector3 &position_measurement){
 //   k_measurement(0) = position_measurement.x();k_measurement(1) = position_measurement.y(); k_measurement(2) = position_measurement.z();
 //   kalman_filter.Update(k_measurement,dt);
 //   kalman_filter.GetPosition(k_position);
 //   position_measurement.setValue(k_position(0),k_position(1),k_position(2));
}

/// Jump filter

Jumps::Jumps(float origin_threashold, float orientation_threashold, bool bDebug):
origin_threashold(origin_threashold),orientation_threashold(orientation_threashold),bFirst(true),bDebug(bDebug)
{

    orientation_buffer.resize(50);
    origin_buffer.resize(50);
}

void Jumps::update(tf::Vector3& origin,tf::Quaternion& orientation){

    if(bFirst){

        origin_buffer.push_back(origin);
        orientation_buffer.push_back(orientation);

        if(origin_buffer.size() == origin_buffer.capacity()){
            bFirst = false;
            ROS_INFO("====== jump filter full ======");
        }

    }else{

        origin_tmp      = origin_buffer[origin_buffer.size()-1];
        orientation_tmp = orientation_buffer[orientation_buffer.size()-1];

        if(bDebug){
            std::cout<< "=== jum debug === " << std::endl;
            std::cout<< "p    : " << origin.x() << "\t" << origin.y() << "\t" << origin.z() << std::endl;
            std::cout<< "p_tmp: " << origin_tmp.x() << "\t" << origin_tmp.y() << "\t" << origin_tmp.z() << std::endl;
            std::cout<< "p_dis: " << origin.distance(origin_tmp) << std::endl;

            std::cout<< "q    : " << orientation.x() << "\t" << orientation.y() << "\t" << orientation.z() <<  "\t" << orientation.w() << std::endl;
            std::cout<< "q_tmp: " << orientation_tmp.x() << "\t" << orientation_tmp.y() << "\t" << orientation_tmp.z() << "\t" << orientation_tmp.w() << std::endl;
            std::cout<< "q_dis: " << dist(orientation,orientation_tmp) << std::endl;
        }

        /// Position jump
        if(jumped(origin,origin_tmp,origin_threashold)){
            ROS_INFO("position jumped !");
            origin = origin_tmp;
           // exit(0);
        }else{
            origin_buffer.push_back(origin);
        }

        /// Orientation jump
        if(jumped(orientation,orientation_tmp,orientation_threashold)){
            ROS_INFO("orientation jumped !");
            orientation = orientation_tmp;
            //exit(0);
        }else{
            orientation_buffer.push_back(orientation);
        }
    }
}

bool Jumps::jumped(const tf::Vector3& p_current,const tf::Vector3& p_previous,const float threashold) const{
    if(p_current.distance(p_previous) < threashold){
        return false;
    }else{
        return true;
    }
}

bool Jumps::jumped(const tf::Quaternion& q_current, const tf::Quaternion& q_previous, const float threashold) const{
    if(dist(q_current,q_previous) < threashold){
        return false;
    }else{
        return true;
    }
}


/// Attractor filter

void Attractors::push_back(const attractor& attractor_in){
    attractors.push_back(attractor_in);
}

void Attractors::update(tf::Quaternion& q_current){

    for(std::size_t i = 0; i < attractors.size();i++){
        update_one(q_current,attractors[i].z_axis,attractors[i].stiff);
    }

}

void Attractors::update_one(tf::Quaternion& q_current,const tf::Vector3& dir_target,float stiff){
    align_quaternion_axis(q_out,q_angle,q_current,dir_target);
    lik = std::exp(-stiff*(1-q_angle)*(1-q_angle));
    if(lik < 0.5){
         lik = 1.0;
    }else{
         lik = 0.0;
    }
    q_current = q_out.slerp(q_current,lik);
}

void Attractors::align_quaternion_axis(tf::Quaternion& q_out, float &angle, const tf::Quaternion &q_in, tf::Vector3 target){
    R.setRotation(q_in);
    tf::Vector3 z_axis(R[0][2],R[1][2],R[2][2]);

    z_axis          = z_axis.normalize();

    tf::Vector3 c   = z_axis.cross(target);
    angle           = z_axis.dot(target);

    tf::Quaternion q_r;
    q_r.setX(c.x());
    q_r.setY(c.y());
    q_r.setZ(c.z());
    q_r.setW(sqrt(z_axis.length2() * target.length2()) + angle);

    q_out = q_r*q_in;
}


/// Filter

Quaternion_filter::Quaternion_filter(float t_){
    t = t_;

    if(t < 0){
        t = 0;
        ROS_INFO("t must be in range [0,1]");
    }
    if(t > 1){
        t = 1;
        ROS_INFO("t must be in range [0,1]");
    }

    q_tmp.setEuler(0,0,0);

}

void Quaternion_filter::update(tf::Quaternion& orientation){

    orientation = q_tmp.slerp(orientation,t);
    q_tmp = orientation;

}


}
