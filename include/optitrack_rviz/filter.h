#ifndef FILTER_H_
#define FILTER_H_


#include <tf/LinearMath/Vector3.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Quaternion.h>



//#include <joint_state_estimator.h>

#include <boost/circular_buffer.hpp>


namespace opti_rviz{

class Kalman{

public:

    Kalman(float dt,float process_noise,float measurement_noise);

    void init(tf::Vector3& origin);

    void update(tf::Vector3& position_measurement);

private:

  //  kkf::JointStateEstimator<float>                     kalman_filter;
  //  kkf::TVec<float>                                    k_measurement,k_position;
    float                                               dt;

};

class Jumps{
public:

    Jumps(float origin_threashold,float orientation_threashold,bool bDebug=false);

    void update(tf::Vector3& origin, tf::Quaternion& orientation);

private:

    bool jumped(const tf::Vector3& p_current,const tf::Vector3& p_previous,const float threashold) const;

    bool jumped(const tf::Quaternion& q_current, const tf::Quaternion& q_previous, const float threashold) const;

    inline float norm (const tf::Quaternion& q) const{
        return sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z() + q.w()*q.w());
    }

    inline float dist(const tf::Quaternion& q1, const tf::Quaternion& q2) const{
        return norm(q1 - q2);
    }

private:

    tf::Vector3                                 origin_tmp;
    tf::Quaternion                              orientation_tmp;
    boost::circular_buffer<tf::Quaternion>      orientation_buffer;
    boost::circular_buffer<tf::Vector3>         origin_buffer;
    bool                                        bFirst;
    bool                                        bDebug;
    float                                       origin_threashold;
    float                                       orientation_threashold;


};

class Attractors{

public:

    typedef struct{
        float stiff;
        tf::Vector3 z_axis;
    } attractor;

public:

    void push_back(const attractor& attractor_in);

    void update(tf::Quaternion& q_current);

private:

    void update_one(tf::Quaternion& q_current,const tf::Vector3& dir_target,float stiff);

private:

    void align_quaternion_axis(tf::Quaternion& q_out, float &angle, const tf::Quaternion &q_in, tf::Vector3 target);

private:

    tf::Matrix3x3          R;
    tf::Quaternion         q_out;
    float                  lik;
    float                  q_angle;
    std::vector<attractor> attractors;

};

class Quaternion_filter{

public:

    Quaternion_filter(float t_ = 0.5);

    void update(tf::Quaternion& orientation);


private:

    tf::Quaternion  q_tmp;
    float           t;


};

}


#endif
