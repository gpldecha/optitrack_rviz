#include "optitrack_rviz/listener.h"


namespace opti_rviz{

Listener::Listener(const std::string &fixed_frame, const std::string &target_frame):
    fixed_frame(fixed_frame),
    target_frame(target_frame),
    bFirst(true)
{
    q.setEuler(0,0,0);
    q_tmp = q;
    R1.setRPY(M_PI/2,0,0);

}

void Listener::update_opti2rviz(tf::Vector3& origin,tf::Matrix3x3& orientation){

    try{
        tf_listener.lookupTransform(fixed_frame,target_frame, ros::Time(0), tf_transform);
        origin.setValue(tf_transform.getOrigin().getX(),-tf_transform.getOrigin().getZ(),tf_transform.getOrigin().getY());
        q    =   tf_transform.getRotation();
    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::Duration(0.2).sleep();
        q      = q_tmp;
        origin = origin_tmp;
    }



    optitrack_to_rviz(q);
    orientation.setRotation(q);

    origin_tmp = origin;
    q_tmp       = q;
}

bool Listener::update(tf::Vector3& origin,tf::Matrix3x3& orientation){
    bReceivedMessage = true;
    try{
        tf_listener.lookupTransform(fixed_frame,target_frame, ros::Time(0), tf_transform);
        origin = tf_transform.getOrigin();
        orientation.setRotation(tf_transform.getRotation());
    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::Duration(0.2).sleep();
        q      = q_tmp;
        origin = origin_tmp;
        bReceivedMessage = false;

    }

    origin_tmp = origin;
    q_tmp       = q;
    return bReceivedMessage;
}

void Listener::update(tf::Vector3& origin,tf::Quaternion& orientation){
    try{
        tf_listener.lookupTransform(fixed_frame,target_frame, ros::Time(0), tf_transform);
        origin      = tf_transform.getOrigin();
        orientation = tf_transform.getRotation();
    }
    catch (tf::TransformException ex){
        ROS_WARN("%s",ex.what());
        ros::Duration(0.2).sleep();
        q      = q_tmp;
        origin = origin_tmp;
    }

    origin_tmp = origin;
    q_tmp       = q;
}

void get_tf_once(const std::string& fixed_frame, const std::string& target_frame,tf::Transform& transform){

}


void Listener::optitrack_to_rviz(tf::Quaternion& q) {

    tmp.setRotation(q);

    tmp2[0][0]  = tmp[0][0];
    tmp2[1][0]  = tmp[1][0];
    tmp2[2][0]  = tmp[2][0];

    tmp2[0][1]  = -tmp[0][2];
    tmp2[1][1]  = -tmp[1][2];
    tmp2[2][1]  = -tmp[2][2];

    tmp2[0][2]  = tmp[0][1];
    tmp2[1][2]  = tmp[1][1];
    tmp2[2][2]  = tmp[2][1];

    tmp2 = R1 * tmp2;

    tmp2.getRotation(q);
}


void Listener::get_tf_once(const std::string& target_frame, const std::string& source_frame, tf::StampedTransform& transform){

    tf::TransformListener listener;
    bool bset = false;
    while(bset != true){
        try{
            listener.waitForTransform(target_frame, source_frame, ros::Time(0), ros::Duration(2.0));
            listener.lookupTransform(target_frame,source_frame, ros::Time(0), transform);
            bset = true;
        }catch (tf::TransformException ex){
            ROS_WARN_THROTTLE(1,"%s",ex.what());
        }
    }

}

void Listener::print(const tf::StampedTransform& transfrom){
    std::cout<< "tf origin: " << transfrom.getOrigin().x() << " " << transfrom.getOrigin().y() << " " << transfrom.getOrigin().z() << std::endl;
    std::cout<< "tf orient: " << transfrom.getRotation().getW() << " "
             <<  transfrom.getRotation().getX() << " "
              <<  transfrom.getRotation().getY() << " "
               <<  transfrom.getRotation().getZ() << std::endl;

}


}

