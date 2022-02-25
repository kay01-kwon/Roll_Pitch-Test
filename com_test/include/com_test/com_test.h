#ifndef COM_TEST_H
#define COM_TEST_H

#include <ros/ros.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

using std_msgs::Float32;
using sensor_msgs::Imu;

using std::cout;
using std::endl;


typedef float flt;
typedef Eigen::Quaternion<flt> quat;
typedef Eigen::Matrix<flt,3,1> quat_vec;
typedef Eigen::Matrix<flt,3,3> rotm;
typedef Eigen::Matrix<flt,3,3> eyem;
typedef Eigen::Matrix<flt,3,3> skiewm;

ros::Time currentTime;

class COM_TEST{

    public:
    // Constructor
    COM_TEST();
    void ImuCallback(const Imu::ConstPtr & imu_msg);
    void quat2rotm(quat &q);
    void quat_vec2_skiewm(quat_vec &q_vec);

    // Destructor
    ~COM_TEST();
    private:
    ros::NodeHandle nh;
    ros::Subscriber imu_quat_subscriber;
    ros::Publisher com_publisher;
    
    quat I_q_B;
    quat_vec q_vec;
    rotm I_R_B;
    eyem Eye;
    skiewm Skiewm;

};

// Constructor
COM_TEST::COM_TEST()
{
    imu_quat_subscriber = nh.subscribe("/mavros/imu/data",1,&COM_TEST::ImuCallback,this);
    
    I_q_B.w() = 1;
    I_q_B.x() = 0;
    I_q_B.y() = 0;
    I_q_B.z() = 0;
    
    I_R_B.Identity();
    Eye<<1,0,0,
        0,1,0,
        0,0,1;
    q_vec.Zero();
}

// Callback
void COM_TEST::ImuCallback(const Imu::ConstPtr & imu_msg)
{
    // Data from Imu orientation message
    I_q_B.w() = imu_msg->orientation.w;
    I_q_B.x() = imu_msg->orientation.x;
    I_q_B.y() = imu_msg->orientation.y;
    I_q_B.z() = imu_msg->orientation.z;

    
    quat2rotm(I_q_B);
}

// Quat to Rotm
void COM_TEST::quat2rotm(quat &q)
{
    q_vec << q.x(), q.y(), q.z();
    quat_vec2_skiewm(q_vec);

    I_R_B = q.w()*q.w()*Eye + 2*q.w()*Skiewm + 2*q_vec*q_vec.transpose() - q_vec.transpose()*q_vec*Eye;
    float roll, pitch;

    roll = asin(I_R_B(2,1));
    pitch = atan2(-I_R_B(2,0)/cos(roll),I_R_B(2,2)/cos(roll));

    cout<<"roll: "<<roll*180.0/3.141592<<endl;
    cout<<"pitch: "<<pitch*180.0/3.141592<<endl;
    cout<<"\n";

}

void COM_TEST::quat_vec2_skiewm(quat_vec &q_vec)
{
    Skiewm <<0, -q_vec(2), q_vec(1),
            q_vec(2), 0 , -q_vec(0),
            -q_vec(1), q_vec(0), 0;

}

COM_TEST::~COM_TEST()
{

}
#endif