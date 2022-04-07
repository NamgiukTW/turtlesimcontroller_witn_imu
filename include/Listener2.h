#pragma once

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

#include <iostream>
#include <mutex>
// test
#include <ros/callback_queue.h>


using namespace std;

class Listener
{
    private:
        float m_Front;
        float m_Rotate;
        float m_X;
        float m_Y;
        float m_Theta;
 
        std::mutex m_Mut;

        // ros::Publisher* m_VeloCommandPublisher;
        ros::Publisher m_ImuData;

        ros::NodeHandle m_Nh;

        ros::CallbackQueue m_MyCallbackQueue;
        ros::AsyncSpinner m_Spinner;

        ros::Subscriber m_TurtlesimControl;
        ros::Subscriber m_TurtlesimPoseSub;

        ros::Timer m_Timer;

    public:
        void imuDataCallback(const sensor_msgs::Imu &imuMsg);
        void poseDataCallback(const turtlesim::PoseConstPtr &turtlePoseMsg);
        void timeCallBack(const ros::TimerEvent &);
        Listener(void);
        ~Listener(void);
};