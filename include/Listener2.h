// 의존하는 헤더, 변수 선언 등 전부 헤더파일에 작성

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

        // 기존에 int main에서 선언했던 변수들을 이곳에 모았다.
        void imuDataCallback(const sensor_msgs::Imu &imuMsg);
        void poseDataCallback(const turtlesim::PoseConstPtr &turtlePoseMsg);
        void timeCallBack(const ros::TimerEvent &);

    public:
        Listener(void); // 생성자
        ~Listener(void); // 소멸자
        // 생성자는 객체 생성과 동시에 맴버 변수를 초기화 할때 사용.
        // 이것으로 기존 노드의 int main 에 선언된 퍼블리셔와 서브스크라이버를 구동할 수 있다.
        // 소멸자는 객체의 수명이 끝났을 때 객체 제거를 위해 사용하지만 해당 노드 구현에 필요없다.
        // 보통 생성자와 소멸자 같이 선언하고 생성자는 클래스명과 같은 이름으로 선언한다.
        // Listener();로 선언해도 되며 ()에 void가 자동 선언된다고 생각하면 된다.
};