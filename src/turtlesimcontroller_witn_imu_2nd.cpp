// 1st 노드에 멀티스레드 옵션 적용
// Imu 를 이용한 turtlesim 제어와 제어된 turtlesim의 위치값과 속도값 출력이 같이 되도록 진행
// 각 스레드 끼리 공유하는 데이터는 mutex 적용

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h" // turtle의 위치값 메시지 출력을 위해 헤더 선언

#include <mutex> //mutex 사용을 위해 선언
// #include <thread>
// #include <iostream>

// #include <algorithm>
// #include <ctime>

// test
#include <ros/callback_queue.h> //callbackqueue 사용을 위해 헤더 선언
float gFront = 0;
float gRotate = 0;
// 해당 전역변수는 각 스레드가 데이터 공유


// unsigned long now = 0;
// unsigned long pass = 0;
// double ms=0;

std::mutex gMut; // mutex의 원활한 사용을 위한 변수 선언
ros::Publisher *pVeloCommandPublisher; // 콜백 함수 스레드 안에서 퍼블리셔를 구동하게 하기 위해 선언

void imuDataCallback(const sensor_msgs::Imu &imuMsg)
{
    float frontLocal1;
    float rotateLocal1;
    geometry_msgs::Twist msg;

    // 39~42 라인 계산 값을 공유하지 않기에 불필요

    // now = imuMsg.header.seq;
    // pass = imuMsg.header.seq;
    // ms = (now-pass) / 100;

    // Front2 = msg.angular_velocity.y * ms;
    // Rotate2 = imuMsg.angular_velocity.x;

    // std::lock_guard<std::mutex> lock(gMut);

    frontLocal1 = atan(-imuMsg.linear_acceleration.x / sqrt(pow(imuMsg.linear_acceleration.y, 2) + pow(imuMsg.linear_acceleration.z, 2)));
    frontLocal1 = frontLocal1 * 1.5;
    rotateLocal1 = atan(-imuMsg.linear_acceleration.y / sqrt(pow(imuMsg.linear_acceleration.x, 2) + pow(imuMsg.linear_acceleration.z, 2)));

    if (frontLocal1 > 1.5 || frontLocal1 < -1.5) // 잘못된 변수 사용
        frontLocal1 = 0;

    if (rotateLocal1 > 1.2 || rotateLocal1 < -1.2)
        rotateLocal1 = 0;

    // ROS_INFO("imu data subscribed...");

    msg.linear.x = frontLocal1;
    msg.angular.z = rotateLocal1;

    // make message
    pVeloCommandPublisher->publish(msg); // 퍼블리셔 메시지 생성

    {
        std::lock_guard<std::mutex> lock(gMut);
        gFront = frontLocal1;
        gRotate = rotateLocal1;
    }
    // 70~73 라인 poseDataCallback 스레드와 변수 데이터를 공유해야 하므로 mutex 필요

    //  publish... (publisher가 본 함수 내에서 선언되면 동작 안해...)
}

void poseDataCallback(const turtlesim::PoseConstPtr &turtlePoseMsg)
{
    float frontLocal2;
    float rotateLocal2;
    float x1 = 0;
    float y1 = 0;
    float theta1 = 0;

    {
        std::lock_guard<std::mutex> lock(gMut);
        frontLocal2 = gFront;
        rotateLocal2 = gRotate;
    }
    // 100~103 라인 imuDataCallback 스레드와 변수 데이터를 공유해야 하므로 mutex 필요

    x1 = turtlePoseMsg->x;
    y1 = turtlePoseMsg->y;
    theta1 = turtlePoseMsg->theta;

    // ROS_INFO("X_posithon = %f ", x1);
    // ROS_INFO("Y_Posetion = %f ", y1);
    // ROS_INFO("theta_Position = %f ", theta1);
    // ROS_INFO("imu data subscribed?");
    std::cout << "X_Posithon = " << x1 << std::endl;
    std::cout << "Y_Posithon = " << y1 << std::endl;
    std::cout << "theta_Position = " << theta1 << std::endl;

    std::cout << "Imu_Front = " << frontLocal2 << std::endl;
    std::cout << "Imu_Rotate = " << rotateLocal2 << std::endl;
    sleep(1); // 출력을 1초 딜레이

    // 139~142 라인 공유하는 스레드 없으므로 mutex 필요 없음
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlesimcontroller_witn_imu_2nd");

    ros::NodeHandle nh;
    ros::Publisher imuData = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
    pVeloCommandPublisher = &imuData;
    // ros::Rate loopRate(10);

    // CallbackQueue 함수와 AsyncSpinner 함수는 같이 사용하면 좋다.
    // AsyncSpinner 에 변수를 선언해 초기화하고 spinner.start();로 구동 시작


    ros::CallbackQueue myCallbackQueue;             // test
    ros::AsyncSpinner spinner(0, &myCallbackQueue); // test
    nh.setCallbackQueue(&myCallbackQueue);          // test

    ros::Subscriber turtlesimControl = nh.subscribe("/imu/data", 100, imuDataCallback);
    ros::Subscriber turtlesimPoseSub = nh.subscribe("/turtle1/pose", 100, poseDataCallback);

    // 모든 콜백함수를 서브스크라이빙 하기 위해 각각 서브스크라이버 설정

    spinner.start();
    ros::waitForShutdown();

    // ros::waitForShutdown();으로 직접 종료 전 까지 프로그램을 끝내지 않는 옵션 지정.

    // while(ros::ok()){
    //     geometry_msgs::Twist msg;

    //     msg.linear.x=gFront;
    //     // ROS_INFO("pitch=%f",gFront);

    //     msg.angular.z=gRotate;
    //     // ROS_INFO("roll=%f",gRotate);

    //     // msg.angular.z=Rotate2;
    //     // ROS_INFO("Rotate=%f",Rotate2);

    //     imuData.publish(msg);
    //     loopRate.sleep();
    // }
    return 0;
}
