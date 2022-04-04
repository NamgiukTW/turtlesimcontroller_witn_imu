#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

#include <mutex>
// #include <thread>
// #include <iostream>

// #include <algorithm>
// #include <ctime>


//test
#include <ros/callback_queue.h>
float gFront=0;
float gRotate=0;
// float side=0;

// unsigned long now = 0;
// unsigned long pass = 0;
// double ms=0;

float gX = 0;
float gY = 0;
float gTheta = 0;

std::mutex gMut;
ros::Publisher* pVeloCommandPublisher;

void imuDataCallback(const sensor_msgs::Imu& imuMsg)
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
    
    if(frontLocal1 > 1.5 || frontLocal1 < -1.5) // 잘못된 변수 사용
        frontLocal1 = 0;

    if(rotateLocal1 > 1.2 || rotateLocal1 < -1.2)
        rotateLocal1 = 0;
   
    // ROS_INFO("imu data subscribed...");

    msg.linear.x=frontLocal1;
    msg.angular.z=rotateLocal1;


    // make message
    pVeloCommandPublisher->publish(msg);


    gMut.lock();
    gFront = frontLocal1;
    gRotate = rotateLocal1;
    gMut.unlock();

    // 70~73 라인 poseDataCallback 스레드와 변수 데이터를 공유해야 하므로 mutex 필요

    //  publish... (publisher가 본 함수 내에서 선언되면 동작 안해...)
   
}


void poseDataCallback(const turtlesim::PoseConstPtr& turtlePoseMsg)
{
    // std::lock_guard<std::mutex> lock(gMut);

    // float xLocal;
    // float yLocal;
    // float thetaLocal;

    // gMut.lock();
    // xLocal = gX;
    // yLocal = gY;
    // thetaLocal = gTheta;
    // gMut.unlock();

    float frontLocal2;
    float rotateLocal2;


    gMut.lock();
    frontLocal2 = gFront;
    rotateLocal2 = gRotate;
    gMut.unlock();

    // 100~103 라인 imuDataCallback 스레드와 변수 데이터를 공유해야 하므로 mutex 필요

    gX = turtlePoseMsg->x;
    gY = turtlePoseMsg->y;
    gTheta = turtlePoseMsg->theta;
    frontLocal2 = turtlePoseMsg->linear_velocity;
    rotateLocal2 = turtlePoseMsg->angular_velocity;

    // ROS_INFO("X_posithon = %f ", gX);
    // ROS_INFO("Y_Posetion = %f ", gY);
    // ROS_INFO("theta_Position = %f ", gTheta);
    // ROS_INFO("imu data subscribed?");
    std::cout << "X_posithon = " << gX << std::endl;
    std::cout << "Y_posithon = " << gY << std::endl;
    std::cout << "theta_Position = " << gTheta << std::endl;
    std::cout << "linear_Velocity = " << frontLocal2 << std::endl;
    std::cout << "angular_Velocity = " << rotateLocal2 << std::endl;
    sleep(1);

    // 139~142 라인 공유하는 스레드 없으므로 mutex 필요 없음

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlesimcontroller_witn_imu_2nd");

    ros::NodeHandle nh;
    ros::Publisher imuData = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",100);
    pVeloCommandPublisher = &imuData;
    // ros::Rate loopRate(10);

    ros::CallbackQueue myCallbackQueue; //test
    ros::AsyncSpinner spinner(0, &myCallbackQueue); //test
    nh.setCallbackQueue(&myCallbackQueue); //test

    ros::Subscriber turtlesimControl = nh.subscribe("/imu/data", 100, imuDataCallback);
    ros::Subscriber turtlesimPoseSub = nh.subscribe("/turtle1/pose", 100, poseDataCallback);
   
    // ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

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
