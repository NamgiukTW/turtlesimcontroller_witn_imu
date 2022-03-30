#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

#include <mutex>
#include <thread>
#include <iostream>

#include <algorithm>
#include <ctime>


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
    float front_Local1;
    float rotate_Local1;
    geometry_msgs::Twist msg;

    gMut.lock();
    front_Local1 = gFront;
    rotate_Local1 = gRotate;
    gMut.unlock();

    // gMut.lock();
    // front_Local = gFront;
    // rotate_Local = gRotate;
    // gMut.unlock();

    // now = imuMsg.header.seq;
    // pass = imuMsg.header.seq;
    // ms = (now-pass) / 100;

    // Front2 = msg.angular_velocity.y * ms;
    // Rotate2 = imuMsg.angular_velocity.x;

    // std::lock_guard<std::mutex> lock(gMut);


    front_Local1 = atan(-imuMsg.linear_acceleration.x / sqrt(pow(imuMsg.linear_acceleration.y, 2) + pow(imuMsg.linear_acceleration.z, 2)));
    front_Local1 = front_Local1 * 1.5;
    rotate_Local1 = atan(-imuMsg.linear_acceleration.y / sqrt(pow(imuMsg.linear_acceleration.x, 2) + pow(imuMsg.linear_acceleration.z, 2)));
    
    if(gFront > 1.5 || gFront < -1.5)
        front_Local1 = 0;

    if(gRotate > 1.2 || gRotate < -1.2)
        rotate_Local1 = 0;
   
    // ROS_INFO("imu data subscribed...");

    msg.linear.x=front_Local1;
    msg.angular.z=rotate_Local1;


    // make message
    pVeloCommandPublisher->publish(msg);


    gMut.lock();
    gFront = front_Local1;
    gRotate = rotate_Local1;
    gMut.unlock();

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

    float front_Local2;
    float rotate_Local2;


    gMut.lock();
    front_Local2 = gFront;
    rotate_Local2 = gRotate;
    gMut.unlock();



    gX = turtlePoseMsg->x;
    gY = turtlePoseMsg->y;
    gTheta = turtlePoseMsg->theta;
    front_Local2 = turtlePoseMsg->linear_velocity;
    rotate_Local2 = turtlePoseMsg->angular_velocity;

    // ROS_INFO("X_posithon = %f ", gX);
    // ROS_INFO("Y_Posetion = %f ", gY);
    // ROS_INFO("theta_Position = %f ", gTheta);
    // ROS_INFO("imu data subscribed?");
    std::cout << "X_posithon = " << gX << std::endl;
    std::cout << "Y_posithon = " <<gY << std::endl;
    std::cout << "theta_Position = " << gTheta << std::endl;
    std::cout << "linear_Velocity = " << front_Local2 << std::endl;
    std::cout << "angular_Velocity = " << rotate_Local2 << std::endl;
    sleep(1);

    // ros::Duration(1.0).sleep();
    // sleep(1);
    // gMut.lock();
    // gX = xLocal;
    // gY = yLocal;
    // gTheta = thetaLocal;
    // gMut.unlock();

    gMut.lock();
    gFront = front_Local2;
    gRotate = rotate_Local2;
    gMut.unlock();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlesimcontroller_witn_imu_2nd");

    ros::NodeHandle nh;
    ros::Publisher imuData = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",100);
    pVeloCommandPublisher = &imuData;
    // ros::Rate loopRate(10);

    ros::CallbackQueue my_callback_queue; //test
    ros::AsyncSpinner spinner(0, &my_callback_queue); //test
    nh.setCallbackQueue(&my_callback_queue); //test

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
