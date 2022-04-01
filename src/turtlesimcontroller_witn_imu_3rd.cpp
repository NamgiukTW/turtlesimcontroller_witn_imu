#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

#include <mutex>

//test
#include <ros/callback_queue.h>
float gFront=0;
float gRotate=0;

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

    // now = imuMsg.header.seq;
    // pass = imuMsg.header.seq;
    // ms = (now-pass) / 100;

    frontLocal1 = atan(-imuMsg.linear_acceleration.x / sqrt(pow(imuMsg.linear_acceleration.y, 2) + pow(imuMsg.linear_acceleration.z, 2)));
    frontLocal1 = frontLocal1 * 1.5;
    rotateLocal1 = atan(-imuMsg.linear_acceleration.y / sqrt(pow(imuMsg.linear_acceleration.x, 2) + pow(imuMsg.linear_acceleration.z, 2)));
    
    if(frontLocal1 > 1.5 || frontLocal1 < -1.5)
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
    float x1;
    float y1;
    float theta1;
    float frontLocal2;
    float rotateLocal2;
    // gX, gY, gTheta 전역 변수와 timeCallBack이 공유하므로 mutex 적용하기 위해 지역 변수 선언
    // gFront, gRotate 전역변수와 imuDataCallback, timeCallBack이 공유하므로 마찬가지로 지역변수 선언


    gMut.lock();
    x1 = gX;
    y1 = gY;
    theta1 = gTheta;
    frontLocal2 = gFront;
    rotateLocal2 = gRotate;
    gMut.unlock();

    x1 = turtlePoseMsg->x;
    y1 = turtlePoseMsg->y;
    theta1 = turtlePoseMsg->theta;
    frontLocal2 = turtlePoseMsg->linear_velocity;
    rotateLocal2 = turtlePoseMsg->angular_velocity;

    gMut.lock();
    gX = x1;
    gY = y1;
    gTheta = theta1;
    gFront = frontLocal2;
    gRotate = rotateLocal2;
    gMut.unlock();
   
    // poseDataCallback에서 볼일이 끝났으므로 지역변수 -> 전역변수 돌릴 시 mutex 적용

}

void timeCallBack(const ros::TimerEvent&){
    
    float x2;
    float y2;
    float theta2;
    float frontLocal3;
    float rotateLocal3;

    gMut.lock();
    x2 = gX;
    y2 = gY;
    theta2 = gTheta;
    frontLocal3 = gFront;
    rotateLocal3 = gRotate;
    gMut.unlock();
    // gX, gY, gTheta 전역 변수와 timeCallBack이 공유하므로 mutex 적용하기 위해 지역 변수 선언
    // gFront, gRotate 전역변수와 imuDataCallback, timeCallBack이 공유하므로 마찬가지로 지역변수 선언
    // poseDataCallback 콜백 함수에서 적용한 결과를 1초 단위로 출력하기 위해 timeCallBack 콜백함수 사용

    std::cout << "X_posithon = " << x2 << std::endl;
    std::cout << "Y_posithon = " << y2 << std::endl;
    std::cout << "theta_Position = " << theta2 << std::endl;
    std::cout << "linear_Velocity = " << frontLocal3 << std::endl;
    std::cout << "angular_Velocity = " << rotateLocal3 << std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlesimcontroller_witn_imu_3rd");

    ros::NodeHandle nh;
    ros::Publisher imuData = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",100);
    pVeloCommandPublisher = &imuData;
    // ros::Rate loopRate(10);

    ros::CallbackQueue myCallbackQueue; //test
    ros::AsyncSpinner spinner(0, &myCallbackQueue); //test
    nh.setCallbackQueue(&myCallbackQueue); //test

    ros::Subscriber turtlesimControl = nh.subscribe("/imu/data", 100, imuDataCallback);
    ros::Subscriber turtlesimPoseSub = nh.subscribe("/turtle1/pose", 100, poseDataCallback);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), timeCallBack);
   
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
