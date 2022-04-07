// 2nd 에 사용한 sleep를 버리고 ROS time 기능을 이용해 출력 딜레이 적용
// time을 사용하기 위한 callback함수를 새로 만든다.

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"

#include <mutex>

// test
#include <ros/callback_queue.h>
float gFront = 0;
float gRotate = 0;
float gX = 0;
float gY = 0;
float gTheta = 0;
// 해당 전역변수는 각 스레드가 데이터 공유

std::mutex gMut;

ros::Publisher *pVeloCommandPublisher;

void imuDataCallback(const sensor_msgs::Imu &imuMsg)
{
    float frontLocal1;
    float rotateLocal1;
    geometry_msgs::Twist msg;

    frontLocal1 = atan(-imuMsg.linear_acceleration.x / sqrt(pow(imuMsg.linear_acceleration.y, 2) + pow(imuMsg.linear_acceleration.z, 2)));
    frontLocal1 = frontLocal1 * 1.5;
    rotateLocal1 = atan(-imuMsg.linear_acceleration.y / sqrt(pow(imuMsg.linear_acceleration.x, 2) + pow(imuMsg.linear_acceleration.z, 2)));

    if (frontLocal1 > 1.5 || frontLocal1 < -1.5)
        frontLocal1 = 0;

    if (rotateLocal1 > 1.2 || rotateLocal1 < -1.2)
        rotateLocal1 = 0;

    // ROS_INFO("imu data subscribed...");

    msg.linear.x = frontLocal1;
    msg.angular.z = rotateLocal1;

    // make message
    pVeloCommandPublisher->publish(msg);

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
    float x1;
    float y1;
    float theta1;
    // gX, gY, gTheta 전역 변수와 timeCallBack이 공유하므로 mutex 적용하기 위해 지역 변수 선언
    // gFront, gRotate 전역변수와 imuDataCallback, timeCallBack이 공유하므로 마찬가지로 지역변수 선언

    x1 = turtlePoseMsg->x;
    y1 = turtlePoseMsg->y;
    theta1 = turtlePoseMsg->theta;

    {
        std::lock_guard<std::mutex> lock(gMut);
        gX = x1;
        gY = y1;
        gTheta = theta1;
    }

    // poseDataCallback에서 볼일이 끝났으므로 지역변수 -> 전역변수 돌릴 시 mutex 적용

    // 2nd 노드와 달리 해당 스레드에서 출력하지 않고 전역변수에 좌표값만 선언함.
}

void timeCallBack(const ros::TimerEvent &)
{

    float x2;
    float y2;
    float theta2;
    float frontLocal2;
    float rotateLocal2;

    {
        std::lock_guard<std::mutex> lock(gMut);
        x2 = gX;
        y2 = gY;
        theta2 = gTheta;
        frontLocal2 = gFront;
        rotateLocal2 = gRotate;
    }
    // gX, gY, gTheta 전역 변수와 poseDataCallback이 공유하므로 mutex 적용하기 위해 지역 변수 선언
    // gFront, gRotate 전역변수와 imuDataCallback이 공유하므로 마찬가지로 지역변수 선언
    // poseDataCallback, imuDataCallback 콜백 함수에서 적용한 결과를 1초 단위로 출력하기 위해 timeCallBack 콜백함수 사용

    std::cout << "X_posithon = " << x2 << std::endl;
    std::cout << "Y_posithon = " << y2 << std::endl;
    std::cout << "theta_Position = " << theta2 << std::endl;
    std::cout << "Imu_Front = " << frontLocal2 << std::endl;
    std::cout << "Imu_Rotate = " << rotateLocal2 << std::endl;

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlesimcontroller_witn_imu_3rd");


    ros::NodeHandle nh;
    ros::Publisher imuData = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
    pVeloCommandPublisher = &imuData;
    // ros::Rate loopRate(10);

    ros::CallbackQueue myCallbackQueue;             // test
    ros::AsyncSpinner spinner(0, &myCallbackQueue); // test
    nh.setCallbackQueue(&myCallbackQueue);          // test

    ros::Subscriber turtlesimControl = nh.subscribe("/imu/data", 100, imuDataCallback);
    ros::Subscriber turtlesimPoseSub = nh.subscribe("/turtle1/pose", 100, poseDataCallback);
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), timeCallBack); // time 콜백함수를 사용하기 위해 적용. 출력이 1초간 딜레이됨

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
