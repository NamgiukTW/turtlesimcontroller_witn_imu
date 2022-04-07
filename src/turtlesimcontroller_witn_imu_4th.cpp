#include "Listener2.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "turtlesimcontroller_witn_imu_4th");

    Listener listener1;

    // ros::NodeHandle nh;
    // ros::Publisher imuData = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",100);
    // pVeloCommandPublisher = &imuData;
    // ros::Rate loopRate(10);

    // ros::CallbackQueue myCallbackQueue; //test
    // ros::AsyncSpinner spinner(0, &myCallbackQueue); //test
    // nh.setCallbackQueue(&myCallbackQueue); //test


    // ros::Subscriber turtlesimControl = nh.subscribe("/imu/data", 100, &Listener::imuDataCallback, &listener);
    // ros::Subscriber turtlesimPoseSub = nh.subscribe("/turtle1/pose", 100, &Listener::poseDataCallback, &listener);
    // ros::Timer timer = nh.createTimer(ros::Duration(1.0), &Listener::timeCallBack, &listener);

    // spinner.start();
    ros::waitForShutdown();
    return 0;
}
