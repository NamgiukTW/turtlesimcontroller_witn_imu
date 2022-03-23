#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"

float front=0;
float side=0;
// float rotate=0;

void msgCallback(const sensor_msgs::Imu& msg)
{
    ROS_INFO("linear.x=%f",msg.orientation.x);
    ROS_INFO("linear.y=%f",msg.orientation.y);
    ROS_INFO("linear.z=%f",msg.orientation.z);
    ROS_INFO("angular.x=%f",msg.angular_velocity.x);
    ROS_INFO("angular.y=%f",msg.angular_velocity.y);
    ROS_INFO("angular.z=%f",msg.angular_velocity.z);

    front = -msg.orientation.y;
    side = msg.orientation.x;
    // rotate = msg.angular_velocity.z;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlesimcontroller_witn_imu");
    ros::NodeHandle nh;

    ros::Publisher imu_controller = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",100);

    ros::Subscriber turtlesim_processer = nh.subscribe("imu/data", 100, msgCallback);
    
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        geometry_msgs::Twist msg;
        msg.linear.x=front;
        msg.linear.y=side;
        msg.linear.z=0.0;
        msg.angular.x=0.0;
        msg.angular.y=0.0;
        msg.angular.z=0.0;

        imu_controller.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;

}