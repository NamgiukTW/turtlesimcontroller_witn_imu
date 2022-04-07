// Imu 의 Roll, Pitch 값으로 turtlesim 을 제어할 것.
// Roll 값으로 turtlesim 방향 제어(Rotate)
// Pitch 값으로 turtlesim 이동 및 속도제어(Front)

#include "ros/ros.h" // ROS 기본 헤더
#include "sensor_msgs/Imu.h" // 퍼블리시할 메시지 헤더
#include "geometry_msgs/Twist.h" //서브스크라이버할 메시지 헤더

float front=0;
// float side=0;
float rotate=0;

// unsigned long now = 0;
// unsigned long pass = 0;
// double ms=0;

void msgCallback(const sensor_msgs::Imu& msg2)
{
    ROS_INFO("pitch.y=%f",front);
    ROS_INFO("roll.x=%f",rotate);
    // ROS_INFO("roll.x=%f",rotate2);
    // ROS_INFO("speed=%f",ms);
    // ROS_INFO("hello?");
    // sleep(1);
    // conversion

    // quaternion to euler 

    // front = pitch (radian) ==> m/s  (mapping: -1m/s ~ 1m/s 선형적으로 변환)
    // rotate = roll (radian) ==> rad/s (mapping: -1 rad/s ~ 1rad/s 선형적으로 변환 )

    // now = msg2.header.seq;
    // pass = msg2.header.seq;
    // ms = (now-pass) / 100;

    // front2 = msg.angular_velocity.y * ms;
    // rotate2 = msg2.angular_velocity.x;

    front = atan(-msg2.linear_acceleration.x / sqrt(pow(msg2.linear_acceleration.y, 2) + pow(msg2.linear_acceleration.z, 2)));
    front = front * 1.5; // 거북이 속도를 pitch 기울기로 나타내는 값 보다 1.5배 빠르게
    rotate = atan(-msg2.linear_acceleration.y / sqrt(pow(msg2.linear_acceleration.x, 2) + pow(msg2.linear_acceleration.z, 2)));

    
    if(front > 1.5 || front < -1.5)
        front = 0;

    if(rotate > 1.2 || rotate < -1.2)
        rotate = 0;

    
    
    // side = msg2.orientation.y;
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtlesimcontroller_witn_imu_1st");
    ros::NodeHandle nh;

    ros::Publisher imu_controller = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",100);

    ros::Subscriber turtlesim_processer = nh.subscribe("/imu/data", 100, msgCallback);
    
    ros::Rate loop_rate(10);
    
    //ros::spinOnce(); 사용시 while(ros::ok()) 사용
    while(ros::ok())
    {
        geometry_msgs::Twist msg;
        
        msg.linear.x=front;
        // ROS_INFO("pitch=%f",front);
        
        msg.angular.z=rotate;
        // ROS_INFO("roll=%f",rotate);
        
        // msg.angular.z=rotate2;
        // ROS_INFO("rotate=%f",rotate2);

        imu_controller.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;

}