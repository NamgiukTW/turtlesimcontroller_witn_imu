// 헤더에 선언된 변수 가공을 해당 라이브러리에 작성

#include "Listener2.h"

void Listener::imuDataCallback(const sensor_msgs::Imu &imuMsg)
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

    msg.linear.x = frontLocal1;
    msg.angular.z = rotateLocal1;

    {
        std::lock_guard<std::mutex> lock(m_Mut);
        m_Front = frontLocal1;
        m_Rotate = rotateLocal1;
    }

    // m_VeloCommandPublisher->publish(msg);
    // 퍼블리셔를 전역변수로 선언하면 m_VeloCommandPublisher->publish(msg);을 사용하지 않아도 된다.
    m_ImuData.publish(msg);
}

void Listener::poseDataCallback(const turtlesim::PoseConstPtr &turtlePoseMsg)
{

    float x1;
    float y1;
    float theta1;

    x1 = turtlePoseMsg->x;
    y1 = turtlePoseMsg->y;
    theta1 = turtlePoseMsg->theta;

    {
        std::lock_guard<std::mutex> lock(m_Mut);
        m_X = x1;
        m_Y = y1;
        m_Theta = theta1;
    }
}

void Listener::timeCallBack(const ros::TimerEvent &)
{

    float x2;
    float y2;
    float theta2;
    float frontLocal3;
    float rotateLocal3;

    {
        std::lock_guard<std::mutex> lock(m_Mut);
        x2 = m_X;
        y2 = m_Y;
        theta2 = m_Theta;
        frontLocal3 = m_Front;
        rotateLocal3 = m_Rotate;
    }

    cout << "===========================" << endl;
    cout << "Hwo are you? " << this << endl;
    cout << "X_posithon = " << x2 << endl;
    cout << "Y_posithon = " << y2 << endl;
    cout << "theta_Position = " << theta2 << endl;
    cout << "linear_Velocity = " << frontLocal3 << endl;
    cout << "angular_Velocity = " << rotateLocal3 << endl;
    
}

Listener::Listener(void) :
                        m_Front(0),
                        m_Rotate(0),
                        m_X(0),
                        m_Y(0),
                        m_Theta(0),
                        m_Mut(),
                        m_ImuData(),
                        m_Nh(),
                        m_MyCallbackQueue(),
                        m_Spinner(0, &m_MyCallbackQueue),
                        m_TurtlesimControl(),
                        m_TurtlesimPoseSub(),
                        m_Timer()
                        // 생성자를 사용할 때 선언했던 모든 변수를 이렇게 작성한다.(일종의 사용법이다.)
{
    // 생성이 될 때, 해야하는 행동을 여기에 작성

    m_Nh.setCallbackQueue(&m_MyCallbackQueue);

    m_ImuData = m_Nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",100);

    m_TurtlesimControl = m_Nh.subscribe("/imu/data", 100, &Listener::imuDataCallback, this);
    m_TurtlesimPoseSub = m_Nh.subscribe("/turtle1/pose", 100, &Listener::poseDataCallback, this);
    m_Timer = m_Nh.createTimer(ros::Duration(1.0), &Listener::timeCallBack, this);
    // 만약 class를 같은 노드 안에서 사용하고 int main에 서브스크라이버나 타임을 선언한다면
    // this가 아닌 class이름으로 선언한 변수를 &(변수)로 입력해야 한다.
    // 예 : Listener listen1 으로 클래스를 변수로 선언했다 가정하면 &listen1 으로 입력 

    m_Spinner.start();
}

Listener::~Listener(void)
{
    // 소멸을 할 때, 해야하는 행동을 여기에 작성
}
//소멸자는 생성자와 역순 호출