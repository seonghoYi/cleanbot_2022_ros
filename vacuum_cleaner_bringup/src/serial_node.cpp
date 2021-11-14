#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Twist.h"
#include "atmega_serial.hpp"

const float PI = 3.1415926f;
const float WHEEL_WIDTH = 0.07f;
const float ROBOT_WIDTH = 0.23f;

void controlMotor(float left_speed, float right_speed);

void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel)
{
    //ROS_INFO("cmd_vel.linear.x: %f, cmd_vel.angular.z * ROBOT_WIDTH / 2: %f\n", cmd_vel.linear.x, cmd_vel.angular.z * ROBOT_WIDTH / 2);
    float left_speed_out = (cmd_vel.linear.x - cmd_vel.angular.z * ROBOT_WIDTH / 2); //v=rw 선속도=반지름*각속도
    float right_speed_out = (cmd_vel.linear.x + cmd_vel.angular.z * ROBOT_WIDTH / 2);
    //ROS_INFO("left speed: %fm/s, right_speed: %fm/s\n", left_speed_out, right_speed_out);
    controlMotor(left_speed_out, right_speed_out);
}

void servoCallback(const std_msgs::Bool &cmd_servo)
{
    if (cmd_servo.data)
    {
        closeClamper();
    }
    else
    {
        openClamper();
    }
}

void suctionCallback(const std_msgs::Bool &cmd_suction)
{
    if (cmd_suction.data)
    {
        runSuctionMotor();
    }
    else
    {
        stopSuctionMotor();
    }
}

void controlMotor(float left_speed, float right_speed)
{
    //static Controller device(port, baudrate, 128);
    //const float MAX_SPEED = 0.376f; //m/s
    
    bool L_dir;
    bool R_dir;
    float L_rpm;
    float R_rpm;

    
    if ((int)(left_speed * 1000000) == 0 || (int)(right_speed * 1000000) == 0)
    {
        stopMotor();
        return;
    }

    L_rpm = left_speed*60/(PI*WHEEL_WIDTH);
    R_rpm = right_speed*60/(PI*WHEEL_WIDTH);

    //ROS_INFO("%f, %f\n", L_rpm, R_rpm);

    if (left_speed > 0)
    {
        L_dir = false;
    }
    else
    {
        L_dir = true;
    }

    if (right_speed > 0)
    {
        R_dir = false;
    }
    else
    {
        R_dir = true;
    }
    //ROS_INFO("L_rpm: %d, R_rpm: %d, L_dir: %d, R_dir: %d\n", (int)abs(L_rpm), (int)abs(R_rpm), (int)L_dir, (int)R_dir);
    setLeftMotorSpeed((std::uint8_t) abs(L_rpm));
    setRightMotorSpeed((std::uint8_t) abs(R_rpm));
    setLeftMotorDirection((std::uint8_t) L_dir);
    setRightMotorDirection((std::uint8_t) R_dir);
    runMotor();
}


int main(int argc, char **argv)
{
    std::string port;
    int baudrate = 38400;
    ros_t rosserial;
    rosserial.state = 0;

    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("port", port, "/dev/ttyUSB1");
    nh_private.param<int>("baudrate", baudrate, 34800); 

    //nh.param<std::string>("port", port, "/dev/ttyUSB1");
    //nh.param<int>("baudrate", baudrate, 38400);

    initMotor(port, baudrate, 128);    

    ros::Subscriber motor_sub = nh.subscribe("cmd_vel", 1000, commandVelocityCallback);
    ros::Subscriber servo_sub = nh.subscribe("cmd_servo", 128, servoCallback);
    ros::Subscriber suction_sub = nh.subscribe("cmd_suction", 128, suctionCallback);
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("current_speed", 100);

    ros::Time last_time = ros::Time::now();

    int L_rpm, R_rpm;
    float L_speed, R_speed;
    geometry_msgs::Twist vel;

    ros::Rate(150.0);

    while(nh.ok())
    {
        ros::spinOnce();
        
        if (ros::Time::now().nsec - last_time.nsec >= 100000000)
        {
            rosserial.state = 0;
        }
        
        if (receive_packet(&rosserial))
        {
            if (rosserial.packet.inst == 0)
            {
                L_rpm = (rosserial.packet.msgs[2] << 0) & 0x00FF | (rosserial.packet.msgs[3] << 8) & 0xFF00;
                R_rpm = (rosserial.packet.msgs[4] << 0) & 0x00FF | (rosserial.packet.msgs[5] << 8) & 0xFF00;

                L_rpm = (rosserial.packet.msgs[0] && 0xFF) ? -L_rpm : L_rpm;
                R_rpm = (rosserial.packet.msgs[1] && 0xFF) ? -R_rpm : R_rpm;
                //ROS_INFO("%d, %d\n", L_rpm, R_rpm);
                //ROS_INFO("%X, %X, %X, %X\n", rosserial.packet.msgs[2], rosserial.packet.msgs[3], rosserial.packet.msgs[4], rosserial.packet.msgs[5]); 
                L_speed = L_rpm * (PI*WHEEL_WIDTH)/60;
                R_speed = R_rpm * (PI*WHEEL_WIDTH)/60;
                //ROS_INFO("%f, %f\n", L_speed, R_speed);

                vel.linear.x = (R_speed + L_speed) / 2;
                vel.angular.z = (R_speed - L_speed) / ROBOT_WIDTH;
                //ROS_INFO("%f, %f\n", vel.linear.x, vel.angular.z);
                pub.publish(vel);
            }
        }
        last_time = ros::Time::now();
    }

    return 0;
}