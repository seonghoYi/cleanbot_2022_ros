#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "atmega_serial.hpp"

void controlMotor(float left_speed, float right_speed);

void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel)
{
    const float ROBOT_WIDTH = 0.35f;
    //ROS_INFO("cmd_vel.linear.x: %f, cmd_vel.angular.z * ROBOT_WIDTH / 2: %f\n", cmd_vel.linear.x, cmd_vel.angular.z * ROBOT_WIDTH / 2);
    float left_speed_out = cmd_vel.linear.x - cmd_vel.angular.z * ROBOT_WIDTH / 2; //v=rw 선속도=반지름*각속도
    float right_speed_out = cmd_vel.linear.x + cmd_vel.angular.z * ROBOT_WIDTH / 2;
    //ROS_INFO("left speed: %fm/s, right_speed: %fm/s\n", left_speed_out, right_speed_out);
    controlMotor(left_speed_out, right_speed_out);
}

void controlMotor(float left_speed, float right_speed)
{
    //static Controller device(port, baudrate, 128);
    //const float MAX_SPEED = 0.376f; //m/s
    const float PI = 3.1415926;
    const float WHEEL_WIDTH = 0.07f;
    
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

    if (left_speed > 0)
    {
        L_dir = true;
    }
    else
    {
        L_dir = false;
    }

    if (right_speed > 0)
    {
        R_dir = true;
    }
    else
    {
        R_dir = false;
    }

    setLeftMotorSpeed((std::uint8_t) L_rpm);
    setLeftMotorDirection((std::uint8_t) L_dir);
    setRightMotorSpeed((std::uint8_t) R_rpm);
    setRightMotorDirection((std::uint8_t) R_dir);
}


int main(int argc, char **argv)
{
    std::string port;
    int baudrate = 38400;


    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    //ros::NodeHandle nh_private("~");
    //nh_private.param<std::string>("port", port, "/dev/ttyUSB1");
    //nh_private.param<int>("baudrate", baudrate, 34800); 

    nh.param<std::string>("port", port, "/dev/ttyUSB1");
    nh.param<int>("baudrate", baudrate, 38400);

    initMotor(port, baudrate, 128);    

    ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, commandVelocityCallback);

    ros::spin();

    return 0;
}