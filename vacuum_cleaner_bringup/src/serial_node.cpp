#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "atmega_serial.hpp"

void controlMotor(float left_speed, float right_speed);

void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel)
{
    
    const float ROBOT_WIDTH = 0.27f;
    //ROS_INFO("cmd_vel.linear.x: %f, cmd_vel.angular.z * ROBOT_WIDTH / 2: %f\n", cmd_vel.linear.x, cmd_vel.angular.z * ROBOT_WIDTH / 2);
    float left_speed_out = cmd_vel.linear.x - cmd_vel.angular.z * ROBOT_WIDTH / 2; //v=rw 선속도=반지름*각속도
    float right_speed_out = cmd_vel.linear.x + cmd_vel.angular.z * ROBOT_WIDTH / 2;
    ROS_INFO("left speed: %fm/s, right_speed: %fm/s\n", left_speed_out, right_speed_out);
    controlMotor(left_speed_out, right_speed_out);
}

void controlMotor(float left_speed, float right_speed)
{
    //static Controller device(port, baudrate, 128);

    const long F_CPU = 16000000UL;
    const int CLK_DIV = 256;
    const float MAX_SPEED = 0.376f; //m/s
    const float PHI = 0.066f; // m
    const float STEP_ANGLE = 0.9f; // half step

    bool L_dir;
    bool R_dir;
    float L_rpm;
    float R_rpm;
    float L_pulse_freq;
    float R_pulse_freq;


    uint8_t L_ocr(0);
    uint8_t R_ocr(0);

    
    if ((int)(left_speed * 1000000) == 0 || (int)(right_speed * 1000000) == 0)
    {
        stopMotor();
        return;
    }

    L_rpm = (abs(left_speed) * 60) / (PHI * M_PI); // 분속 / 바퀴 지름 = rpm
    R_rpm = (abs(right_speed) * 60) / (PHI * M_PI);

    L_pulse_freq = L_rpm / (STEP_ANGLE / 6); // rpm = (1/6)*θ*P
    R_pulse_freq = R_rpm / (STEP_ANGLE / 6);

    L_ocr = (int)(F_CPU / (2 * CLK_DIV * L_pulse_freq));
    R_ocr = (int)(F_CPU / (2 * CLK_DIV * R_pulse_freq));

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

    ROS_INFO("L_tcnt: %d, L_dir: %d, R_tcnt: %d, R_dir: %d\n", L_ocr, L_dir, R_ocr, R_dir);
    configMotor((std::uint8_t)L_ocr, L_dir, (std::uint8_t)R_ocr, R_dir);

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