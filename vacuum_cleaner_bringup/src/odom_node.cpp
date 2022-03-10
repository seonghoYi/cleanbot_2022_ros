#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/Imu.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"

#include <cmath>

static float cmd_vx, cmd_vth, vx, vth;
static float th_imu;
 // initial pose
static double x(0), y(0), th(0);

void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel)
{
    cmd_vx = cmd_vel.linear.x;
    cmd_vth = cmd_vel.angular.z;
}

void velocityCallback(const geometry_msgs::Twist &vel)
{
    vx = vel.linear.x;
    //vth = vel.angular.z;
}

void imuCallback(const sensor_msgs::Imu &imu)
{
    vth = imu.angular_velocity.z;
    //ROS_INFO("vth: %f\n", vth);
    th_imu = tf::getYaw(imu.orientation);

    //ROS_INFO("%f, %f\n", vth, th_imu);

}

bool odomClear(std_srvs::Empty::Request &req,
                std_srvs::Empty::Response &res)
{
    x = 0;
    y = 0;
    th = 0;
    vx = 0;
    vth = 0;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_pub");
    ros::NodeHandle nh;
    //ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1000, commandVelocityCallback);
    ros::Subscriber imu_sub = nh.subscribe("imu/data_filtered", 100, imuCallback);
    ros::Subscriber vel_sub = nh.subscribe("current_speed", 1024, velocityCallback);
    
    ros::Publisher move_state_pub = nh.advertise<std_msgs::Bool>("move_state", 128);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 128);

    ros::ServiceServer odom_clear = nh.advertiseService("vacuum_cleaner/odom_clear", odomClear);
    tf::TransformBroadcaster odom_broadcaster;


   

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    std_srvs::Empty empty_msgs;
    std_msgs::Bool is_moving;

    ros::Rate r(50.0);

    while(nh.ok())
    {
        ros::spinOnce();
        current_time = ros::Time::now();


        double dt = (current_time - last_time).toSec();
        
        double delta_x = vx * cos(th) * dt;
        double delta_y = vx * sin(th) * dt;
        double delta_th = vth * dt;
        

        x += delta_x;
        y += delta_y;
        //th += delta_th;
        th = th_imu;
        /*
        double delta_translation = vx * dt;
        double delta_th = vth * dt;
        
        th += delta_th;
        //th = delta_th;
        x += delta_translation * cos(th);
        y += delta_translation * sin(th);
        */
        //ROS_INFO("th: %lf\n", th);
        //ROS_INFO("x:%f, y:%f, th:%f\n", x, y, th);
        //ROS_INFO("delta_translation: %f, delta_th:%f\n", delta_translation, delta_th);
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);
        
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx * cos(th);
        odom.twist.twist.linear.y = vx * sin(th);
        odom.twist.twist.angular.z = vth;
		

		odom.pose.covariance[0] = 0.00001;
        odom.pose.covariance[7] = 0.00001;
        odom.pose.covariance[14] = 1000000000000.0;
        odom.pose.covariance[21] = 1000000000000.0;
        odom.pose.covariance[28] = 1000000000000.0;
        odom.pose.covariance[35] = 0.001;

        last_time = current_time;

        odom_pub.publish(odom);

        if (abs(delta_x) < 0.0001 && abs(delta_y) < 0.0001 && abs(delta_th) < 0.001)
        {
            is_moving.data = false;
            move_state_pub.publish(is_moving);
        }
        else
        {
            is_moving.data = true;
            move_state_pub.publish(is_moving);
        }

        r.sleep();
    }
}
