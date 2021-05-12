#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"


static float vx, vth;


void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel)
{
    vx = cmd_vel.linear.x;
    vth = cmd_vel.angular.z;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_pub");
    ros::NodeHandle nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;


    // initial pose
    double x = 0;
    double y = 0;
    double th = 0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(1.0);

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
        th += delta_th;

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


        odom_pub.publish(odom);
        last_time = current_time;

        r.sleep();
    }
}