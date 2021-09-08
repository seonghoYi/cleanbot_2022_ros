#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"


void poseCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;

    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,
                                    msg->pose.pose.position.y,
                                    msg->pose.pose.position.z));

    tf::Quaternion q;

    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);
    q.setW(msg->pose.pose.orientation.w);

    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_tf_publisher");
    ros::NodeHandle nh;
    ros::Subscriber odom_sub = nh.subscribe("odom", 100, poseCallback);
    ros::Rate r(50);

    ros::spin();

    /*
    tf::TransformBroadcaster broadcaster;

    while(nh.ok())
    {
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(0.07, 0, 0.09)),
                ros::Time::now(), "base_link", "laser_frame"));
        r.sleep();
    }
    */
    /*
    tf::TransformBroadcaster br;
    tf::Transform transform;

    while(nh.ok())
    {
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));

        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));

        r.sleep();
    }
    */



    return 0;
}