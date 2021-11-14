#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"

#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"

#include "tf/transform_listener.h"
#include "tf/tf.h"

#include "vacuum_cleaner_msgs/ObjectList.h"

#include <map>
#include <vector>
#include <tuple>
#include <algorithm>
#include <cmath>

/*
namespace ObjectHistory
{
typedef std::multimap<std::string, float> objHistory;
typedef std::pair<std::string, float> obj;

class ObjectHistory
{
    objHistory obj_history;
    

public:
    ObjectHistory();
    ObjectHistory(objHistory obj_history_);
    ~ObjectHistory();
    bool updateHistory(obj obj_);
    bool sortByDistance(void);
    std::pair<std::string, float> getHistory(void);

private:
    std::multimap<float, std::string> obj_sorted_by_distance;
    std::multimap<float, std::string>::iterator index, end_index;

};

ObjectHistory::ObjectHistory()
{

}


ObjectHistory::ObjectHistory(objHistory obj_history_) : obj_history(obj_history_)
{

}

ObjectHistory::~ObjectHistory()
{

}

bool ObjectHistory::updateHistory(obj obj_)
{
    bool ret = false;
    bool is_same = false;
    //std::pair<std::map<std::string, float>::iterator, std::map<std::string, float>::iterator> iter_pair;


    //sort
    std::multimap<float, std::string> temp;
    for (auto i=obj_history.begin(); i!=obj_history.end(); i++)
    {
        ROS_INFO("%s, %f\n", i->first.c_str(), i->second);
        temp.insert({i->second, i->first});
    }
    obj_history.clear();
    for (auto i=temp.begin(); i!=temp.end(); i++)
    {
        obj_history.insert({i->second, i->first});
    }

    //search
    auto iter_pair = this->obj_history.equal_range(obj_.first);

    //ROS_INFO("%s, %f\n", iter_pair.first->first.c_str(), iter_pair.first->second);
    //ROS_INFO("%s, %f\n", iter_pair.second->first.c_str(), iter_pair.second->second);
    
    if (iter_pair.first == iter_pair.second) // very first time
    {
        //ROS_INFO("new!\n");
        obj_history.insert(obj_);
        return true;
    }

    for (objHistory::iterator iter = iter_pair.first; iter != iter_pair.second; iter++) //after detected object
    {
        //ROS_INFO("%s, %f\n", iter->first.c_str(), iter->second);
        if ((obj_.second - 0.2 <= iter->second) && (iter->second <= obj_.second + 0.2)) // resolution 0.2m
        {
            is_same = true;
            break;
        }
        else
        {
            is_same = false;
        }
    }

    if (is_same)
    {
        return false;
    }
    else
    {
        //ROS_INFO("%s, %f\n", iter->first.c_str(), iter->second);
        //ROS_INFO("%s, %f, %f\n", iter->first.c_str(), iter->second, obj_.second);
        obj_history.insert(obj_);
        return true;
    }

    return false;
}

bool ObjectHistory::sortByDistance(void)
{
    for (auto i=obj_history.begin(); i!=obj_history.end(); i++)
    {
        ROS_INFO("%s, %f\n", i->first.c_str(), i->second);
        obj_sorted_by_distance.insert({i->second, i->first});
    }

    index = obj_sorted_by_distance.begin();
    end_index = obj_sorted_by_distance.end();
}

std::pair<std::string, float> ObjectHistory::getHistory(void)
{
    index++;
    if (index == end_index)
    {
        return {"end", 0};
    }

    return {index->second, index->first};
}

}
*/

namespace ObjectHistory
{
typedef std::vector<std::tuple<std::string, float, geometry_msgs::Point32>> ObjHistory;
typedef std::tuple<std::string, float, geometry_msgs::Point32> OBJ;
typedef std::pair<ObjHistory::iterator, ObjHistory::iterator> ObjIndex;


bool compareString(const OBJ& a, const OBJ& b) //ascending sort
{
    if (std::get<0>(a).compare(std::get<0>(b)) > 0)
    {
        return true;
    }
    else if (std::get<0>(a).compare(std::get<0>(b)) < 0)
    {
        return false;
    }
    else
    {
        return false;
    }
}

bool compareDistance(const OBJ& a, const OBJ& b) //discending sort
{
    return std::get<1>(a) < std::get<1>(b);
}

std::pair<ObjHistory::iterator, ObjHistory::iterator> equal_range(const ObjHistory::iterator& begin, const ObjHistory::iterator& end, std::string str)
{
    ObjHistory::iterator return_begin, return_end;

    for (auto it=begin; it!=end; it++)
    {
        if (std::get<0>(*it).compare(str) == 0)
        {
            //ROS_INFO("%s\n", std::get<0>(*it).c_str());
            return_begin = it;
            break;
        }
    }

    for (auto it=begin; it!=end; it++)
    {
        if (std::get<0>(*it).compare(str) == 0)
        {
            return_end = it+1; //past the end!!!
        }
    }
    return {return_begin, return_end};
}


class ObjectHistory
{
public:
    ObjectHistory();
    ObjectHistory(ObjHistory obj_history_);
    ~ObjectHistory();
    bool updateHistory(OBJ obj_);
    bool sortByDistance(void);
    OBJ getHistory(void);
    ObjIndex readHistory(void);

private:
    ObjHistory obj_history, obj_sorted_by_distance, obj_history_backup;
    ObjHistory::iterator index, end_index;

};

ObjectHistory::ObjectHistory()
{

}


ObjectHistory::ObjectHistory(ObjHistory obj_history_) : obj_history(obj_history_)
{

}

ObjectHistory::~ObjectHistory()
{

}

bool ObjectHistory::updateHistory(OBJ obj_)
{
    bool ret = false;
    bool is_same = false;
    //std::pair<std::map<std::string, float>::iterator, std::map<std::string, float>::iterator> iter_pair;
    //ROS_INFO("%ld\n", sizeof(obj_));
    //ROS_INFO("%ld\n", obj_history.size());
    
    obj_history_backup = obj_history;

    //sort
    std::sort(obj_history.begin(), obj_history.end(), compareString);
    /*
    for (auto it = obj_history.begin(); it!=obj_history.end(); it++)
    {
        ROS_INFO("%s, %f, (%f, %f)\n", std::get<0>(*it).c_str(), std::get<1>(*it), std::get<2>(*it).x,std::get<2>(*it).y );
    }
    */
    //ROS_INFO("%s, %f\n", iter_pair.first->first.c_str(), iter_pair.first->second);
    //ROS_INFO("%s, %f\n", iter_pair.second->first.c_str(), iter_pair.second->second);

    auto iter_pair = equal_range(obj_history.begin(), obj_history.end(), std::get<0>(obj_));
    
    if (iter_pair.first == obj_history.end()) // very first time
    {
        ROS_INFO("new!");
        obj_history.push_back(obj_);
        return true;
    }

    for (ObjHistory::iterator iter = iter_pair.first; iter != iter_pair.second; iter++) //after detected object
    {
        //ROS_INFO("%s, %f\n", std::get<0>(*iter).c_str(), std::get<1>(*iter));
        if ((std::get<1>(obj_) - 0.2 <= std::get<1>(*iter)) && (std::get<1>(*iter) <= std::get<1>(obj_) + 0.2)) // resolution 0.2m
        {
            is_same = true;
            break;
        }
        else
        {
            is_same = false;
        }
    }
    //ROS_INFO("?\n");

    if (is_same)
    {
        return false;
    }
    else
    {
        //ROS_INFO("%s, %f\n", iter->first.c_str(), iter->second);
        //ROS_INFO("%s, %f, %f\n", iter->first.c_str(), iter->second, obj_.second);
        obj_history.push_back(obj_);
        return true;
    }   

    

    return false;
}

bool ObjectHistory::sortByDistance(void)
{   
    /*
    for (auto i=obj_history.begin(); i!=obj_history.end(); i++)
    {
        ROS_INFO("%s, %f\n", std::get<0>(*i).c_str(), std::get<1>(*i));
    }
    */
    obj_sorted_by_distance.resize(obj_history.size());
    std::copy(obj_history.begin(), obj_history.end(), obj_sorted_by_distance.begin());

    obj_history.clear();
    ObjHistory().swap(obj_history); // old history clear
    std::sort(obj_sorted_by_distance.begin(), obj_sorted_by_distance.end(), compareDistance);

    /*
    for (auto i=obj_sorted_by_distance.begin(); i!=obj_sorted_by_distance.end(); i++)
    {
        ROS_INFO("%s, %f\n", std::get<0>(*i).c_str(), std::get<1>(*i));
    }
    */

    index = obj_sorted_by_distance.begin();
    end_index = obj_sorted_by_distance.end();

    return true;
}

OBJ ObjectHistory::getHistory(void)
{
    
    if (index == end_index)
    {
        geometry_msgs::Point32 point;
        point.x = 0;
        point.y = 0;
        point.z = 0;
        obj_sorted_by_distance.clear();
        ObjHistory().swap(obj_sorted_by_distance);
        return {"NULL", 0.0, point};
    }

    auto ret = index++;

    return *ret;
}

ObjIndex ObjectHistory::readHistory(void)
{
    auto begin=obj_history_backup.begin(), end=obj_history_backup.end();

    return std::make_pair(begin, end);
}

}


enum
{
    RECYCLE_START = 0,
    RECYCLE_GET_NEXT_TARGET,
    RECYCLE_GOAL_SEND,
    RECYCLE_GOAL_REACHED,
    RECYCLE_FRONT_TARGET,
    RECYCLE_CATCH_TARGET,
    RECYCLE_TO_GOAL_READY,
    RECYCLE_FRONT_GOAL_SITE,
    RECYCLE_GOAL_SITE_REACHED,
    RECYCLE_READY,
    RECYCLE_READY_FACE
};

enum
{
    PLAN_FRONT_TARGET = 0,
    PLAN_CATCH_TARGET,
    PLAN_READY_TO_GOAL,
    PLAN_GO_TO_GOAL_FRONT,
    PLAN_GO_TO_GOAL,
    PLAN_READY,
    PLAN_READY_RECYCLE
};

enum
{
    SEARCHING_SEQ_0 = 1,
    SEARCHING_SEQ_1,
    SEARCHING_SEQ_2,
    SEARCHING_SEQ_3,
    SEARCHING_SEQ_4,
    SEARCHING_SEQ_5,
    SEARCHING_SEQ_6
};



typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void move_to(MoveBaseClient &ac, geometry_msgs::Pose2D plan)
{
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    tf::Quaternion q;
    q.setEuler(0, 0, plan.theta);
    goal.target_pose.pose.position.x = plan.x;
    goal.target_pose.pose.position.y = plan.y;
    goal.target_pose.pose.position.z = 0;
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();

    ac.sendGoal(goal);
}

class Scheduler
{
    geometry_msgs::Point32 CAN_GOAL, PLASTIC_GOAL, CARTON_GOAL, READY_GOAL;
    ros::NodeHandle nh_;
    ros::Subscriber move_state_sub_;
    ros::Subscriber obj_list_sub_;
    ros::Subscriber amcl_pose_sub_;
    ros::Publisher servo_pub_;
    ros::Publisher marker_pub_, abs_obj_point_pub_;
    tf::TransformListener obj_listener, base_listener;
    tf::StampedTransform obj_transform, base_transform;

    MoveBaseClient ac;

    std::string obj_name;
    float distance;
    ObjectHistory::ObjectHistory object_history;

public:
    Scheduler(ros::NodeHandle nh);
    ~Scheduler();
    void task();
    void abs_object_publisher();
    

private:
    int next_state, current_state, last_state, search_seq;
    bool obj_check;
    ros::Time current_time, last_time;
    geometry_msgs::Pose2D current_pose;
    std::vector<geometry_msgs::Pose2D> plan;

    std::string target;

    bool is_moving;
    void moveStateCallback(const std_msgs::BoolConstPtr &msg);
    void objectListCallback(const vacuum_cleaner_msgs::ObjectListConstPtr& msgs);
    void amclPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

};

Scheduler::Scheduler(ros::NodeHandle nh) : nh_(nh), ac("move_base", true)
{
    next_state = RECYCLE_START;
    search_seq = SEARCHING_SEQ_0;
    obj_check = false;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    CAN_GOAL.x = -0.4;
    CAN_GOAL.y = 0.0;
    CAN_GOAL.z = 0.0;

    PLASTIC_GOAL.x = -0.4;
    PLASTIC_GOAL.y = -0.15;
    PLASTIC_GOAL.z = 0.0;

    CARTON_GOAL.x = -0.4;
    CARTON_GOAL.y = -0.25;
    CARTON_GOAL.z = 0.0;

    READY_GOAL.x = -0.25;
    READY_GOAL.y = -0.1;
    READY_GOAL.z = 0;

    this->servo_pub_ = this->nh_.advertise<std_msgs::Bool>("cmd_servo", 100);
    this->abs_obj_point_pub_ = this->nh_.advertise<vacuum_cleaner_msgs::ObjectList>("vacuum_cleaner/abs_object_list", 100);
    this->move_state_sub_ = this->nh_.subscribe("move_state", 100, &Scheduler::moveStateCallback, this);
    this->obj_list_sub_ = this->nh_.subscribe("coral_ros/object_list", 100, &Scheduler::objectListCallback, this);
    this->amcl_pose_sub_ = this->nh_.subscribe("amcl_pose", 100, &Scheduler::amclPoseCallback, this);

    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
}

Scheduler::~Scheduler()
{

}


void Scheduler::task()
{
    current_state = next_state;
    current_time = ros::Time::now();
    if (current_state == RECYCLE_START)
    {
        obj_check = true;
        if ((current_time - last_time).toSec() > 3.0)
        {
            ROS_INFO("\n-------sampling Done!-------");
            last_time = ros::Time::now();;
            object_history.sortByDistance();
            obj_check = false;
            last_state = current_state;
            next_state = RECYCLE_GET_NEXT_TARGET;
        }
    }
    else if (current_state == RECYCLE_GET_NEXT_TARGET)
    {
        ObjectHistory::OBJ obj = object_history.getHistory();
        ROS_INFO("current_target: %s\n", std::get<0>(obj).c_str());
        
        base_listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
        
        if (std::get<0>(obj) == "NULL")
        {
            geometry_msgs::Pose2D search_pose = current_pose;;

            switch (search_seq)
            {
            case SEARCHING_SEQ_0:
                search_pose.theta = 0*M_PI/180;
                break;
            case SEARCHING_SEQ_1:
                search_pose.theta = 30*M_PI/180;
                break;
            case SEARCHING_SEQ_2:
                search_pose.theta = 60*M_PI/180;
                break;
            case SEARCHING_SEQ_3:
                search_pose.theta = 90*M_PI/180;
                break;
            case SEARCHING_SEQ_4:
                search_pose.theta = -30*M_PI/180;
                break;
            case SEARCHING_SEQ_5:
                search_pose.theta = -60*M_PI/180;
                break;
            case SEARCHING_SEQ_6:
                search_pose.theta = -90*M_PI/180;
                break;
            default:
                search_seq = 0; // executiong last state
                break;           
            }
            search_seq++;

            move_to(ac, search_pose);
            
            last_state = current_state;
            next_state = RECYCLE_GOAL_SEND;
            
            last_time = ros::Time::now();
            return;
        }

        try
        {
            base_listener.lookupTransform("/map", "/base_link", ros::Time(0), base_transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            return;
        }


        float local_distance = sqrt(pow(base_transform.getOrigin().x() - std::get<2>(obj).x, 2) + pow(base_transform.getOrigin().y() - std::get<2>(obj).y, 2));
        geometry_msgs::Pose2D target_front_pose, catch_target_pose, goal_ready_pose, goal_front_pose, goal_pose, ready_pose, ready_face_pose;
        target_front_pose.x = std::get<2>(obj).x - 0.35 * cos(current_pose.theta);
        target_front_pose.y = std::get<2>(obj).y - 0.35 * sin(current_pose.theta);
        target_front_pose.theta = current_pose.theta;
        ROS_INFO("target_front= x: %f, y: %f, th: %f", target_front_pose.x, target_front_pose.y, target_front_pose.theta);

        plan.push_back(target_front_pose);

        catch_target_pose.x = std::get<2>(obj).x - 0.1 * cos(current_pose.theta);
        catch_target_pose.y = std::get<2>(obj).y - 0.1 * sin(current_pose.theta);
        catch_target_pose.theta = current_pose.theta;
        ROS_INFO("target_catch= x: %f, y: %f, th: %f", catch_target_pose.x, catch_target_pose.y, catch_target_pose.theta);
        plan.push_back(catch_target_pose);

        goal_ready_pose.x = std::get<2>(obj).x;
        goal_ready_pose.y = std::get<2>(obj).y;
        goal_ready_pose.theta = -180*M_PI/180;
        ROS_INFO("ready_goal= x: %f, y: %f, th: %f", goal_ready_pose.x, goal_ready_pose.y, goal_ready_pose.theta);

        plan.push_back(goal_ready_pose);

        goal_ready_pose.theta = -180*M_PI/180;
        goal_ready_pose.x = READY_GOAL.x - 0.1 * cos(goal_ready_pose.theta);
        goal_ready_pose.y = READY_GOAL.y - 0.1 * sin(goal_ready_pose.theta);
        
        ROS_INFO("goal_ready_pose= x: %f, y: %f, th: %f", goal_ready_pose.x, goal_ready_pose.y, goal_ready_pose.theta);

        plan.push_back(goal_ready_pose);

        if (std::get<0>(obj) == "can")
        {
            goal_pose.x = CAN_GOAL.x;
            goal_pose.y = CAN_GOAL.y;
            goal_pose.theta = 135*M_PI/180; //-180*M_PI/180;
        }
        else if (std::get<0>(obj) == "plastic_bottle")
        {
            goal_pose.x = PLASTIC_GOAL.x;
            goal_pose.y = PLASTIC_GOAL.y;
            goal_pose.theta = -180*M_PI/180;
        }
        else if (std::get<0>(obj) == "carton")
        {  
            goal_pose.x = CARTON_GOAL.x;
            goal_pose.y = CARTON_GOAL.y;
            goal_pose.theta = -135*M_PI/180; //-180*M_PI/180;
        }

        ROS_INFO("goal_pose= x: %f, y: %f, th: %f", goal_pose.x, goal_pose.y, goal_pose.theta);

        plan.push_back(goal_pose);


        ready_pose.x = READY_GOAL.x;
        ready_pose.y = READY_GOAL.y;
        ready_pose.theta = 0; // replanning below
        ROS_INFO("ready_pose= x: %f, y: %f, th: %f//will replan", ready_pose.x, ready_pose.y, ready_pose.theta);
        plan.push_back(ready_pose);

        ready_face_pose.x = READY_GOAL.x;
        ready_face_pose.y = READY_GOAL.y;
        ready_face_pose.theta = current_pose.theta;
        ROS_INFO("ready_face_pose= x: %f, y: %f, th: %f", ready_face_pose.x, ready_face_pose.y, ready_face_pose.theta);
        plan.push_back(ready_face_pose);


        last_state = current_state;
        next_state = RECYCLE_FRONT_TARGET;
    }
    else if (current_state == RECYCLE_FRONT_TARGET)
    {
        
        move_to(ac, plan[PLAN_FRONT_TARGET]);
        last_state = current_state;
        next_state = RECYCLE_GOAL_SEND;
    }
    else if (current_state == RECYCLE_CATCH_TARGET)
    {
        if ((current_time - last_time).toSec() > 3.0)
        {
            move_to(ac, plan[PLAN_CATCH_TARGET]);
            last_state = current_state;
            next_state = RECYCLE_GOAL_SEND;
        }
    }
    else if (current_state == RECYCLE_TO_GOAL_READY)
    {
        move_to(ac, plan[PLAN_READY_TO_GOAL]);
        last_state = current_state;
        next_state = RECYCLE_GOAL_SEND;
    }
    else if (current_state == RECYCLE_FRONT_GOAL_SITE)
    {
        
        move_to(ac, plan[PLAN_GO_TO_GOAL_FRONT]);
        last_state = current_state;
        next_state = RECYCLE_GOAL_SEND;
        
        
    }
    else if (current_state == RECYCLE_GOAL_SITE_REACHED)
    {
        move_to(ac, plan[PLAN_GO_TO_GOAL]);
        last_state = current_state;
        next_state = RECYCLE_GOAL_SEND;
    }
    else if (current_state == RECYCLE_READY)
    {
        plan[PLAN_READY].theta = current_pose.theta;
        move_to(ac, plan[PLAN_READY]);
        last_state = current_state;
        next_state = RECYCLE_GOAL_SEND;
    }
    else if (current_state == RECYCLE_READY_FACE)
    {
        move_to(ac, plan[PLAN_READY_RECYCLE]);
        last_state = current_state;
        next_state = RECYCLE_GOAL_SEND;
    }


    if (current_state == RECYCLE_GOAL_SEND)
    {
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            bool tmp;
            ROS_INFO("Goal Reached");

            switch (last_state)
            {
            case RECYCLE_GET_NEXT_TARGET:
                last_state = RECYCLE_GOAL_REACHED;
                next_state = RECYCLE_START;
                last_time = ros::Time::now();
                break;
            case RECYCLE_FRONT_TARGET:
                last_state = RECYCLE_GOAL_REACHED;
                next_state = RECYCLE_CATCH_TARGET;
                last_time = ros::Time::now();
                break;
            case RECYCLE_CATCH_TARGET:
                tmp = true;
                servo_pub_.publish(tmp);
                last_state = RECYCLE_GOAL_REACHED;
                next_state = RECYCLE_TO_GOAL_READY;
                break;
            case RECYCLE_TO_GOAL_READY:
                last_state = RECYCLE_GOAL_REACHED;
                next_state = RECYCLE_FRONT_GOAL_SITE;
                
                break;
            case RECYCLE_FRONT_GOAL_SITE:
                last_state = RECYCLE_FRONT_GOAL_SITE;
                next_state = RECYCLE_GOAL_SITE_REACHED;
                break;
            case RECYCLE_GOAL_SITE_REACHED:
                tmp = false;
                servo_pub_.publish(tmp);
                last_state = RECYCLE_GOAL_SITE_REACHED;
                next_state = RECYCLE_READY;
                break;
            case RECYCLE_READY:  
                last_state = RECYCLE_GOAL_SITE_REACHED;
                next_state = RECYCLE_READY_FACE;
                last_time = ros::Time::now();
                break;
            case RECYCLE_READY_FACE:
                plan.clear();
                last_state = RECYCLE_GOAL_SITE_REACHED;
                next_state = RECYCLE_START;
                last_time = ros::Time::now();
                break;
            }      
        }
    }

}

void Scheduler::abs_object_publisher(void)
{
    ObjectHistory::ObjIndex idx = this->object_history.readHistory();
    vacuum_cleaner_msgs::ObjectList abs_obj_list;
    geometry_msgs::PointStamped temp;
    temp.header.frame_id = "map";
    temp.header.stamp = ros::Time::now();

    for(auto it=idx.first; it!=idx.second; it++)
    {
        temp.point.x = std::get<2>(*it).x;
        temp.point.y = std::get<2>(*it).y;

        abs_obj_list.object_names.push_back(std::get<0>(*it));
        abs_obj_list.object_points.push_back(temp);
    }

    abs_obj_point_pub_.publish(abs_obj_list);
}

void Scheduler::amclPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_pose.theta = yaw;
}

void Scheduler::moveStateCallback(const std_msgs::BoolConstPtr &msg)
{
    this->is_moving = msg->data;
}

void Scheduler::objectListCallback(const vacuum_cleaner_msgs::ObjectListConstPtr& msgs)
{
    vacuum_cleaner_msgs::ObjectList list = *msgs;
    geometry_msgs::PointStamped point_transformed;

    if (!obj_check)
    {
        return;
    }

    
    for(auto i=0; i<list.object_names.size(); i++)
    {
        try
        {
            obj_listener.transformPoint("map", list.object_points[i], point_transformed);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            return;
        }

        geometry_msgs::Point32 obj;
        obj.x = point_transformed.point.x;
        obj.y = point_transformed.point.y;
        obj.z = 0;

        distance = sqrt(pow(obj.x, 2) + pow(obj.y, 2));
        std::string parsed_name = list.object_names[i].substr(0, list.object_names[i].find('-'));
        bool test = object_history.updateHistory({parsed_name, distance, obj});
    }



}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "recycle_scheduler");

    ros::NodeHandle nh;

    Scheduler scheduler(nh);

    ros::Rate r(40.0);

    while(nh.ok())
    {
        ros::spinOnce();
        scheduler.task();
        //scheduler.object_marker_publisher();
        scheduler.abs_object_publisher();
        r.sleep();
    }
}