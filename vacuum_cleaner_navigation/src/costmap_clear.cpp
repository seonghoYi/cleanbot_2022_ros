#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "std_srvs/Empty.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "nav_msgs/OccupancyGrid.h"
#include "costmap_2d/costmap_2d.h"
#include "costmap_2d/cost_values.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PolygonStamped.h"

#include "vacuum_cleaner_msgs/ObjectList.h"

#include <vector>

typedef std::vector<geometry_msgs::Point> Polygon;

class MapUpdater
{
    ros::NodeHandle nh;
    ros::Publisher local_costmap_pub;
    ros::Subscriber local_costmap_sub;
    ros::Subscriber local_costmap_update_sub;
    ros::Publisher global_costmap_pub;
    ros::Subscriber global_costmap_sub;
    ros::Subscriber global_costmap_update_sub;
    ros::Subscriber obj_name_sub;
    ros::Subscriber obj_point_sub;
    ros::Subscriber obj_list_sub;
    nav_msgs::OccupancyGrid local_grid_, global_grid_;
    tf::TransformListener local_listener, global_listener;
    
public:
    MapUpdater(ros::NodeHandle node);
    ~MapUpdater();
    void task();

private:
    geometry_msgs::Point32 local_point, global_point;
    costmap_2d::Costmap2D local_costmap, global_costmap;
    std::vector<Polygon> local_polygon_list, global_polygon_list;

    int     getIndex(nav_msgs::OccupancyGrid &grid_, int x, int y);
    void    globalupdateCallback(const map_msgs::OccupancyGridUpdateConstPtr &msg);
    void    globalgridCallback(const nav_msgs::OccupancyGridConstPtr &grid);
    void    localupdateCallback(const map_msgs::OccupancyGridUpdateConstPtr &msg);
    void    localgridCallback(const nav_msgs::OccupancyGridConstPtr &grid);
    void    objnameCallback(const std_msgs::StringConstPtr &name);
    void    objpointCallback(const geometry_msgs::Point32ConstPtr &point);
    void    objlistCallback(const vacuum_cleaner_msgs::ObjectListConstPtr& msgs);
};

MapUpdater::MapUpdater(ros::NodeHandle node) : nh(node)
{
    this->global_costmap_pub = this->nh.advertise<nav_msgs::OccupancyGrid>("vacuum_cleaner/modified_global_costmap", 100, true);
    this->global_costmap_sub = this->nh.subscribe("move_base/global_costmap/costmap", 100, &MapUpdater::globalgridCallback, this);
    //this->global_costmap_update_sub = this->nh.subscribe("move_base/global_costmap/costmap_updates", 100, &MapUpdater::globalupdateCallback, this);
    this->local_costmap_pub = this->nh.advertise<nav_msgs::OccupancyGrid>("vacuum_cleaner/modified_local_costmap", 100, true);
    this->local_costmap_sub = this->nh.subscribe("move_base/local_costmap/costmap", 100, &MapUpdater::localgridCallback, this);
    //this->local_costmap_update_sub = this->nh.subscribe("move_base/local_costmap/costmap_updates", 100, &MapUpdater::localupdateCallback, this);
    //this->obj_name_sub = this->nh.subscribe("object_tracker/object_name", 100, &MapUpdater::objnameCallback, this);
    //this->obj_list_sub = this->nh.subscribe("vacuum_cleaner/object_list", 100, &MapUpdater::objlistCallback, this);
    //this->obj_list_sub = this->nh.subscribe("coral_ros/object_list", 100, &MapUpdater::objlistCallback, this);
    this->obj_list_sub = this->nh.subscribe("vacuum_cleaner/abs_object_list", 100, &MapUpdater::objlistCallback, this);
    //this->obj_point_sub = this->nh.subscribe("object_tracker/object_point", 100, &MapUpdater::objpointCallback, this); 
}

MapUpdater::~MapUpdater()
{

}

void MapUpdater::task()
{
    geometry_msgs::Point goal_left_high, goal_left_low, goal_right_high, goal_right_low;
    geometry_msgs::Point robot_left_high, robot_left_low, robot_right_high, robot_right_low;
    Polygon global_polygon, local_polygon;


    /*
    if (local_polygon_list.empty() || global_polygon_list.empty())
    {
        return;
    }
    */

    

    goal_left_high.x = -0.36;
    goal_left_high.y = 0.3;

    goal_left_low.x = -0.93;
    goal_left_low.y = 0.3;

    goal_right_high.x = -0.9;
    goal_right_high.y = -0.5;

    goal_right_low.x = -0.37;
    goal_right_low.y = -0.5;

    global_polygon.push_back(goal_left_high);
    global_polygon.push_back(goal_left_low);
    global_polygon.push_back(goal_right_high);
    global_polygon.push_back(goal_right_low);

    
    global_polygon_list.push_back(global_polygon);

    
    robot_left_high.x = -0.3;
    robot_left_high.y = -0.04;
    robot_left_low.x = 0.02;
    robot_left_low.y = 0.32;
    robot_right_high.x = -0.04;
    robot_right_high.y = -0.31;
    robot_right_low.x = -0.36;
    robot_right_low.y = 0.06;

    local_polygon.push_back(robot_left_high);
    local_polygon.push_back(robot_left_low);
    local_polygon.push_back(robot_right_high);
    local_polygon.push_back(robot_right_low);

    local_polygon_list.push_back(local_polygon);
    
    //ROS_INFO("%ld\n", polygon_list.size());
    for (auto it=local_polygon_list.begin(); it!=local_polygon_list.end(); it++)
    {
        ROS_INFO("%d", local_costmap.setConvexPolygonCost(*it, costmap_2d::FREE_SPACE));
        
    }

    for (auto it=global_polygon_list.begin(); it!=global_polygon_list.end(); it++)
    {
        global_costmap.setConvexPolygonCost(*it, costmap_2d::FREE_SPACE);
    }

    local_polygon_list.clear();
    global_polygon_list.clear();

    nav_msgs::OccupancyGrid mod_local_costmap = local_grid_;
    for(int y=0; y< local_costmap.getSizeInCellsY(); y++){
        for(int x=0; x< local_costmap.getSizeInCellsX(); x++){
            mod_local_costmap.data[getIndex(mod_local_costmap,x,y)] = local_costmap.getCost(x, y);
        }
    }

    nav_msgs::OccupancyGrid mod_global_costmap = global_grid_;
    for(int y=0; y< global_costmap.getSizeInCellsY(); y++){
        for(int x=0; x< global_costmap.getSizeInCellsX(); x++){
            mod_global_costmap.data[getIndex(mod_global_costmap,x,y)] = global_costmap.getCost(x, y);
        }
    }

    this->local_costmap_pub.publish(mod_local_costmap);
    this->global_costmap_pub.publish(mod_global_costmap);
}

int MapUpdater::getIndex(nav_msgs::OccupancyGrid &grid_, int x, int y){
    int sx = grid_.info.width;
    return y * sx + x;
}

void MapUpdater::globalgridCallback(const nav_msgs::OccupancyGridConstPtr &grid)
{
    global_grid_ = *grid;
    costmap_2d::Costmap2D costmap_(global_grid_.info.width, global_grid_.info.height, global_grid_.info.resolution, global_grid_.info.origin.position.x, global_grid_.info.origin.position.y);
    //ROS_INFO("%f, %f", costmap_.getOriginX(), costmap_.getOriginY());
    for(int y=0; y< global_costmap.getSizeInCellsY(); y++){
        for(int x=0; x< global_costmap.getSizeInCellsX(); x++){
            costmap_.setCost(x, y, global_grid_.data[getIndex(global_grid_,x,y)]);
        }
    }
    global_costmap = costmap_;
}

void MapUpdater::globalupdateCallback(const map_msgs::OccupancyGridUpdateConstPtr &msg)
{
    int index = 0;
    costmap_2d::Costmap2D costmap_(global_grid_.info.width, global_grid_.info.height, global_grid_.info.resolution, global_grid_.info.origin.position.x, global_grid_.info.origin.position.y);
    for(int y=msg->y; y< msg->y+msg->height; y++){
        for(int x=msg->x; x< msg->x+msg->width; x++){
            global_grid_.data[getIndex(global_grid_,x,y)] = msg->data[index++ ];
            costmap_.setCost(x, y, msg->data[index++ ]);
        }
    }
    global_costmap = costmap_;
}

void MapUpdater::localgridCallback(const nav_msgs::OccupancyGridConstPtr &grid)
{
    local_grid_ = *grid;
    costmap_2d::Costmap2D costmap_(local_grid_.info.width, local_grid_.info.height, local_grid_.info.resolution, local_grid_.info.origin.position.x, local_grid_.info.origin.position.y);
    
    for(int y=0; y< local_costmap.getSizeInCellsY(); y++){
        for(int x=0; x< local_costmap.getSizeInCellsX(); x++){
            costmap_.setCost(x, y, local_grid_.data[getIndex(local_grid_,x,y)]);
        }
    }
    local_costmap = costmap_;
}

void MapUpdater::localupdateCallback(const map_msgs::OccupancyGridUpdateConstPtr &msg)
{
    int index = 0;
    costmap_2d::Costmap2D costmap_(local_grid_.info.width, local_grid_.info.height, local_grid_.info.resolution, local_grid_.info.origin.position.x, local_grid_.info.origin.position.y);
    for(int y=msg->y; y< msg->y+msg->height; y++){
        for(int x=msg->x; x< msg->x+msg->width; x++){
            local_grid_.data[getIndex(local_grid_,x,y)] = msg->data[index++ ];
            costmap_.setCost(x, y, msg->data[index++ ]);
        }
    }
    local_costmap = costmap_;
    //this->costmap_pub.publish(grid_);
}

void MapUpdater::objlistCallback(const vacuum_cleaner_msgs::ObjectListConstPtr& msgs)
{
    vacuum_cleaner_msgs::ObjectList list = *msgs;
    geometry_msgs::PointStamped local_point_transfromed, global_point_transfromed;

    for(auto i=0; i<list.object_names.size(); i++)
    {
        try
        {
            local_listener.transformPoint("odom", list.object_points[i], local_point_transfromed);
            global_listener.transformPoint("map", list.object_points[i], global_point_transfromed);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            return;
        }
        /*
        local_point.x = local_transform.getOrigin().x();
        local_point.y = local_transform.getOrigin().y();
        local_point.z = 0;
        global_point.x = global_transform.getOrigin().x();
        global_point.y = global_transform.getOrigin().y();
        global_point.z = 0;
        */
        local_point.x = local_point_transfromed.point.x;
        local_point.y = local_point_transfromed.point.y;
        global_point.x = global_point_transfromed.point.x;
        global_point.y = global_point_transfromed.point.y;

        geometry_msgs::Point left_high;
        geometry_msgs::Point left_low;
        geometry_msgs::Point right_high;
        geometry_msgs::Point right_low;

        Polygon polygon(4);

        left_high.x = local_point.x + 0.12;
        left_high.y = local_point.y + 0.12;
        left_high.z = local_point.z;

        left_low.x = local_point.x - 0.12;
        left_low.y = local_point.y + 0.12;
        left_low.z = local_point.z;

        right_high.x = local_point.x + 0.12;
        right_high.y = local_point.y - 0.12;
        right_high.z = local_point.z;

        right_low.x = local_point.x - 0.12;
        right_low.y = local_point.y - 0.12;
        right_low.z = local_point.z;

        

        polygon[0] = left_high;
        polygon[1] = left_low;
        polygon[2] = right_high;
        polygon[3] = right_low;
        local_polygon_list.push_back(polygon);

        left_high.x = global_point.x + 0.12;
        left_high.y = global_point.y + 0.12;
        left_high.z = global_point.z;

        left_low.x = global_point.x - 0.12;
        left_low.y = global_point.y + 0.12;
        left_low.z = global_point.z;

        right_high.x = global_point.x + 0.12;
        right_high.y = global_point.y - 0.12;
        right_high.z = global_point.z;

        right_low.x = global_point.x - 0.12;
        right_low.y = global_point.y - 0.12;
        right_low.z = global_point.z;

        polygon.push_back(left_high);
        polygon.push_back(left_low);
        polygon.push_back(right_high);
        polygon.push_back(right_low);

        global_polygon_list.push_back(polygon);
    }
}



int main(int argc, char** argv)
{
    std_srvs::Empty emptymsg;

    ros::init(argc, argv, "costmap_cleaner");
    ROS_INFO("costmap clearing start");
    ros::NodeHandle nh;

    MapUpdater a(nh);
    

    
    ros::Rate r(20.0);

    while(nh.ok())
    {
        ros::spinOnce();
        ros::service::call("/move_base/clear_costmaps", emptymsg);
        a.task();
        r.sleep();
    }
}