#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "std_srvs/Empty.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "nav_msgs/OccupancyGrid.h"
#include "costmap_2d/costmap_2d.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point32.h"

#include <vector>

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
    nav_msgs::OccupancyGrid local_grid_, global_grid_;
    tf::TransformListener local_listener, global_listener;
    
public:
    MapUpdater(ros::NodeHandle node);
    ~MapUpdater();

private:
    geometry_msgs::Point32 local_point, global_point;
    costmap_2d::Costmap2D local_costmap, global_costmap;

    int     getIndex(nav_msgs::OccupancyGrid &grid_, int x, int y);
    void    globalupdateCallback(const map_msgs::OccupancyGridUpdateConstPtr &msg);
    void    globalgridCallback(const nav_msgs::OccupancyGridConstPtr &grid);
    void    localupdateCallback(const map_msgs::OccupancyGridUpdateConstPtr &msg);
    void    localgridCallback(const nav_msgs::OccupancyGridConstPtr &grid);
    void    objnameCallback(const std_msgs::StringConstPtr &name);
    void    objpointCallback(const geometry_msgs::Point32ConstPtr &point);
};

MapUpdater::MapUpdater(ros::NodeHandle node) : nh(node)
{
    this->global_costmap_pub = this->nh.advertise<nav_msgs::OccupancyGrid>("vacuum_cleaner/modified_global_costmap", 100);
    this->global_costmap_sub = this->nh.subscribe("move_base/global_costmap/costmap", 100, &MapUpdater::globalgridCallback, this);
    this->global_costmap_update_sub = this->nh.subscribe("move_base/global_costmap/costmap_updates", 100, &MapUpdater::globalupdateCallback, this);
    this->local_costmap_pub = this->nh.advertise<nav_msgs::OccupancyGrid>("vacuum_cleaner/modified_local_costmap", 100);
    this->local_costmap_sub = this->nh.subscribe("move_base/local_costmap/costmap", 100, &MapUpdater::localgridCallback, this);
    this->local_costmap_update_sub = this->nh.subscribe("move_base/local_costmap/costmap_updates", 100, &MapUpdater::localupdateCallback, this);
    this->obj_name_sub = this->nh.subscribe("object_tracker/object_name", 100, &MapUpdater::objnameCallback, this);
    //this->obj_point_sub = this->nh.subscribe("object_tracker/object_point", 100, &MapUpdater::objpointCallback, this); 
}

MapUpdater::~MapUpdater()
{

}

int MapUpdater::getIndex(nav_msgs::OccupancyGrid &grid_, int x, int y){
    int sx = grid_.info.width;
    return y * sx + x;
}

void MapUpdater::globalgridCallback(const nav_msgs::OccupancyGridConstPtr &grid)
{
    global_grid_ = *grid;
    costmap_2d::Costmap2D costmap_(global_grid_.info.width, global_grid_.info.height, global_grid_.info.resolution, global_grid_.info.origin.position.x, global_grid_.info.origin.position.y);
    
    
    for(int y=0; y< global_costmap.getSizeInCellsY(); y++){
        for(int x=0; x< global_costmap.getSizeInCellsX(); x++){
            costmap_.setCost(x, y, global_grid_.data[getIndex(global_grid_,x,y)]);
        }
    }
    global_costmap = costmap_;
}

void MapUpdater::globalupdateCallback(const map_msgs::OccupancyGridUpdateConstPtr &msg)
{

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

void MapUpdater::objnameCallback(const std_msgs::StringConstPtr &name)
{
    //ROS_INFO("%s\n", name->data.c_str());
    tf::StampedTransform local_transform, global_transform;
    local_listener.waitForTransform("/odom", name->data, ros::Time(0), ros::Duration(1.0));
    global_listener.waitForTransform("/map", name->data, ros::Time(0), ros::Duration(1.0));
    try
    {
        local_listener.lookupTransform("/odom", name->data, ros::Time(0), local_transform);
        global_listener.lookupTransform("/map", name->data, ros::Time(0), global_transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        return;
    }

    global_point.x = global_transform.getOrigin().x();
    global_point.y = global_transform.getOrigin().y();
    global_point.z = 0;
    local_point.x = local_transform.getOrigin().x();
    local_point.y = local_transform.getOrigin().y();
    local_point.z = 0;
    //ROS_INFO("%f, %f, %f\n", local_transform.getOrigin().x(), local_transform.getOrigin().y(), local_transform.getOrigin().z());

    geometry_msgs::Point left_high;
    geometry_msgs::Point left_low;
    geometry_msgs::Point right_high;
    geometry_msgs::Point right_low;

    left_high.x = local_point.x + 0.2;
    left_high.y = local_point.y + 0.2;
    left_high.z = local_point.z;

    left_low.x = local_point.x - 0.2;
    left_low.y = local_point.y - 0.2;
    left_low.z = local_point.z;

    right_high.x = local_point.x + 0.2;
    right_high.y = local_point.y + 0.2;
    right_high.z = local_point.z;

    right_high.x = local_point.x + 0.2;
    right_high.y = local_point.y - 0.2;
    right_high.z = local_point.z;


    std::vector<geometry_msgs::Point> polygon;

    polygon.push_back(left_high);
    polygon.push_back(left_low);
    polygon.push_back(right_high);
    polygon.push_back(right_low);


    ROS_INFO("local: %x\n", local_costmap.setConvexPolygonCost(polygon, 0));

    left_high.x = global_point.x + 0.2;
    left_high.y = global_point.y + 0.2;
    left_high.z = global_point.z;

    left_low.x = global_point.x - 0.2;
    left_low.y = global_point.y - 0.2;
    left_low.z = global_point.z;

    right_high.x = global_point.x + 0.2;
    right_high.y = global_point.y + 0.2;
    right_high.z = global_point.z;

    right_high.x = global_point.x + 0.2;
    right_high.y = global_point.y - 0.2;
    right_high.z = global_point.z;

    polygon[0] = left_high;
    polygon[1] = left_low;
    polygon[2] = right_high;
    polygon[3] = right_low;

    ROS_INFO("global: %x\n", global_costmap.setConvexPolygonCost(polygon, 0));
    //costmap.setCost(point.x, point.y, 0);

    //map_msgs::OccupancyGridUpdate modified_costmap;
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

void MapUpdater::objpointCallback(const geometry_msgs::Point32ConstPtr &point)
{
    tf::StampedTransform transform;
    try
    {
        local_listener.lookupTransform("/camera_aligned_depth_to_color_frame", "/odom", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        return;
    }
    //ROS_INFO("%f, %f, %f\n", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    //ROS_INFO("%f, %f, %f\n", transform.getOrigin().x() + clear_point.x, transform.getOrigin().y() + clear_point.y, transform.getOrigin().z() + clear_point.z);

}



int main(int argc, char** argv)
{
    std_srvs::Empty emptymsg;

    ros::init(argc, argv, "costmap_cleaner");
    ros::NodeHandle nh;

    MapUpdater a(nh);
    

    
    ros::Rate r(10);

    while(nh.ok())
    {
        ros::spinOnce();
        ros::service::call("/move_base/clear_costmaps", emptymsg);
        r.sleep();
    }
}