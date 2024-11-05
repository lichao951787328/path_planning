#pragma once

#include <path_planning/path_smooth.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace path_smooth
{


class path_smoother
{
private:
    ros::NodeHandle nh;
    ros::Publisher path_pub;
    ros::Subscriber path_sub;
    ros::Subscriber map_sub;
    grid_map::GridMap map;
    bool map_received = false;
    nav_msgs::Path raw_path;
    bool path_received = false;
    BsplineParams bspline_param;
public:
    path_smoother(ros::NodeHandle & n);
    void pathCallback(const nav_msgs::Path::ConstPtr & msg);
    void mapCallback(const grid_map_msgs::GridMap::ConstPtr & msg);
    void run();
    ~path_smoother();
};

}
