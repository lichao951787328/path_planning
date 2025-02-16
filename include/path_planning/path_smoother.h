/*
 * @Author: lichao951787328 951787328@qq.com
 * @Date: 2025-02-12 10:15:31
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2025-02-16 15:31:02
 * @FilePath: /path_planning/include/path_planning/path_smoother.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
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
    string globalmap_frame_id;
public:
    path_smoother(ros::NodeHandle & n);
    void pathCallback(const nav_msgs::Path::ConstPtr & msg);
    void mapCallback(const grid_map_msgs::GridMap::ConstPtr & msg);
    void run();
    ~path_smoother();
};

}
