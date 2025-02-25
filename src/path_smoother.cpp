/*
 * @Author: lichao951787328 951787328@qq.com
 * @Date: 2025-02-12 10:15:31
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2025-02-16 15:31:07
 * @FilePath: /path_planning/src/path_smoother.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <path_planning/path_smoother.h>

namespace path_smooth
{


path_smoother::path_smoother(ros::NodeHandle & n):nh(n)
{
    int Bspline_degree;
    int Bspline_numPoints;
    nh.getParam("Bspline_degree", Bspline_degree);
    nh.getParam("Bspline_numPoints", Bspline_numPoints);
    nh.getParam("globalmap_frame_id", globalmap_frame_id);
    bspline_param.degree = Bspline_degree;
    bspline_param.numPoints = Bspline_numPoints;
    bspline_param.printParam();
    path_sub = nh.subscribe("path", 1, &path_smoother::pathCallback, this);
    path_pub = nh.advertise<nav_msgs::Path>("smoothed_path", 1);
    map_sub = nh.subscribe("global_map", 1, &path_smoother::mapCallback, this);
}

void path_smoother::pathCallback(const nav_msgs::Path::ConstPtr & msg)
{
    ROS_INFO("Received path");
    raw_path = *msg;
    path_received = true;
    run();
}

void path_smoother::mapCallback(const grid_map_msgs::GridMap::ConstPtr & msg)
{
    // ROS_INFO("Received map");
    grid_map::GridMapRosConverter::fromMessage(*msg, map);
    map_received = true;
    run();
}

void path_smoother::run()
{
    if (map_received && path_received)
    {
        ROS_INFO("Starting path smoothing");
        map_received = false;
        path_received = false;
        pathSmoother path_smoother(raw_path, map, bspline_param);
        path_smoother.smoothBSPline();
        nav_msgs::Path smoothed_path = path_smoother.getSmoothedPath();
        double length = 0;
        for (int i = 0; i < smoothed_path.poses.size() - 1; i++)
        {
            Eigen::Vector2d v1(smoothed_path.poses[i].pose.position.x, smoothed_path.poses[i].pose.position.y);
            Eigen::Vector2d v2(smoothed_path.poses[i + 1].pose.position.x, smoothed_path.poses[i + 1].pose.position.y);
            length += (v1 - v2).norm();
        }
        std::cout<<"Path length: "<<length<<std::endl;
        std::cout<<"Path smoothed: "<<smoothed_path.poses.size()<<std::endl;
        smoothed_path.header.frame_id = globalmap_frame_id;
        path_pub.publish(smoothed_path);
    }
    
}
path_smoother::~path_smoother()
{
}

}
