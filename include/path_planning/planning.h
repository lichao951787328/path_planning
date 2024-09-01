#pragma once
#include <grid_map_core/GridMap.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <peac/PEAC_plane_detection.hpp>
#include <vector>
#define DEBUG

struct SupportArea
{
    double Up, Button, Left, Right;
    SupportArea(double up, double button, double left, double right)
    {
        CHECK(up > 0 && button > 0 && left > 0 && right > 0);
        Up = up; Button = button; Left = left; Right = right;
    }
    SupportArea()
    {

    }
};


class PathPlanning
{
private:
    ros::NodeHandle nh;
    grid_map::GridMap map;
    ros::Publisher pub;
    SupportArea support_area;
    plane_detection pd;
    cv::Mat obstacle_layer;
    int full_size; // 支撑区域内cell数
    
    // 只有这些区域的不是全向通行或者全向非通行
    std::vector<cv::Mat> checks_Mat;
    cv::Mat Nan_feasible; // 所有不可通行的
    cv::Mat Full_feasible; // 所有全向通行的
public:
    PathPlanning(ros::NodeHandle & nodehand, grid_map::GridMap & map_, SupportArea support_area_);

    void constructPlaneAwareMap();

    void constructObstacleLayer(int chect_scope);

    bool isFeasible(grid_map::Index & index, double angle);
    bool isFeasibleNew(grid_map::Index & index, double angle);

    void checkFeasibleDirect();

    bool getPoints(grid_map::Position TL, grid_map::Position TR, grid_map::Position BL, grid_map::Position BR, vector<Eigen::Vector3d> & return_points);

    inline cv::Mat getObstacleLayer()
    {
        return obstacle_layer;
    }

    inline cv::Mat getNanRegion()
    {
        return Nan_feasible;
    }

    inline cv::Mat getFullRegion()
    {
        return Full_feasible;
    }

    ~PathPlanning();


    // test
    
    void testisFeasible();
};



