/*
 * @Author: lichao951787328 951787328@qq.com
 * @Date: 2025-02-12 10:23:38
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2025-02-12 13:59:26
 * @FilePath: /path_planning/src/flatElevationMap.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <glog/logging.h>
using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "flat_elevation_map");
    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>("global_map", 1, true);

    grid_map::GridMap map({"elevation"});
    map.setFrameId("global_map");
    grid_map::Length length(5, 2);
    grid_map::Position position(1, 0);
    map.setGeometry(length, 0.02, position);
    map["elevation"].setConstant(0.0);
    grid_map_msgs::GridMap message;
    grid_map::GridMapRosConverter::toMessage(map, message);
    map_pub.publish(message);

    // 将这个保存成一个bag
    if (grid_map::GridMapRosConverter::saveToBag(map, "/home/lichao/catkin_location/src/hdl_localization/data/globalmap.bag", "global_map"))
    {
        LOG(INFO)<<"save global map success.";
        return true;
    }
    else
    {
        LOG(ERROR)<<"save global map error.";
        return false;
    }
    ros::spin();
    return 0;
}