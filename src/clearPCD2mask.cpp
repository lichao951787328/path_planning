/*
 * @Author: lichao951787328 951787328@qq.com
 * @Date: 2025-02-21 09:23:36
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2025-03-03 09:38:07
 * @FilePath: /path_planning/src/clearPCD2mask.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <glog/logging.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include <ros/ros.h>
using namespace std;
// 使用得到的点云创建一个mask，这个mask是高程图中为nan的点，后续需要手动筛选出那些地面点，对剩余的点进行inpaint
int main(int argc, char **argv)
{
    ros::init(argc, argv, "clearPCD2mask");
    std::string package_path = ros::package::getPath("path_planning");
    std::cout << "Package path: " << package_path << std::endl;
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    // 修过的点云
    pcl::io::loadPCDFile(package_path + "/data/globalmap1.pcd", pointcloud);


    // 原始点云
    // pcl::io::loadPCDFile("/home/lichao/catkin_pathplanning/src/path_planning/data/globalmap.pcd", pointcloud);
    // 计算点云在x，y方向的中点及在x y方向上点离中点的最远距离
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(pointcloud, centroid);
    double x_center = centroid[0];
    double y_center = centroid[1];

    double max_distance_x = 0.0;
    double max_distance_y = 0.0;
    for (const auto& point : pointcloud.points)
    {
        double distance_x = std::abs(point.x - x_center);
        double distance_y = std::abs(point.y - y_center);
        if (distance_x > max_distance_x)
        {
            max_distance_x = distance_x;
        }
        if (distance_y > max_distance_y)
        {
            max_distance_y = distance_y;
        }
    }

    // 根据采集过程可以得到地面点云的坐标系和全局定位点云坐标系重合
    LOG(INFO)<<"Point cloud centroid: x: "<<x_center<<", y: "<<y_center;
    LOG(INFO)<<"Max distance from centroid in x direction: "<<max_distance_x;
    LOG(INFO)<<"Max distance from centroid in y direction: "<<max_distance_y;
    

    // 创建一个GridMap对象
    grid_map::GridMap global_map({"elevation"});

    // 设置GridMap的分辨率和尺寸
    double resolution = 0.02;  // 设置分辨率
    global_map.setGeometry(grid_map::Length(2 * max_distance_x, 2 * max_distance_y), resolution, grid_map::Position(x_center, y_center));
    global_map.setFrameId("map");
    // 填充GridMap的高程数据
    for (const auto& point : pointcloud.points)
    {
        grid_map::Position position(point.x, point.y);
        if (global_map.isInside(position))
        {
            global_map.atPosition("elevation", position) = point.z;
        }
    }
    
    global_map.add("inpaint_mask", 0.0);
    string height_layer = "elevation";
    global_map.setBasicLayers(std::vector<std::string>());
    for (grid_map::GridMapIterator iterator(global_map); !iterator.isPastEnd(); ++iterator) {
        if (!global_map.isValid(*iterator, height_layer)) {
            global_map.at("inpaint_mask", *iterator) = 1.0;
        }
    }
    cv::Mat mask;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(global_map, "inpaint_mask", CV_8UC1, mask);
    cv::imshow("mask", mask);
    cv::imwrite(package_path + "/data/mask1.png", mask);
    cv::waitKey(0);

    return 0;
}