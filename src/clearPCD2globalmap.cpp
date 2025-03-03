#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <glog/logging.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <rosbag/bag.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <ros/package.h>
using namespace std;
// 这是一个点云预处理程序，用于将全局地图中的点云进行滤波和分割，提取出平面部分，并将平面部分高度小于5cm的点删除。
// 后续再去cloudcompare里将点云中的一些噪点删除
int main(int argc, char **argv)
{
    ros::init(argc, argv, "clearPCD2globalmap");

    std::string package_path = ros::package::getPath("path_planning");
    std::cout << "Package path: " << package_path << std::endl;
    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>("/globalmap", 1);
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    // 修过的点云
    pcl::io::loadPCDFile(package_path + "/data/globalmap1.pcd", pointcloud);

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
    cv::Mat originalImage;
    cv::Mat mask;
    cv::Mat filledImage;
    // const float minValue = global_map.get(height_layer).minCoeffOfFinites();
    const float minValue = -0.76;
    const float maxValue = global_map.get(height_layer).maxCoeffOfFinites();

    grid_map::GridMapCvConverter::toImage<unsigned char, 3>(global_map, height_layer, CV_8UC3, minValue, maxValue,originalImage);

    mask = cv::imread(package_path + "/data/mask1.png", cv::IMREAD_GRAYSCALE);
    if (mask.channels() > 1) 
    {
        cv::cvtColor(mask, mask, cv::COLOR_BGR2GRAY);
    }
    cv::imshow("originalImage", originalImage);
    cv::waitKey(0);
    

    // Inpaint the filtered mask
    const double radiusInPixels = 5;
    cv::inpaint(originalImage, mask, filledImage, radiusInPixels, cv::INPAINT_NS);
    cv::imshow("filledImage", filledImage);
    cv::waitKey(0);
    global_map.erase(height_layer);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(filledImage, height_layer, global_map, minValue, maxValue);

    for (grid_map::GridMapIterator iterator(global_map); !iterator.isPastEnd(); ++iterator) 
    {
        if (!global_map.isValid(*iterator, height_layer)) 
        {
            global_map.at(height_layer, *iterator) = -0.78;
        }
    }


    // 发布GridMap消息
    grid_map_msgs::GridMap globalmap_msg;
    grid_map::GridMapRosConverter::toMessage(global_map, globalmap_msg);
    globalmap_msg.info.header.frame_id = "globalmap";

    if (grid_map::GridMapRosConverter::saveToBag(global_map, package_path + "/data/globalmap.bag", "global_map"))
    {
        LOG(INFO)<<"save global map success.";
        // return true;
    }
    else
    {
        LOG(ERROR)<<"save global map error.";
        // return false;
    }

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        map_pub.publish(globalmap_msg);
        loop_rate.sleep();
    }
    
    
    // 后续还需要在cloudcompare里将点云中的一些噪点删除
    return 0;
}

