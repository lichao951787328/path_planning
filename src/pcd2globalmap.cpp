#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_cv/GridMapCvProcessing.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <rosbag/bag.h>
#include <opencv2/opencv.hpp>
using namespace std;
int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd2globalmap");
    ros::NodeHandle nh; 
    
    string cloud_path, height_layer, globalmap_frameid, globalmap_topic, globalmap_path;
    nh.param("cloud_path", cloud_path, string("/home/lichao/catkin_pathplanning/src/global_map/data/globalmap.pcd"));
    nh.param("height_layer", height_layer, string("elevation"));
    nh.param("globalmap_frameid", globalmap_frameid, string("map"));
    nh.param("globalmap_topic", globalmap_topic, string("globalmap"));
    nh.param("globalmap_path", globalmap_path, string("/home/lichao/catkin_pathplanning/src/path_planning/bag/globalmap.bag"));
    
    ros::Publisher globalmap_pub = nh.advertise<grid_map_msgs::GridMap>(globalmap_topic, 1, true);

    double resolution;
    nh.param("resolution", resolution, 0.02);

    double off_set;
    nh.param("off_set", off_set, 0.2);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZ> (cloud_path, cloud);
    ROS_INFO("Loaded %d points", (int) cloud.points.size());

    grid_map::GridMap map({height_layer});
    map.setFrameId(globalmap_frameid);

    double max_abs_x = 0;
    double max_abs_y = 0;
    for (auto & point : cloud)
    {
        if (abs(point.x) > max_abs_x)
        {
            max_abs_x = abs(point.x);
        }
        if (abs(point.y) > max_abs_y)
        {
            max_abs_y = abs(point.y);
        }
    }
    
    grid_map::Length length(2 * max_abs_x, 2 * max_abs_y);
    grid_map::Position position(max_abs_x-0.2, 0);
    map.setGeometry(length, resolution, position);
    for (auto & point : cloud)
    {
        grid_map::Index index;
        if (map.getIndex(grid_map::Position(point.x, point.y), index))
        {
            map[height_layer](index.x(), index.y()) = point.z;
        }
    }


    //Convert elevation layer to OpenCV image to fill in holes.
    //Get the inpaint mask (nonzero pixels indicate where values need to be filled in).
    map.add("inpaint_mask", 0.0);

    map.setBasicLayers(std::vector<std::string>());
    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        if (!map.isValid(*iterator, height_layer)) {
        map.at("inpaint_mask", *iterator) = 1.0;
        }
    }
    cv::Mat originalImage;
    cv::Mat mask;
    cv::Mat filledImage;
    const float minValue = map.get(height_layer).minCoeffOfFinites();
    const float maxValue = map.get(height_layer).maxCoeffOfFinites();

    grid_map::GridMapCvConverter::toImage<unsigned char, 3>(map, height_layer, CV_8UC3, minValue, maxValue,originalImage);
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "inpaint_mask", CV_8UC1, mask);

    const double radiusInPixels = 0.1 / map.getResolution();
    cv::inpaint(originalImage, mask, filledImage, radiusInPixels, cv::INPAINT_NS);
    map.erase(height_layer);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(filledImage, height_layer, map, minValue, maxValue);
    map.erase("inpaint_mask");

    grid_map_msgs::GridMap globalmap_msg;
    grid_map::GridMapRosConverter::toMessage(map, globalmap_msg);
    rosbag::Bag bag;
    bag.open(globalmap_path, rosbag::bagmode::Write);
    bag.write(globalmap_topic, ros::Time::now(), globalmap_msg);
    bag.close();

    ros::Rate loop_rate(2);
    while (ros::ok())
    {
        globalmap_pub.publish(globalmap_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}


