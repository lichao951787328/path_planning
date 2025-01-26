#include <grid_map_core/GridMap.hpp>
#include <iostream>
#include <glog/logging.h>
#include <ros/ros.h>
#include <grid_map_core/iterators/SubmapIterator.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <path_planning/planning.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/Marker.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_filters/MeanInRadiusFilter.hpp>
// 是否使用rviz来获取起点和终点
#define SET_BY_RVIZ
#define IN_PAINT
Eigen::Vector3d start, goal;
bool start_get = false, goal_get = false;


void StartCallback(const geometry_msgs::PoseWithCovarianceStamped::Ptr msg)
{
    // 提取 x 和 y 坐标
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    // 提取四元数
    tf2::Quaternion quat;
    quat.setX(msg->pose.pose.orientation.x);
    quat.setY(msg->pose.pose.orientation.y);
    quat.setZ(msg->pose.pose.orientation.z);
    quat.setW(msg->pose.pose.orientation.w);

    // 将四元数转换为欧拉角
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    start[0] = x;
    start[1] = y;
    start[2] = yaw;
    // 输出 x, y, 和 yaw
    ROS_INFO("start: x: %f, y: %f, yaw: %f", x, y, yaw * 57.3);
    start_get = true;
}

void GoalCallback(const geometry_msgs::PoseStamped::Ptr msg)
{
    // 提取 x 和 y 坐标
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;

    // 提取四元数
    tf2::Quaternion quat;
    quat.setX(msg->pose.orientation.x);
    quat.setY(msg->pose.orientation.y);
    quat.setZ(msg->pose.orientation.z);
    quat.setW(msg->pose.orientation.w);

    // 将四元数转换为欧拉角
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    goal[0] = x;
    goal[1] = y;
    goal[2] = yaw;
    // 输出 x, y, 和 yaw
    ROS_INFO("goal: x: %f, y: %f, yaw: %f", x, y, yaw * 57.3);
    goal_get = true;
}

visualization_msgs::Marker convertPath2visualmsgs(nav_msgs::Path & path)
{
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = 0.4;  // 设置线条宽度
    path_marker.color.a = 1.0;  // 设置透明度
    path_marker.color.r = 0.0;  // 设置红色
    path_marker.color.g = 1.0;  // 设置绿色
    path_marker.color.b = 0.0;  // 设置蓝色
    for (int i = 0; i < path.poses.size() - 1; i++)
    {
        // 添加路径点
        geometry_msgs::Point p1;
        p1.x = path.poses.at(i).pose.position.x;
        p1.y = path.poses.at(i).pose.position.y;
        p1.z = path.poses.at(i).pose.position.z;
        path_marker.points.push_back(p1);

        geometry_msgs::Point p2;
        p2.x = path.poses.at(i+1).pose.position.x;
        p2.y = path.poses.at(i+1).pose.position.y;
        p2.z = path.poses.at(i+1).pose.position.z;
        path_marker.points.push_back(p2);
    }
    
    return path_marker;
}

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]); 
    google::InstallFailureSignalHandler();
    // google::SetCommandLineOptionWithMode("FLAGS_minloglevel", "2");
    FLAGS_minloglevel = 0;
    FLAGS_colorlogtostderr = true;
    FLAGS_alsologtostderr = true;
    ros::init(argc, argv, "planning");
    ros::NodeHandle nh;

    // 加载参数
    double support_area_up, support_area_button, support_area_left, support_area_right;
    // double resolution;
    int obstacle_rad;
    double obstacle_inflation_radius, obstacle_length, safe_region_radius;

    double d_safe_goal, d_vort_goal, d_noinflu_offset_goal;

    // 普通障碍距离参数
    double d_safe_gen, d_vort_gen, d_noinflu_offset_gen;

    double goal_obstacle_cof, gen_obstacle_cof, d_g_att, d_g_att_cof;
    double support_ratio, step;

    nh.getParam("support_area_up", support_area_up);
    nh.getParam("support_area_button", support_area_button);
    nh.getParam("support_area_left", support_area_left);
    nh.getParam("support_area_right", support_area_right);
    
    nh.getParam("obstacle_inflation_radius", obstacle_inflation_radius);
    nh.getParam("obstacle_length", obstacle_length);
    nh.getParam("obstacle_rad", obstacle_rad);
    nh.getParam("safe_region_radius", safe_region_radius);

    // 终点障碍
    nh.getParam("d_safe_goal", d_safe_goal);
    nh.getParam("d_vort_goal", d_vort_goal);
    nh.getParam("d_noinflu_offset_goal", d_noinflu_offset_goal);

    // 一般障碍
    nh.getParam("d_safe_gen", d_safe_gen);
    nh.getParam("d_vort_gen", d_vort_gen);
    nh.getParam("d_noinflu_offset_gen", d_noinflu_offset_gen);


    nh.getParam("goal_obstacle_cof", goal_obstacle_cof);
    nh.getParam("gen_obstacle_cof", gen_obstacle_cof);
    nh.getParam("d_g_att", d_g_att);
    nh.getParam("d_g_att_cof", d_g_att_cof);
    nh.getParam("support_ratio", support_ratio);
    nh.getParam("step", step);
    LOG(INFO)<<support_area_up<<" "<<support_area_button<<" "<<support_area_left<<" "<<support_area_right<<" "<<obstacle_inflation_radius<<" "<<obstacle_length<<" "<<obstacle_rad<<" "<<safe_region_radius<<" "<<d_safe_goal<<" "<<d_vort_goal<<" "<<d_noinflu_offset_goal<<d_safe_gen<<" "<<d_vort_gen<<" "<<d_noinflu_offset_gen<<" "<<goal_obstacle_cof<<" "<<gen_obstacle_cof<<" "<<d_g_att<<" "<<d_g_att_cof<<" "<<support_ratio<<" "<<step;
    planningParam param;
   
    ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, StartCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, GoalCallback);
    ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>("global_map", 1);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    ros::Publisher path_marker_pub = nh.advertise<visualization_msgs::Marker>("path_marker", 1);

    // string globalmap_bag_path, globalmap_topic;
    // nh.param("globalmap_bag_path", globalmap_bag_path, string("/home/lichao/catkin_pathplanning/src/path_planning/bag/globalmap.bag"));
    // nh.param("globalmap_topic", globalmap_topic, string("globalmap"));

    // 从采集的点云中转换成高程图
    
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    // 修过的点云
    pcl::io::loadPCDFile("/home/lichao/catkin_pathplanning/src/path_planning/data/globalmap_filte_seg.pcd", pointcloud);
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

    ROS_INFO("Point cloud centroid: x: %f, y: %f", x_center, y_center);
    ROS_INFO("Max distance from centroid in x direction: %f", max_distance_x);
    ROS_INFO("Max distance from centroid in y direction: %f", max_distance_y);

    // 创建一个GridMap对象
    grid_map::GridMap global_map({"elevation"});
    // grid_map::GridMap global_map;
    

    // grid_map::MeanInRadiusFilter ;

    // 设置GridMap的分辨率和尺寸
    double resolution = 0.02;  // 设置分辨率
    global_map.setGeometry(grid_map::Length(2 * max_distance_x, 2 * max_distance_y), resolution, grid_map::Position(x_center, y_center));
    // global_map.add("elevation", 0.0);
    // global_map;
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

#ifdef IN_PAINT

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
    const float minValue = 0.0;
    const float maxValue = global_map.get(height_layer).maxCoeffOfFinites();

    grid_map::GridMapCvConverter::toImage<unsigned char, 3>(global_map, height_layer, CV_8UC3, minValue, maxValue,originalImage);
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(global_map, "inpaint_mask", CV_8UC1, mask);
    // cv::imshow("originalImage", originalImage);
    // cv::waitKey(0);
    // LOG(INFO)<<"CCC";
    // 2. 定义滤波参数
    cv::Size kernelSize(3, 3);  // 5x5 高斯核
    double sigmaX = 1.2;        // X 方向的标准差
    double sigmaY = 1.2;        // Y 方向的标准差
   
    // 3. 应用高斯滤波
    cv::Mat smoothedImage;
    cv::GaussianBlur(originalImage, smoothedImage, kernelSize, sigmaX, sigmaY);
    global_map.erase(height_layer);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(smoothedImage, height_layer, global_map, minValue, maxValue);

    if (global_map.exists(height_layer))
    {
        LOG(INFO)<<"add layer success";
    }
    
#endif

    // const double radiusInPixels = 0.1 / global_map.getResolution();
    // cv::inpaint(originalImage, mask, filledImage, radiusInPixels, cv::INPAINT_NS);
    // global_map.erase(height_layer);
    // grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(filledImage, height_layer, global_map, minValue, maxValue);
    // global_map.erase("inpaint_mask");

    // 发布GridMap消息
    grid_map_msgs::GridMap globalmap_msg;
    grid_map::GridMapRosConverter::toMessage(global_map, globalmap_msg);

    param.support_area.Up = support_area_up;
    param.support_area.Button = support_area_button;
    param.support_area.Left = support_area_left;
    param.support_area.Right = support_area_right;
    param.resolution = global_map.getResolution();
    param.obstacle_inflation_radius = obstacle_inflation_radius;
    param.obstacle_length = obstacle_length;
    param.safe_region_radius = safe_region_radius;
    param.obstacle_rad = obstacle_rad;

    param.d_safe_goal = d_safe_goal;
    param.d_vort_goal = d_vort_goal;
    param.d_noinflu_offset_goal = d_noinflu_offset_goal;

    param.d_safe_gen = d_safe_gen;
    param.d_vort_gen = d_vort_gen;
    param.d_noinflu_offset_gen = d_noinflu_offset_gen;
    
    param.computeRadius();
    param.goal_obstacle_cof = goal_obstacle_cof;
    param.gen_obstacle_cof = gen_obstacle_cof;
    param.d_g_att = d_g_att;
    param.d_g_att_cof = d_g_att_cof;
    param.support_ratio = support_ratio;
    param.step = step;
    param.computeSupportNumTh();

    // // 自己先给定终点，来对比普通Astar和人工势场Astar的区别
    // start[0] = 3.26811;
    // start[1] = -2.39386;
    // start[2] = 0.289504;
#ifndef SET_BY_RVIZ
    start = Eigen::Vector3d(5.23212, 2.57562, 1.81097);
    goal = Eigen::Vector3d(-0.992767, 3.33181, -3.06899);
#endif
    // goal[0] = 7.21435;
    // goal[1] = 2.10046;
    // goal[2] = 0;
    sleep(2);

    LOG(INFO)<<"PUB";
    // 下面是通过rviz给定起点和终点
    ros::Rate loop_rate(0.4);
    while (ros::ok())
    {
        map_pub.publish(globalmap_msg);
        loop_rate.sleep();
#ifndef SET_BY_RVIZ
        PathPlanning path_planning(global_map, start, goal, param);
        path_planning.processing();
        nav_msgs::Path path = path_planning.getPath();
        path.header.frame_id = "map";
        path_pub.publish(path);
        // visualization_msgs::Marker path_marker = convertPath2visualmsgs(path);
        // path_marker_pub.publish(path_marker);
        // loop_rate.sleep();
#else
        ros::spinOnce();
        // // start: x: 2.338864, y: 0.339921, yaw: 11.690219
        // start[0] = 2.338864;
        // start[1] = 0.339921;
        // start[2] = 11.690219/57.3;

        // // goal: x: 3.358081, y: 3.320854, yaw: 104.043889
        // goal[0] = 3.358081;
        // goal[1] = 3.320854;
        // goal[2] = 104.043889/57.3;

        // start: x: 2.305987, y: 0.471433, yaw: 13.173531
        start[0] = 2.305987;
        start[1] = 0.471433;
        start[2] = 13.173531/57.3;
        // goal: x: 4.530727, y: 3.375651, yaw: 70.351363
        goal[0] = 4.530727;
        goal[1] = 3.375651;
        goal[2] = 70.351363/57.3;

        PathPlanning path_planning(global_map, start, goal, param);
        path_planning.processing();
        LOG(INFO)<<"OUT";

        // if (!(start_get && goal_get))
        // {
        //     LOG(INFO)<<"wait start and goal";
        //     loop_rate.sleep();
        //     continue;
        // }
        // else
        // {
        //     LOG(INFO)<<"initial path planning";
        //     LOG(INFO)<<start.transpose();
        //     LOG(INFO)<<goal.transpose();
        //     PathPlanning path_planning(global_map, start, goal, param);
        //     path_planning.processing();
        //     nav_msgs::Path path = path_planning.getPath();
        //     // path_planning.constructPlaneAwareMap();
        //     // path_planning.constructObstacleLayer(6);
        //     // path_planning.computeRep();
        //     path.header.frame_id = "map";
        //     path_pub.publish(path);
        //     start_get = false;
        //     goal_get = false;
        // }
#endif
    }
    LOG(INFO)<<"OUT";
    

    return 0;
}