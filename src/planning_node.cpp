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
// 是否使用rviz来获取起点和终点
#define SET_BY_RVIZ

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
    // 使用仿真环境来创建地图
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // pcl::io::loadPCDFile("/home/lichao/catkin_pathplanning/src/path_planning/sim-sence/164241000.pcd", cloud);
    // Eigen::Matrix4f transform;

    // transform.setIdentity();
    // transform(0, 0) = -1;
    // transform(2, 2) = -1;
    // transform(0, 3) = 3;
    // transform(2, 3) = 28;

    // pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    // pcl::transformPointCloud(cloud, transformed_cloud, transform);
    // pcl::io::savePCDFileASCII("/home/lichao/catkin_pathplanning/src/path_planning/sim-sence/164241000_transformed.pcd", transformed_cloud);


    // grid_map::GridMap map;
    // grid_map::Length length(10, 10);
    // grid_map::Position position(2.5, 0.5);
    // map.setGeometry(length, 0.02, position);

    // map.add("elevation", 0.0);

    // for (auto & point : transformed_cloud)
    // {
    //     grid_map::Position position(point.x, point.y);
    //     if (map.isInside(position))
    //     {
    //         map.atPosition("elevation", position) = point.z;
    //     }
    // }
    

    // 不再使用自己设计的简单高程图，而是使用load pcd
    // grid_map::Position stair_s(8, 2.8);
    // grid_map::Position stair_e(6, 1);
    // double step_w = 0.5;
    // int index = 1;
    // for (double s = stair_e.x(); s < stair_s.x(); s+=step_w)
    // {
    //     grid_map::Position step_s(s, stair_s.y());
    //     grid_map::Position step_e(s+step_w, stair_e.y());
    //     double step_elevation = index * 0.1;
    //     index++;
    //     grid_map::Index step_s_index;
    //     grid_map::Size step_size(step_w/map.getResolution() + 2, 1.5/map.getResolution());
    //     if (map.getIndex(step_s, step_s_index))
    //     {
    //         for (grid_map::SubmapIterator iter(map, step_s_index, step_size); !iter.isPastEnd(); ++iter)
    //         {
    //             grid_map::Index iter_index(*iter);
    //             map["elevation"](iter_index.x(), iter_index.y()) = step_elevation;
    //         }
    //     }
    // }
    // grid_map::Position obstacle_s(3, -0.5);
    // grid_map::Index obstacle_s_index;
    // if (map.getIndex(obstacle_s, obstacle_s_index))
    // {
    //     grid_map::Size obstacle_size(1/map.getResolution(), 0.4/map.getResolution());
    //     for (grid_map::SubmapIterator iter(map, obstacle_s_index, obstacle_size); !iter.isPastEnd(); ++iter)
    //     {
    //         grid_map::Index iter_index(*iter);
    //         map["elevation"](iter_index.x(), iter_index.y()) = 1.0;
    //     }
    // }

    string globalmap_bag_path, globalmap_topic;
    nh.param("globalmap_bag_path", globalmap_bag_path, string("/home/lichao/catkin_pathplanning/src/path_planning/bag/globalmap.bag"));
    nh.param("globalmap_topic", globalmap_topic, string("globalmap"));

    // 从bag中加载地图
    rosbag::Bag bag;
    bag.open(globalmap_bag_path, rosbag::bagmode::Read);

    // 使用 topic 名称来过滤消息
    std::vector<std::string> topics;
    topics.push_back(globalmap_topic);

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    grid_map_msgs::GridMap globalmap_msg;
    // 遍历 bag 文件中的消息
    for (const rosbag::MessageInstance& msg : view) 
    {
        grid_map_msgs::GridMap::ConstPtr path_msg = msg.instantiate<grid_map_msgs::GridMap>();
        if (path_msg != nullptr) 
        {
            globalmap_msg = *path_msg;
        }
    }
    bag.close();

    grid_map::GridMap global_map;
    grid_map::GridMapRosConverter::fromMessage(globalmap_msg, global_map);

    // grid_map_msgs::GridMap globalmap_msg;
    // grid_map::GridMapRosConverter::toMessage(map, globalmap_msg);
    // globalmap_msg.info.header.frame_id = "map";

    // 保存自己创建的地图
    // rosbag::Bag sim_map_bag;
    // sim_map_bag.open("/home/lichao/catkin_pathplanning/src/path_planning/bag/sim_map2.bag", rosbag::bagmode::Write);
    // sim_map_bag.write("path", ros::Time::now(), globalmap_msg);
    // sim_map_bag.close();

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
    
    // PathPlanning path_planning(global_map, start, goal, param);
    // path_planning.processing();
    // nav_msgs::Path path = path_planning.getPath();

    // rosbag::Bag bag;
    // bag.open("/home/lichao/catkin_pathplanning/src/path_planning/bag/path.bag", rosbag::bagmode::Write);
    // bag.write("path", ros::Time::now(), path);
    // bag.close();

    LOG(INFO)<<"PUB";
    // ros::Rate loop_rate(0.5);
    // while (ros::ok())
    // {
    //     // grid_map::GridMap map = path_planning.getMap();
    //     // grid_map_msgs::GridMap obstacle_goal_msg;
    //     // grid_map::GridMapRosConverter::toMessage(map, obstacle_goal_msg);
    //     // obstacle_goal_msg.info.header.frame_id = "map";
    //     map_pub.publish(globalmap_msg);
    //     // path.header.frame_id = "map";
    //     // path_pub.publish(path);
    //     ros::spinOnce();
    //     loop_rate.sleep();
    // }
    
    sleep(3);
    // 下面是通过rviz给定起点和终点
    ros::Rate loop_rate(0.4);
    while (ros::ok())
    {
        map_pub.publish(globalmap_msg);

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
        if (!(start_get && goal_get))
        {
            LOG(INFO)<<"wait start and goal";
            loop_rate.sleep();
            continue;
        }
        else
        {
            LOG(INFO)<<"initial path planning";
            LOG(INFO)<<start.transpose();
            LOG(INFO)<<goal.transpose();
            PathPlanning path_planning(global_map, start, goal, param);
            path_planning.processing();
            nav_msgs::Path path = path_planning.getPath();
            // path_planning.constructPlaneAwareMap();
            // path_planning.constructObstacleLayer(6);
            // path_planning.computeRep();
            path.header.frame_id = "map";
            path_pub.publish(path);
            start_get = false;
            goal_get = false;
        }
#endif
    }
    

    // PathPlanning path_planning(nh, map, support_area);
    // path_planning.constructPlaneAwareMap();
    // path_planning.constructObstacleLayer(6);
    // cv::imshow("obstacle_layer", path_planning.getObstacleLayer());
    // cv::waitKey(0);
    // path_planning.testisFeasible();
    // path_planning.checkFeasibleDirect();

    // cv::imshow("getNanRegion", path_planning.getNanRegion());
    // cv::waitKey(0);

    // cv::imshow("getFullRegion", path_planning.getFullRegion());
    // cv::waitKey(0);

    // cv::imshow("getCheckMat", path_planning.getCheckMat());
    // cv::waitKey(0);

    // path_planning.computeObstacles();
    // path_planning.clustering();
    // path_planning.clusterFeasibleRegions();
    LOG(INFO)<<"OUT";
    sleep(5);
    return 0;
}