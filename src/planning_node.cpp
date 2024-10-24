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

    ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, StartCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, GoalCallback);
    ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>("global_map", 1);

    grid_map::GridMap map;
    grid_map::Length length(10, 6);
    grid_map::Position position(5, 0);
    map.setGeometry(length, 0.02, position);

    map.add("elevation", 0.0);

    grid_map::Position stair_s(8, 2.8);
    grid_map::Position stair_e(6, 1);
    double step_w = 0.5;
    int index = 1;
    for (double s = stair_e.x(); s < stair_s.x(); s+=step_w)
    {
        grid_map::Position step_s(s, stair_s.y());
        grid_map::Position step_e(s+step_w, stair_e.y());
        double step_elevation = index * 0.1;
        index++;
        grid_map::Index step_s_index;
        grid_map::Size step_size(step_w/map.getResolution() + 2, 1.5/map.getResolution());
        if (map.getIndex(step_s, step_s_index))
        {
            for (grid_map::SubmapIterator iter(map, step_s_index, step_size); !iter.isPastEnd(); ++iter)
            {
                grid_map::Index iter_index(*iter);
                map["elevation"](iter_index.x(), iter_index.y()) = step_elevation;
            }
        }
    }

    grid_map::Position obstacle_s(3, -0.5);
    grid_map::Index obstacle_s_index;
    if (map.getIndex(obstacle_s, obstacle_s_index))
    {
        grid_map::Size obstacle_size(1/map.getResolution(), 0.4/map.getResolution());
        for (grid_map::SubmapIterator iter(map, obstacle_s_index, obstacle_size); !iter.isPastEnd(); ++iter)
        {
            grid_map::Index iter_index(*iter);
            map["elevation"](iter_index.x(), iter_index.y()) = 1.0;
        }
    }

    grid_map_msgs::GridMap globalmap_msg;
    grid_map::GridMapRosConverter::toMessage(map, globalmap_msg);
    globalmap_msg.info.header.frame_id = "map";

    SupportArea support_area(0.15, 0.1, 0.165, 0.165);

    // 自己先给定终点
    start[0] = 3.26811;
    start[1] = -2.39386;
    start[2] = 0.289504;

    goal[0] = 7.21435;
    goal[1] = 2.10046;
    goal[2] = 0;

    PathPlanning path_planning(nh, map, support_area, start, goal);
    path_planning.constructPlaneAwareMap();
    path_planning.constructObstacleLayer(6);
    path_planning.constructFullFeasibleRegion(0.55);
    path_planning.computeRep();
    path_planning.constructGoalVortexRegion();
    path_planning.plan();
    LOG(INFO)<<"PUB";
    ros::Rate loop_rate(0.5);
    while (ros::ok())
    {
        grid_map::GridMap map = path_planning.getMap();
        grid_map_msgs::GridMap obstacle_goal_msg;
        grid_map::GridMapRosConverter::toMessage(map, obstacle_goal_msg);
        obstacle_goal_msg.info.header.frame_id = "map";
        map_pub.publish(obstacle_goal_msg);
        ros::spinOnce();
    }
    

    // 下面是通过rviz给定起点和终点
    // ros::Rate loop_rate(0.4);
    // while (ros::ok())
    // {
    //     map_pub.publish(globalmap_msg);
    //     ros::spinOnce();
    //     if (!(start_get && goal_get))
    //     {
    //         LOG(INFO)<<"wait start and goal";
    //         loop_rate.sleep();
    //         continue;
    //     }
    //     else
    //     {
    //         LOG(INFO)<<"initial path planning";
    //         LOG(INFO)<<start.transpose();
    //         LOG(INFO)<<goal.transpose();
    //         PathPlanning path_planning(nh, map, support_area, start, goal);
    //         path_planning.constructPlaneAwareMap();
    //         path_planning.constructObstacleLayer(6);
    //         path_planning.computeRep();
    //         start_get = false;
    //         goal_get = false;
    //     }
    // }
    

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
    return 0;
}