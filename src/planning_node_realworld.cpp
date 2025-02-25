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
#include <boost/foreach.hpp>  // 用于遍历
#include <path_smoother/path_smoother.h>

// 是否使用rviz来获取起点和终点
// #define SET_BY_RVIZ
// visualization_msgs::Marker convertPath2visualmsgs(nav_msgs::Path & path)
// {
//     visualization_msgs::Marker path_marker;
//     path_marker.header.frame_id = "map";
//     path_marker.header.stamp = ros::Time::now();
//     path_marker.ns = "path";
//     path_marker.id = 0;
//     path_marker.type = visualization_msgs::Marker::LINE_STRIP;
//     path_marker.action = visualization_msgs::Marker::ADD;
//     path_marker.scale.x = 0.4;  // 设置线条宽度
//     path_marker.color.a = 1.0;  // 设置透明度
//     path_marker.color.r = 0.0;  // 设置红色
//     path_marker.color.g = 1.0;  // 设置绿色
//     path_marker.color.b = 0.0;  // 设置蓝色
//     for (int i = 0; i < path.poses.size() - 1; i++)
//     {
//         // 添加路径点
//         geometry_msgs::Point p1;
//         p1.x = path.poses.at(i).pose.position.x;
//         p1.y = path.poses.at(i).pose.position.y;
//         p1.z = path.poses.at(i).pose.position.z;
//         path_marker.points.push_back(p1);

//         geometry_msgs::Point p2;
//         p2.x = path.poses.at(i+1).pose.position.x;
//         p2.y = path.poses.at(i+1).pose.position.y;
//         p2.z = path.poses.at(i+1).pose.position.z;
//         path_marker.points.push_back(p2);
//     }
    
//     return path_marker;
// }


class planningNode
{
private:
    ros::NodeHandle nh;
    ros::Subscriber map_sub;
    ros::Subscriber start_sub, goal_sub;
    ros::Publisher path_pub;
    ros::Publisher smooth_path_pub;

    ros::Publisher path_marker_pub;
    ros::Publisher smooth_path_marker_pub;

    grid_map::GridMap map;
    Eigen::Vector3d start, goal;
    bool get_map = false;
    bool get_start = false;
    bool get_goal = false;

    planningParam param;
public:
    planningNode(ros::NodeHandle & n):nh(n)
    {
        map_sub = nh.subscribe("/global_map", 1, &planningNode::MapCallback, this);
        start_sub = nh.subscribe("/initialpose", 1, &planningNode::StartCallback, this);
        goal_sub = nh.subscribe("/move_base_simple/goal", 1, &planningNode::GoalCallback, this);
        path_pub = nh.advertise<nav_msgs::Path>("/path", 1);
        smooth_path_pub = nh.advertise<nav_msgs::Path>("/smooth_path", 1);

        path_marker_pub = nh.advertise<visualization_msgs::Marker>("/path_marker", 1);
        smooth_path_marker_pub = nh.advertise<visualization_msgs::Marker>("/smooth_path_marker", 1);
        nh.getParam("support_area_up", param.support_area.Up);
        nh.getParam("support_area_button", param.support_area.Button);
        nh.getParam("support_area_left", param.support_area.Left);
        nh.getParam("support_area_right", param.support_area.Right);
        
        nh.getParam("obstacle_inflation_radius", param.obstacle_inflation_radius);
        nh.getParam("obstacle_length", param.obstacle_length);
        nh.getParam("obstacle_rad", param.obstacle_rad);
        nh.getParam("safe_region_radius", param.safe_region_radius);

        // 终点障碍
        nh.getParam("d_safe_goal", param.d_safe_goal);
        nh.getParam("d_vort_goal", param.d_vort_goal);
        nh.getParam("d_noinflu_offset_goal", param.d_noinflu_offset_goal);

        // 一般障碍
        nh.getParam("d_safe_gen", param.d_safe_gen);
        nh.getParam("d_vort_gen", param.d_vort_gen);
        nh.getParam("d_noinflu_offset_gen", param.d_noinflu_offset_gen);

        nh.getParam("goal_obstacle_cof", param.goal_obstacle_cof);
        nh.getParam("gen_obstacle_cof", param.gen_obstacle_cof);
        nh.getParam("d_g_att", param.d_g_att);
        nh.getParam("d_g_att_cof", param.d_g_att_cof);
        nh.getParam("support_ratio", param.support_ratio);
        nh.getParam("step", param.step);
    }

    visualization_msgs::Marker convertPath2visualmsgs(nav_msgs::Path & path, Eigen::Vector3d color)
    {
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = ros::Time::now();
        path_marker.ns = "path";
        path_marker.id = 0;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.scale.x = 0.05;
        path_marker.color.a = 1;
        path_marker.color.r = color.x();
        path_marker.color.g = color.y();
        path_marker.color.b = color.z();
        for (int i = 0; i < path.poses.size() - 1; i++)
        {
            // 添加路径点
            geometry_msgs::Point p1;
            p1.x = path.poses.at(i).pose.position.x;
            p1.y = path.poses.at(i).pose.position.y;
            p1.z = path.poses.at(i).pose.position.z+0.04;
            path_marker.points.push_back(p1);

        }
        return path_marker;
    }

    void MapCallback(const grid_map_msgs::GridMap::ConstPtr & msg)
    {
        LOG(INFO)<<"enter map callback";
        if (get_map == false)
        {
            LOG(INFO)<<"get map";
            grid_map::GridMapRosConverter::fromMessage(*msg, map);
            get_map = true;
            param.resolution = map.getResolution();
            param.computeRadius();
            param.computeSupportNumTh();
#ifndef SET_BY_RVIZ
            // 在此设置起点和终点
            // 第一组
            // start = Eigen::Vector3d(0.0556173, -4.03035, 1.17601);
            // goal = Eigen::Vector3d(-0.741608, 3.43, 3.01851);

            // 第二组
            // start = Eigen::Vector3d(0.202, -3.89, 1.178);
            // goal = Eigen::Vector3d(1.725, 4.02225, 1.20682);

            // 第三组,放弃
            start = Eigen::Vector3d(4.0266, -4.05476, 1.79661);
            goal = Eigen::Vector3d(3.41288, 4.50446, 0.429762);

            // 第四组
            // start = Eigen::Vector3d(-2.12158, -1.11767, -0.143738);
            // goal = Eigen::Vector3d(5.64858, 1.85231, 1.48214);

            // 第五组
            start = Eigen::Vector3d(-1.57361, -3.89037, 0.0837733);
            goal = Eigen::Vector3d(5.75817, 1.69888, 1.41214);

            // 第六组 放弃

            // 第七组 
            start = Eigen::Vector3d(-1.93527, 1.68792, -0.110657);
            goal = Eigen::Vector3d(5.98832, -3.32049, -0.278299);

            // 第八组 
            start = Eigen::Vector3d(2.91971, -3.53968, 0.341616);
            goal = Eigen::Vector3d(5.46227, 1.78655, 1.58804);

            // 第九组 
            start = Eigen::Vector3d(0.947031, -3.8575, 0.563316);
            goal = Eigen::Vector3d(5.24308, 1.73175, 1.46481);

            // 第十组 
            start = Eigen::Vector3d(5.23212, 2.57562, 1.81097);
            goal = Eigen::Vector3d(-0.992767, 3.33181, -3.06899);

            PathPlanning path_planning(map, start, goal, param);
            if (path_planning.processing())
            {
                nav_msgs::Path path = path_planning.getPath();
                path.header.frame_id = "map";
                path_pub.publish(path);
                // sleep(2);
                cv::Mat obstacle_layer = path_planning.getObstacleVoronoi();
                cv::imshow("obstacle_layer-----", obstacle_layer);
                cv::waitKey(0);
                std::vector<cv::Point> points = path2CvPoint(path);
                cv::bitwise_not(obstacle_layer, obstacle_layer);
                cv::Mat obstacle_layer_8UC3;
                cv::cvtColor(obstacle_layer, obstacle_layer_8UC3, cv::COLOR_GRAY2BGR);
                for (int i = 0; i < obstacle_layer_8UC3.rows; i++)
                {
                    for (int j = 0; j < obstacle_layer_8UC3.cols; j++)
                    {
                        if (obstacle_layer_8UC3.at<cv::Vec3b>(i, j) == cv::Vec3b(0, 0, 0))
                        {
                            obstacle_layer_8UC3.at<cv::Vec3b>(i, j) = cv::Vec3b(19, 69, 139); // Dark brown-yellow color
                        }
                    }
                }
                cv::imshow("obstacle_layer_8UC3", obstacle_layer_8UC3);
                cv::waitKey(0);



                PathSmoother path_smoother(obstacle_layer, points);
                path_smoother.smoothPath();
                
                // 这是在图像坐标系下的角度和坐标，要转成地图坐标系下
                std::vector<Pose2d> smooth_path_3d = path_smoother.getSmoothedPath();
                nav_msgs::Path smooth_path;
                for (int i = 0; i < smooth_path_3d.size(); i++)
                {
                    Pose2d pose = smooth_path_3d[i];
                    geometry_msgs::PoseStamped pose_stamped;

                    grid_map::Index index(pose.y(), pose.x());
                    grid_map::Position3 position3;
                    if (map.getPosition3("elevation", index, position3))
                    {
                        pose_stamped.pose.position.x = position3.x();
                        pose_stamped.pose.position.y = position3.y();
                        pose_stamped.pose.position.z = position3.z();
                    }

                    Eigen::AngleAxisd rotation_vector(pose.theta(), Eigen::Vector3d::UnitZ());
                    Eigen::Quaterniond quaternion(rotation_vector);
                    pose_stamped.pose.orientation.x = quaternion.x();
                    pose_stamped.pose.orientation.y = quaternion.y();
                    pose_stamped.pose.orientation.z = quaternion.z();
                    pose_stamped.pose.orientation.w = quaternion.w();
                    
                    smooth_path.poses.emplace_back(pose_stamped);
                }
                smooth_path.header.frame_id = "map";
                smooth_path_pub.publish(smooth_path);

                visualization_msgs::Marker path_marker = convertPath2visualmsgs(path, Eigen::Vector3d(0, 0, 1));
                path_marker_pub.publish(path_marker);
                visualization_msgs::Marker smooth_path_marker = convertPath2visualmsgs(smooth_path, Eigen::Vector3d(0, 1, 0));
                smooth_path_marker_pub.publish(smooth_path_marker);

                // Calculate the total distance and angle change for the original path
                double total_distance = 0.0;
                double total_angle_change = 0.0;
                for (size_t i = 0; i < path.poses.size() - 1; i++)
                {
                    double dx = path.poses.at(i+1).pose.position.x - path.poses.at(i).pose.position.x;
                    double dy = path.poses.at(i+1).pose.position.y - path.poses.at(i).pose.position.y;
                    total_distance += std::sqrt(dx * dx + dy * dy);

                    if (i < path.poses.size() - 2)
                    {
                        double dx_next = path.poses.at(i+2).pose.position.x - path.poses.at(i+1).pose.position.x;
                        double dy_next = path.poses.at(i+2).pose.position.y - path.poses.at(i+1).pose.position.y;
                        double angle1 = std::atan2(dy, dx);
                        double angle2 = std::atan2(dy_next, dx_next);
                        double angle_change = std::abs(angle2 - angle1);

                        // Ensure the angle change is consistent with the direction of the path
                        if (angle_change > M_PI)
                        {
                            angle_change = 2 * M_PI - angle_change;
                        }
                        total_angle_change += angle_change;
                    }
                }

                // Calculate the total distance and angle change for the smoothed path
                double smooth_total_distance = 0.0;
                double smooth_total_angle_change = 0.0;
                for (size_t i = 0; i < smooth_path.poses.size() - 1; i++)
                {
                    double dx = smooth_path.poses.at(i+1).pose.position.x - smooth_path.poses.at(i).pose.position.x;
                    double dy = smooth_path.poses.at(i+1).pose.position.y - smooth_path.poses.at(i).pose.position.y;
                    smooth_total_distance += std::sqrt(dx * dx + dy * dy);

                    if (i < smooth_path.poses.size() - 2)
                    {
                        double dx_next = smooth_path.poses.at(i+2).pose.position.x - smooth_path.poses.at(i+1).pose.position.x;
                        double dy_next = smooth_path.poses.at(i+2).pose.position.y - smooth_path.poses.at(i+1).pose.position.y;
                        double angle1 = std::atan2(dy, dx);
                        double angle2 = std::atan2(dy_next, dx_next);
                        double angle_change = std::abs(angle2 - angle1);

                        // Ensure the angle change is consistent with the direction of the path
                        if (angle_change > M_PI)
                        {
                            angle_change = 2 * M_PI - angle_change;
                        }
                        smooth_total_angle_change += angle_change;
                    }
                }

                ROS_INFO("Original Path: Total Distance = %f, Total Angle Change = %f", total_distance, total_angle_change);
                ROS_INFO("Smoothed Path: Total Distance = %f, Total Angle Change = %f", smooth_total_distance, smooth_total_angle_change);


                // Draw the original path in blue
                for (size_t i = 0; i < points.size() - 1; i++)
                {
                    cv::line(obstacle_layer_8UC3, points[i], points[i + 1], cv::Scalar(255, 0, 0), 2); // Blue color
                }

                // Draw the smoothed path in green
                for (size_t i = 0; i < smooth_path_3d.size() - 1; i++)
                {
                    cv::Point point1(smooth_path_3d[i].x(), smooth_path_3d[i].y());
                    cv::Point point2(smooth_path_3d[i + 1].x(), smooth_path_3d[i + 1].y());
                    cv::line(obstacle_layer_8UC3, point1, point2, cv::Scalar(0, 255, 0), 2); // Green color
                }

                cv::imshow("obstacle_layer_with_paths", obstacle_layer_8UC3);
                cv::imwrite("/home/lichao/catkin_pathplanning/src/path_planning/experiment_data/case10/obstacle_layer_with_paths.jpg", obstacle_layer_8UC3);
                cv::waitKey(0);

                // 
            }
            get_map = false;
#endif
        }
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
        get_goal = true;
#ifdef SET_BY_RVIZ
        if (get_map && get_start && get_goal)
        {
            PathPlanning path_planning(map, start, goal, param);
            if (path_planning.processing())
            {
                nav_msgs::Path path = path_planning.getPath();
                path.header.frame_id = "map";
                path_pub.publish(path);

                // 把生成的障碍图进行膨胀，对终点障碍区域和开口区域不作为障碍区域，此时的开口区域为可通行的区域

                cv::Mat obstacle_layer = path_planning.getObstacleVoronoi();
                std::vector<cv::Point> points = path2CvPoint(path);
                cv::bitwise_not(obstacle_layer, obstacle_layer);
                PathSmoother path_smoother(obstacle_layer, points);
                path_smoother.smoothPath();
                
                // 这是在图像坐标系下的角度和坐标，要转成地图坐标系下
                std::vector<Pose2d> smooth_path_3d = path_smoother.getSmoothedPath();
                nav_msgs::Path smooth_path;
                for (int i = 0; i < smooth_path_3d.size(); i++)
                {
                    Pose2d pose = smooth_path_3d[i];
                    geometry_msgs::PoseStamped pose_stamped;

                    grid_map::Index index(pose.y(), pose.x());
                    grid_map::Position3 position3;
                    if (map.getPosition3("elevation", index, position3))
                    {
                        pose_stamped.pose.position.x = position3.x();
                        pose_stamped.pose.position.y = position3.y();
                        pose_stamped.pose.position.z = position3.z();
                    }

                    Eigen::AngleAxisd rotation_vector(pose.theta(), Eigen::Vector3d::UnitZ());
                    Eigen::Quaterniond quaternion(rotation_vector);
                    pose_stamped.pose.orientation.x = quaternion.x();
                    pose_stamped.pose.orientation.y = quaternion.y();
                    pose_stamped.pose.orientation.z = quaternion.z();
                    pose_stamped.pose.orientation.w = quaternion.w();
                    
                    smooth_path.poses.emplace_back(pose_stamped);
                }
                smooth_path.header.frame_id = "map";
                smooth_path_pub.publish(smooth_path);
            }
            get_start = false;
            get_goal = false;
        }
#endif
    }

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
        get_start = true;
    }
    
    vector<cv::Point> path2CvPoint(nav_msgs::Path & path)
    {
        vector<cv::Point> cv_points;
        for (auto & point : path.poses)
        {
            grid_map::Position position(point.pose.position.x, point.pose.position.y);
            grid_map::Index index;
            if (map.getIndex(position, index))
            {
                cv_points.emplace_back(index.y(), index.x());
            }
        }
        return cv_points;
    }
    ~planningNode()
    {

    }
};



int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]); 
    google::InstallFailureSignalHandler();
    // google::SetCommandLineOptionWithMode("FLAGS_minloglevel", "2");
    FLAGS_minloglevel = 0;
    FLAGS_colorlogtostderr = true;
    FLAGS_alsologtostderr = true;
    ros::init(argc, argv, "planning");
    ros::NodeHandle n;

    planningNode planning_node(n);

    ros::spin();
    LOG(INFO)<<"OUT";
    return 0;
}