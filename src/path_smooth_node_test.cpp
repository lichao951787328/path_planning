#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/SubmapIterator.hpp>
#include <path_planning/path_smooth.h>
#include <glog/logging.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "read_path_from_bag");
    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>("map", 1);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("smoothed_path", 1);
    ros::Publisher raw_path_pub = nh.advertise<nav_msgs::Path>("raw_path", 1);

    rosbag::Bag bag;
    bag.open("/home/lichao/catkin_pathplanning/src/path_planning/bag/path.bag", rosbag::bagmode::Read);

    // 使用 topic 名称来过滤消息
    std::vector<std::string> topics;
    topics.push_back("path");

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    nav_msgs::Path path;
    // 遍历 bag 文件中的消息
    for (const rosbag::MessageInstance& msg : view) 
    {
        nav_msgs::Path::ConstPtr path_msg = msg.instantiate<nav_msgs::Path>();
        if (path_msg != nullptr) 
        {
            path = *path_msg;
            ROS_INFO("Read path with %lu poses", path_msg->poses.size());
            for (const auto& pose : path_msg->poses) 
            {
                ROS_INFO("Pose position: x=%f, y=%f, z=%f",
                         pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            }
        }
    }
    bag.close();
    // 使用梯度进行平滑
    // double alpha, wObstacle, wCurvature, wSmoothness, vorObsDMax, obsDMax, kappaMax;
    // int max_iterations;
    // nh.getParam("alpha", alpha);
    // nh.getParam("wObstacle", wObstacle);
    // nh.getParam("wCurvature", wCurvature);
    // nh.getParam("wSmoothness", wSmoothness);
    // nh.getParam("vorObsDMax", vorObsDMax);
    // nh.getParam("obsDMax", obsDMax);
    // nh.getParam("kappaMax", kappaMax);
    // nh.getParam("max_iterations", max_iterations);

    // LOG(INFO)<<alpha<<" "<<wObstacle<<" "<<wCurvature<<" "<<wSmoothness<<" "<<vorObsDMax<<" "<<obsDMax<<" "<<kappaMax<<" "<<max_iterations;
    path_smooth::pathSmoothParams params;
    // params.alpha = alpha;
    // params.wObstacle = wObstacle;
    // params.wCurvature = wCurvature;
    // params.wSmoothness = wSmoothness;
    // params.vorObsDMax = vorObsDMax;
    // params.obsDMax = obsDMax;
    // params.kappaMax = kappaMax;
    // params.max_iterations = max_iterations;

    int Bspline_degree;
    int Bspline_numPoints;
    nh.getParam("Bspline_degree", Bspline_degree);
    nh.getParam("Bspline_numPoints", Bspline_numPoints);
    path_smooth::BsplineParams Bspline_params;
    Bspline_params.degree = Bspline_degree;
    Bspline_params.numPoints = Bspline_numPoints;
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

    // load image from /home/lichao/catkin_pathplanning/src/path_planning/bag
    cv::Mat obstacle_image = cv::imread("/home/lichao/catkin_pathplanning/src/path_planning/bag/obstacle_map.png", CV_8UC1);
    
    cv::Mat check_image = cv::imread("/home/lichao/catkin_pathplanning/src/path_planning/bag/check_image.png", CV_8UC1);
    LOG(INFO)<<"GET PARAM";
    path_smooth::pathSmoother path_smoother(path, obstacle_image, check_image, map, params, Bspline_params);
    LOG(INFO)<<"initial";
    // path_smoother.smoothPath();
    path_smoother.smoothBSPline();
    nav_msgs::Path smoothed_path = path_smoother.getSmoothedPath();
    nav_msgs::Path raw_path = path_smoother.getRawPath();
    grid_map_msgs::GridMap map_msg;
    grid_map::GridMapRosConverter::toMessage(map, map_msg);
    map_msg.info.header.frame_id = "map";
    smoothed_path.header.frame_id = "map";
    raw_path.header.frame_id = "map";
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        map_pub.publish(map_msg);
        path_pub.publish(smoothed_path);
        raw_path_pub.publish(raw_path);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    LOG(INFO)<<"OUTPUT";
    return 0;
}
