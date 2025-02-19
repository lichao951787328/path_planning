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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <glog/logging.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
// 实际使用时，使用定位+订阅全局地图+订阅终点
// 起点使用tf树
class PathPlanningPipline
{
private:
    ros::NodeHandle nh;
    ros::Subscriber goal_sub;
    ros::Subscriber globalmap_sub;
    ros::Publisher path_pub;
    ros::Publisher path_marker_pub;

    grid_map::GridMap globalmap;
    bool get_globalmap = false;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    planningParam param;

    string pointcloud_map_frame_id;
    string globalmap_frame_id;
    string lidar_frame_id;
    string globalmap_topic;

public:
    PathPlanningPipline(ros::NodeHandle & n);
    void GoalCallback(const geometry_msgs::PoseStamped::Ptr msg);
    void globalmapCallback(const grid_map_msgs::GridMap::ConstPtr globalmap_msg);
    vector<cv::Point> path2CvPoint(grid_map::GridMap & map, nav_msgs::Path & path);
    void map2ObstacleLayer(grid_map::GridMap & map, string heightLayer);
    visualization_msgs::Marker convertPath2visualmsgs(nav_msgs::Path & path);
    ~PathPlanningPipline();
};

PathPlanningPipline::PathPlanningPipline(ros::NodeHandle & n):nh(n)
{
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

    nh.getParam("pointcloud_map_frame_id", pointcloud_map_frame_id);
    nh.getParam("globalmap_frame_id", globalmap_frame_id);
    nh.getParam("lidar_frame_id", lidar_frame_id);
    nh.getParam("globalmap_topic", globalmap_topic);

    param.support_area.Up = support_area_up;
    param.support_area.Button = support_area_button;
    param.support_area.Left = support_area_left;
    param.support_area.Right = support_area_right;
    
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
    
    
    param.goal_obstacle_cof = goal_obstacle_cof;
    param.gen_obstacle_cof = gen_obstacle_cof;
    param.d_g_att = d_g_att;
    param.d_g_att_cof = d_g_att_cof;
    param.support_ratio = support_ratio;
    param.step = step;

    goal_sub = nh.subscribe("/move_base_simple/goal", 1, &PathPlanningPipline::GoalCallback, this);
    globalmap_sub = nh.subscribe(globalmap_topic, 1, &PathPlanningPipline::globalmapCallback, this);

    path_pub = nh.advertise<nav_msgs::Path>("path", 1);
    path_marker_pub = nh.advertise<visualization_msgs::Marker>("path_marker", 1);

    visualization_msgs::Marker convertPath2visualmsgs(nav_msgs::Path & path);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void PathPlanningPipline::globalmapCallback(const grid_map_msgs::GridMap::ConstPtr globalmap_msg)
{
    if (!get_globalmap)
    {
        LOG(INFO)<<"get global map";
        grid_map::GridMapRosConverter::fromMessage(*globalmap_msg, globalmap);
        get_globalmap = true;
        param.resolution = globalmap.getResolution();
        param.computeRadius();
        param.computeSupportNumTh();
    }
}


void PathPlanningPipline::map2ObstacleLayer(grid_map::GridMap & map, string heightLayer)
{
    // 把高程图转成障碍图，注意是根据高度梯度，只有在高度梯度下降达到某个程度的点才会被膨胀，生成障碍区域。
    const float minValue = map.get("elevation").minCoeffOfFinites();
    const float maxValue = map.get("elevation").maxCoeffOfFinites();

    cv::Mat heightImage;
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(map, "elevation", CV_8UC1, minValue, maxValue, heightImage);

    // 计算水平和垂直梯度
    cv::Mat grad_x, grad_y;
    cv::Sobel(heightImage, grad_x, CV_32F, 1, 0, 3);
    cv::Sobel(heightImage, grad_y, CV_32F, 0, 1, 3);

    // 计算梯度幅值
    cv::Mat grad_magnitude;
    cv::magnitude(grad_x, grad_y, grad_magnitude);

    // 计算梯度方向（以角度表示）
    cv::Mat grad_angle;
    cv::phase(grad_x, grad_y, grad_angle, true);

    // 创建掩膜，用于膨胀
    cv::Mat dilate_mask = cv::Mat::zeros(heightImage.size(), CV_8U);

    // 遍历图像，根据梯度方向来选择性膨胀
    for (int i = 1; i < heightImage.rows - 1; ++i) 
    {
        for (int j = 1; j < heightImage.cols - 1; ++j) 
        {
            // 获取当前像素的梯度方向
            float angle = grad_angle.at<float>(i, j);

            // 如果梯度方向指向下降的方向，则进行膨胀
            // 这里的判断依据需要根据实际场景调整，比如检查角度的范围
            if (angle >= 45 && angle <= 135) 
            {
                dilate_mask.at<uchar>(i, j) = 255;  // 例如设置为255代表膨胀
            }
        }
    }
}

vector<cv::Point> PathPlanningPipline::path2CvPoint(grid_map::GridMap & map, nav_msgs::Path & path)
{
    vector<cv::Point> cv_points;
    for (auto & point : path.poses)
    {
        grid_map::Position position(point.pose.position.x, point.pose.position.y);
        grid_map::Index index;
        if (map.getIndex(position, index))
        {
            cv_points.emplace_back(index.x(), index.y());
        }
    }
    return cv_points;
}

// 起点终点都是相对于点云地图的，要转到全局地形地图坐标系下
void PathPlanningPipline::GoalCallback(const geometry_msgs::PoseStamped::Ptr msg)
{
    if (!get_globalmap)
    {
        LOG(WARNING)<<"not globalmap, return";
        return;
    }
    
    Eigen::Vector3d start, goal;
    // 获取终点在全局地形地图坐标系下的值
    geometry_msgs::TransformStamped transformStamped_globalmap_map;
    try
    {
        transformStamped_globalmap_map = tf_buffer_->lookupTransform(globalmap_frame_id, pointcloud_map_frame_id, ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        std::cerr << ex.what() << '\n';
    }
    geometry_msgs::PoseStamped goal_globalmap;
    tf2::doTransform(*msg, goal_globalmap, transformStamped_globalmap_map);
    // 提取 x 和 y 坐标
    double x = goal_globalmap.pose.position.x;
    double y = goal_globalmap.pose.position.y;

    // 提取四元数
    tf2::Quaternion quat;
    quat.setX(goal_globalmap.pose.orientation.x);
    quat.setY(goal_globalmap.pose.orientation.y);
    quat.setZ(goal_globalmap.pose.orientation.z);
    quat.setW(goal_globalmap.pose.orientation.w);

    // 将四元数转换为欧拉角
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    goal[0] = x;
    goal[1] = y;
    goal[2] = yaw;
    // 输出 x, y, 和 yaw
    ROS_INFO("goal: x: %f, y: %f, yaw: %f degree", x, y, yaw * 57.3);

    geometry_msgs::TransformStamped transformStamped_globalmap_velodyne;
    try
    {
        transformStamped_globalmap_velodyne = tf_buffer_->lookupTransform(globalmap_frame_id, lidar_frame_id, ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        std::cerr << ex.what() << '\n';
    }
    // tf2::Transform T_base_3dworld;
    // tf2::fromMsg(transformStamped_globalmap_velodyne.transform, T_base_3dworld);

    double x_start = transformStamped_globalmap_velodyne.transform.translation.x;
    double y_start = transformStamped_globalmap_velodyne.transform.translation.y;
    tf2::Quaternion quat_start;
    quat_start.setX(transformStamped_globalmap_velodyne.transform.rotation.x);
    quat_start.setY(transformStamped_globalmap_velodyne.transform.rotation.y);
    quat_start.setZ(transformStamped_globalmap_velodyne.transform.rotation.z);
    quat_start.setW(transformStamped_globalmap_velodyne.transform.rotation.w);

    double roll_start, pitch_start, yaw_start;
    tf2::Matrix3x3(quat_start).getRPY(roll_start, pitch_start, yaw_start);
    start[0] = x_start;
    start[1] = y_start;
    start[2] = yaw_start;
    ROS_INFO("start: x: %f, y: %f, yaw: %f degree", x_start, y_start, yaw_start * 57.3);
    PathPlanning path_planning(globalmap, start, goal, param);
    path_planning.processing();
    nav_msgs::Path path = path_planning.getPath();

    // 规划的path转成cv点，传入顺滑器中


    path.header.frame_id = globalmap_frame_id;
    path_pub.publish(path);
}

visualization_msgs::Marker PathPlanningPipline::convertPath2visualmsgs(nav_msgs::Path & path)
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


PathPlanningPipline::~PathPlanningPipline()
{
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

    PathPlanningPipline path_planning_pipline(nh);
    ros::spin();
    LOG(INFO)<<"OUT";
    return 0;
}