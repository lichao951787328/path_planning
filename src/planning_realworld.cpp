#include <path_planning/planning.h>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <Eigen/Dense>
#include <omp.h>
#include <chrono>
#include <matplotlibcpp.h>
#include <random>
#include <queue>
// #include <path_planning/math_helper.h>
#include <ros/package.h>
#include <map>
#include <limits>
#include <cstdlib>
#include <pcl/io/pcd_io.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <chrono>
void initial_package_path(string package_name, string & package_path)
{
  package_path = ros::package::getPath(package_name);
  // 检查是否成功找到包路径
  if (package_path.empty()) {
      std::cerr << "Error: Could not find package " << package_name << std::endl;
  }
  cout<<"package path: "<<package_path<<endl;
}


PathPlanning::PathPlanning(grid_map::GridMap & map_, Eigen::Vector3d & start, Eigen::Vector3d & goal, planningParam & param):map(map_), start_(start), goal_(goal),planning_param(param)
{
    initial_package_path("path_planning", package_path);
    LOG(INFO)<<"package path is: "<<package_path;
    pd.initial(package_path + "/config/plane_fitter_pcd.ini");

    LOG(INFO)<<start_.transpose();
    LOG(INFO)<<goal_.transpose();
    
    Node start_node;
    start_node.position = start_.head(2);
    start_node.ori = start_.z();
    startP = std::make_shared<Node>(start_node);
    
    if (!map.getIndex(goal_.head(2), goal_index_))
    {
        LOG(ERROR)<<"goal is out of map";
    }
    goalDirectInImage(goal_.z());
    LOG(INFO)<<"pixel direct: "<<ray_Dir;
    // Nan_feasible = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
#ifdef DEBUG
    // cv::imshow("Nan_feasible", Nan_feasible);
    // cv::waitKey(0);
#endif
    // Full_feasible = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    check_Mat = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);

    obstacle_layer_voronoi = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);

    obstacle_layer_voronoi_goal = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);

    hole_rect = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
#ifdef DEBUG
    // cv::imshow("Full_feasible", Full_feasible);
    // cv::waitKey(0);
#endif
    debug_image = cv::Mat(map.getSize().x(), map.getSize().y(), CV_8UC3, cv::Scalar(255, 255, 255));
}

pcl::PointCloud<pcl::PointXYZ> gridMap2Pointcloud(grid_map::GridMap & map)
{
    pcl::PointCloud<pcl::PointXYZ> pc;
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            grid_map::Index index(i, j);
            grid_map::Position3 p3;
            if (map.getPosition3("elevation", index, p3))
            {
                if (!std::isnan(p3.z()))
                {
                    pc.emplace_back(pcl::PointXYZ(p3.x(), p3.y(), p3.z()));
                }
                else
                {
                    pc.emplace_back(pcl::PointXYZ(NAN, NAN, NAN));
                }
            }
            else
            {
                pc.emplace_back(pcl::PointXYZ(NAN, NAN, NAN));
            }
        }
    }
    pc.width = map.getSize().y();
    pc.height = map.getSize().x();
    return pc;
}

// 实际使用时，可以先把地面滤除，
void PathPlanning::constructPlaneAwareMap()
{
    pcl::PointCloud<pcl::PointXYZ> pc = gridMap2Pointcloud(map);
    // pcl::io::savePCDFile(package_path + "/data/pc.pcd", pc);

    // 上面的检测方法里，平面信息和平面像素并不对应。后面针对像素对平面信息重新进行了检测
    pd.detect(pc);
    vector<ahc::PlaneSeg::Stats> statses(pd.planes.size());
    map.add("label");
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            bool flag = false;
            for (int label = 0; label < pd.planes.size(); label++)
            {
                if (pd.planes.at(label).at<uchar>(i, j) == 255)
                {
                    flag = true;
                    map["label"](i ,j) = label;
                    grid_map::Position3 p3;
                    if (map.getPosition3("elevation", grid_map::Index(i, j), p3))
                    {
                        // planes_info.at(label).stats.push(p3.x(), p3.y(), p3.z());
                        statses.at(label).push(p3.x(), p3.y(), p3.z());
                    }
                    break;
                }
            }
            if (!flag)
            {
                map["label"](i,j) = NAN;
            }
        }
    }
    pd.planes_info.clear();
    for (auto & stats : statses)
    {
        // plane_info.update();
        double center[3], normal[3];
        double mse, curvature;
        stats.compute(center, normal, mse, curvature);
        planeInfo pi;
        pi.center = Eigen::Vector3d(center[0], center[1], center[2]);
        if (normal[2] < 0)
        {
            pi.normal = Eigen::Vector3d(-normal[0], -normal[1], -normal[2]);
        }
        else
        {
            pi.normal = Eigen::Vector3d(normal[0], normal[1], normal[2]);
        }
        pd.planes_info.emplace_back(pi);
    }
    
    vector<cv::Scalar> Scalars;

    for (int i = 0; i < pd.planes.size(); i++)
    {
        int r = rand() % 256; // 随机红色分量
        int g = rand() % 256; // 随机绿色分量
        int b = rand() % 256;
        Scalars.emplace_back(cv::Scalar(b, g, r));
        debug_image.setTo(cv::Scalar(b, g, r), pd.planes.at(i));
#ifdef DEBUG
        LOG(INFO)<<"normal: "<<pd.planes_info.at(i).normal.transpose();
        LOG(INFO)<<"center: "<<pd.planes_info.at(i).center.transpose();

        cv::imshow("plane_map" + std::to_string(i), pd.planes.at(i));
        cv::waitKey(0);
#endif
    }
#ifdef DEBUG
    cv::Mat plane_map = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC3);
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        { 
            if (!std::isnan(map["label"](i, j)))
            {
                int index = static_cast<int>(map["label"](i, j));
                cv::Vec3b color(Scalars.at(index)(0), Scalars.at(index)(1), Scalars.at(index)(2));
                plane_map.at<cv::Vec3b>(i, j) = color;
            }
        }
    }
    cv::imshow("plane_map", plane_map);
    cv::waitKey(0);
#endif
    LOG(INFO)<<"construct plane-aware map finish";
}

bool PathPlanning::computePath(NodePtr goal)
{
    NodePtr iter = goal;
    vector<NodePtr> node_path;
    while (iter != nullptr)
    {
        node_path.emplace_back(iter);
        // LOG(INFO)<<"position: "<<iter->position.transpose();
        iter = iter->PreFootstepNode;
    }
    std::reverse(node_path.begin(), node_path.end());
    LOG(INFO)<<"iter number: "<<node_path.size() - 1;
    double length = 0;
    for (int i = 0; i < node_path.size() - 1; i++)
    {
        length += (node_path.at(i)->position - node_path.at(i + 1)->position).norm();
    }
    LOG(INFO)<<"length: "<<length;
    for (auto node : node_path)
    {
        grid_map::Index index;
        if (map.getIndex(node->position, index))
        {
            geometry_msgs::PoseStamped pose;
            grid_map::Position3 p3;
            map.getPosition3("elevation", index, p3);
            pose.pose.position.x = p3.x();
            pose.pose.position.y = p3.y();
            pose.pose.position.z = p3.z();
            tf2::Quaternion q;
            q.setRPY(0, 0, node->ori);
            geometry_msgs::Quaternion quaternion;
            quaternion.x = q.x();
            quaternion.y = q.y();
            quaternion.z = q.z();
            quaternion.w = q.w();
            pose.pose.orientation = quaternion;
            path.poses.push_back(pose);
        }
        else
        {
            return false;
        }
        
    }
    LOG(INFO)<<"path size: "<<path.poses.size();
    return true;
    
}


void PathPlanning::constructFullFeasibleRegion(double safe_length)
{
    LOG(INFO)<<"construct full feasible region";

    full_feasible_region = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    int radius = safe_length / planning_param.resolution;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(radius*2 + 1, radius*2 + 1), cv::Point(1, 1));
    for (int i = 0; i < pd.planes.size(); i++)
    {
        cv::Mat image = pd.planes.at(i).clone();
        cv::erode(image, image, element);
        full_feasible_region.setTo(255, image);
    }
    full_feasible_region.setTo(0, obstacle_layer);
#ifdef DEBUG
    cv::imshow("full_feasible_region", full_feasible_region);
    cv::waitKey(0);
#endif
    LOG(INFO)<<"OVER";
}

// 变更思路，不再只对平面的边远区域进行寻找确定障碍，而是除开平面较为内部的区域都寻找，确定障碍区域
void PathPlanning::constructObstacleLayer(int chect_scope)
{
    obstacle_layer = cv::Mat(map.getSize().x(), map.getSize().y(), CV_8UC1, cv::Scalar(255));
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            double max_height = - std::numeric_limits<double>::max();
            double min_height = std::numeric_limits<double>::max();
            for (int ky = -chect_scope / 2; ky <= chect_scope / 2; ++ky) 
            {
                for (int kx = -chect_scope / 2; kx <= chect_scope / 2; ++kx) 
                {
                    int nx = i + kx;
                    int ny = j + ky;
                    // 检查邻域内的点是否在图像边界内
                    if (nx >= 0 && ny >= 0 && nx < obstacle_layer.rows && ny < obstacle_layer.cols) 
                    {
                        grid_map::Position3 p3;
                        grid_map::Position3 plane_label;
                        if (map.getPosition3("elevation", grid_map::Index(nx, ny), p3))
                        {
                            // LOG(INFO)<<p3.transpose();
                            if (!std::isnan(p3.z()))
                            {
                                if (p3.z() > max_height)
                                {
                                    max_height = p3.z();
                                }
                                if (p3.z() < min_height)
                                {
                                    min_height = p3.z();
                                }
                            }
                        }
                    }
                }
            }
            // 最大迈不高度不超过0.15
            if (abs(max_height - min_height) < 0.23)
            {
                // LOG(INFO)<<max_height<<" "<<min_height;
                obstacle_layer.at<uchar>(i, j) = 0;
            }
        }
    }
#ifdef DEBUG
    cv::imshow("obstacle_layer", obstacle_layer);
    cv::waitKey(0);
#endif
    // 如果障碍区域构成回环，且回环面积小于某个阈值，可以将整个回环区域均设为障碍区域
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(obstacle_layer, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
    for (int i = 0; i < contours.size(); i++)
    {
        if (hierarchy[i][3] != -1) // 有内轮廓
        {
            cv::Mat temp_image = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            cv::drawContours(temp_image, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
            obstacle_layer.setTo(255, temp_image);
        }
    }
#ifdef DEBUG
    LOG(INFO)<<"construct obstacle map finish";
    cv::imshow("obstacle_layer", obstacle_layer);
    cv::waitKey(0);
#endif
    // 如果障碍凸区域之间有包含关系，则删除被包含的区域
    std::vector<std::vector<cv::Point>> contours_cut;
    std::vector<cv::Vec4i> hierarchy_cut;
    cv::findContours(obstacle_layer, contours_cut, hierarchy_cut, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < contours_cut.size(); ++i) {
        std::vector<cv::Point> hull_i;
        cv::convexHull(contours_cut[i], hull_i);

        for (size_t j = 0; j < contours_cut.size(); ++j) {
            if (i != j) {
                std::vector<cv::Point> hull_j;
                cv::convexHull(contours_cut[j], hull_j);

                cv::Mat mask_i = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
                cv::Mat mask_j = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);

                cv::drawContours(mask_i, std::vector<std::vector<cv::Point>>{hull_i}, -1, cv::Scalar(255), cv::FILLED);
                cv::drawContours(mask_j, std::vector<std::vector<cv::Point>>{hull_j}, -1, cv::Scalar(255), cv::FILLED);

                cv::Mat intersection;
                cv::bitwise_and(mask_i, mask_j, intersection);

                if (cv::countNonZero(intersection) == cv::countNonZero(mask_j)) {
                    cv::drawContours(obstacle_layer, std::vector<std::vector<cv::Point>>{hull_j}, -1, cv::Scalar(0), cv::FILLED);
                }
            }
        }
    }
#ifdef DEBUG
    cv::imshow("obstacle_layer", obstacle_layer);
    cv::waitKey(0);
#endif
    LOG(INFO)<<"over";
}


void PathPlanning::goalDirectInImage(double yaw)
{
    
    if (yaw >= -(3.14159)/2 - 0.001 && yaw <= -(3.14159)/2 + 0.001)
    {
        ray_Dir = cv::Point2f(1, 0);
    }
    else if (yaw >= (3.14159)/2 - 0.001 && yaw <= (3.14159)/2 + 0.01)
    {
        ray_Dir = cv::Point2f(-1, 0);
    }
    else
    {
        if (yaw > 0)
        {
            if (yaw < (3.14159)/2 - 0.001)
            {
                ray_Dir = cv::Point2f(-tan(yaw), - 1);
            }
            else
            {
                ray_Dir = cv::Point2f(-tan(yaw), 1);
            }
        }
        else
        {
            if (yaw > -(3.14159)/2 + 0.001)
            {
                ray_Dir = cv::Point2f(-tan(yaw), - 1);
            }
            else
            {
                ray_Dir = cv::Point2f(-tan(yaw), 1);
            }
        }
    }
    // return ray_Dir;
}

struct cv_point_dis
{
    int index;

    double dis;
};
struct cv_pointDisCompare
{
    bool operator()(const cv_point_dis & n1, const cv_point_dis & n2)
    {
        return n1.dis > n2.dis;
    }
};


// 注意要去除一些小的噪声障碍点，这些点是由于噪声引起，但是不能为此障碍附加势场
// 将每个障碍区域的轮廓进行光滑
void PathPlanning::computeObstacles()
{
    LOG(INFO)<<"computeObstacles";
    check_image = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> out_contours;
    std::vector<cv::Vec4i> out_hierarchy;
    cv::findContours(obstacle_layer, out_contours, out_hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
#ifdef DEBUG
    cv::imshow("obstacle_layer", obstacle_layer);
    cv::waitKey(0);
#endif


    LOG(INFO)<<"out_contours.size()"<<out_contours.size();

    for (auto & contour :  out_contours)
    {
        std::vector<cv::Point> hull;
        // 对每个轮廓计算凸包
        cv::convexHull(contour, hull);
        cv::Mat hull_region = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> hullContours = {hull};
        cv::drawContours(hull_region, hullContours, 0, cv::Scalar(255), cv::FILLED);
#ifdef DEBUG 
        cv::imshow("Hull Region", hull_region);
        cv::waitKey(0);
#endif
        // 使用pointPolygonTest判断点是否在凸包内
        double result = cv::pointPolygonTest(hull, cv::Point2f(goal_index_.y(), goal_index_.x()), false);
        if (result < 0) // 不在凸包内的一般障碍
        {
            cv::Mat obstacle = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            // 对一障碍，不要凸包也可以
            std::vector<std::vector<cv::Point>> Contours = {hull};
            cv::drawContours(obstacle, Contours, 0, cv::Scalar(255), cv::FILLED);
            obstacles.emplace_back(obstacle);

            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(planning_param.d_safe_rad_gen  * 2 + 1, planning_param.d_safe_rad_gen * 2 + 1));
            cv::Mat dilated_region;
            cv::dilate(obstacle, dilated_region, kernel);
            obstacle_layer_voronoi.setTo(255, dilated_region);

#ifdef DEBUG 
            LOG(INFO)<<"show obstacle";
            cv::imshow("obstacle", obstacle);
            cv::waitKey(0);
#endif
            check_image.setTo(255, obstacle);
            // LOG(INFO)<<"check obstacle";
            // cv::imshow("check_image_3", check_image);
            // cv::waitKey(0);
            // 为每个其他障碍添加标记
            // map.add("SAPF_OBSTACLE_" + std::to_string(hull_index), 0);
        }
        else 
        {
            goal_obstacle = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            std::vector<std::vector<cv::Point>> Contours = {contour};

#ifdef DEBUG
            cv::Mat contours_image = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            cv::drawContours(contours_image, Contours, -1, 255, 2);
            cv::imshow("contours_image", contours_image);
            cv::waitKey(0);
#endif
        

            std::vector<std::vector<cv::Point>> hullContours = {hull};
            cv::drawContours(check_image, hullContours, 0, cv::Scalar(255), cv::FILLED);
            cv::drawContours(goal_obstacle, hullContours, 0, cv::Scalar(255), cv::FILLED);
#ifdef DEBUG 
            LOG(INFO)<<"goal in obstacle";
            cv::imshow("goal_obstacle", goal_obstacle);
            cv::waitKey(0);
#endif
            // 将凸区域的部分障碍部分去除，得到内部区域
            cv::Mat hull_region = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            // 整个凸区域
            cv::drawContours(hull_region, hullContours, 0, cv::Scalar(255), cv::FILLED);

#ifdef DEBUG
            cv::imshow("hull_region1", hull_region);
            cv::waitKey(0);
#endif

            // 求voronoi，轨迹优化时用到
            obstacle_layer_voronoi_goal = hull_region.clone();

            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(planning_param.d_safe_rad_goal * 2 + 1, planning_param.d_safe_rad_goal * 2 + 1));
            cv::Mat dilated_region;
            cv::dilate(hull_region, dilated_region, kernel);
#ifdef DEBUG
            cv::imshow("hull_region2", hull_region);
            cv::waitKey(0);
            cv::imshow("dilated_region", dilated_region);
            cv::waitKey(0);
#endif


            obstacle_layer_voronoi.setTo(255, dilated_region);


            // 将原始轮廓区域去除
            cv::drawContours(hull_region, Contours, 0, cv::Scalar(0), cv::FILLED);


#ifdef DEBUG 
            LOG(INFO)<<"goal in obstacle";
            cv::imshow("hull_region3", hull_region);
            cv::waitKey(0);
#endif
            // 这个去除后，里面有多个不连接的区域。这里需要把每个区域分开，只考虑在终点所在的子区域
            std::vector<std::vector<cv::Point>> contours_subregion;
            std::vector<cv::Vec4i> hierarchy_region;
            cv::findContours(hull_region, contours_subregion, hierarchy_region, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
            // LOG(INFO)<<"...."<<contours_subregion.size();
            cv::Mat subregion_goal = cv::Mat::zeros(hull_region.size(), CV_8UC1);

            for (int i = 0; i < contours_subregion.size(); i++)
            {
                cv::Mat tmp = cv::Mat::zeros(hull_region.size(), CV_8UC1);
                cv::drawContours(tmp, contours_subregion, i, cv::Scalar(255), cv::FILLED);
                
#ifdef DEBUG 
                cv::imshow("tmp", tmp);
                cv::waitKey(0);
                LOG(INFO)<<"goal_index_: "<<goal_index_.transpose();
#endif
                if (tmp.at<uchar>(goal_index_.x(), goal_index_.y()) == 255)
                {
                    subregion_goal = tmp;
                    break;
                }
            }
#ifdef DEBUG
            cv::imshow("subregion_goal", subregion_goal);
            cv::waitKey(0);
#endif
            cv::Mat hull_contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            cv::drawContours(hull_contour, hullContours, -1, 255, 1);
            
            cv::Mat line = cv::Mat::zeros(goal_obstacle.size(), CV_8UC1);
            cv::bitwise_and(subregion_goal, hull_contour, line);
#ifdef DEBUG
            cv::imshow("line", line);
            cv::waitKey(0);
#endif
            std::vector<std::vector<cv::Point>> line_contours;
            std::vector<cv::Vec4i> line_hierarchy;
            cv::findContours(line, line_contours, line_hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            // 假设我们只处理第一个轮廓
            const std::vector<cv::Point>& contour = line_contours[0];  

            cv::Point2f tangant = contour.front() - contour.at(contour.size()/2);
            LOG(INFO)<<"tangant: "<<tangant;
            // cv::Point2f normal(center.y, -center.x);
            seg_normal = cv::Point2f(tangant.y, tangant.x);
            LOG(INFO)<<"seg_normal: "<<seg_normal;

            // 线段的两个端点
            cv::Point pt1 = contour.front();
            cv::Point pt2 = contour.at(contour.size()/2);

            seg_point1 = contour.front();
            seg_point2 = contour.at(contour.size()/2);

            // 计算线段的中心点
            cv::Point center((pt1.x + pt2.x) / 2, (pt1.y + pt2.y) / 2);

            // 计算线段的长度
            double length = std::sqrt(std::pow(pt2.x - pt1.x, 2) + std::pow(pt2.y - pt1.y, 2));

            // 设置短轴长度（确保短轴小于长轴）
            double short_axis = planning_param.d_inf_rad_gen;

            // 计算线段的角度（以水平轴为基准的旋转角度）
            double angle = std::atan2(pt2.y - pt1.y, pt2.x - pt1.x) * 180 / CV_PI;

            // 椭圆的长轴和短轴（需要传入半轴长度）
            cv::Size axes(length / 2, short_axis / 2);

            // 绘制椭圆
            cv::ellipse(check_image, center, axes, angle, 0, 360, cv::Scalar(255, 0, 0), -1);

            int extend = 2 * planning_param.d_safe_rad_goal;

            // 计算线段方向向量 (dx, dy)
            cv::Point2f dir = pt2 - pt1;

            // 计算线段的长度
            float length_rect = cv::norm(dir);

            // 归一化方向向量
            cv::Point2f unitDir = dir / length_rect;

            // 计算矩形的长边方向向量，并调整长度
            cv::Point2f longDir = unitDir * ((length_rect) / 2.0);
            cv::Point longDir_int(longDir.x, longDir.y);

            // 计算矩形的短边方向（垂直于线段方向）
            cv::Point2f perpDir(unitDir.y, -unitDir.x); // 顺时针旋转 90°
            cv::Point2f shortDir = perpDir * (extend / 2.0);
            cv::Point shortDir_int(shortDir.x, shortDir.y);

            // 计算矩形四个顶点
            cv::Point rectPts[4] = {
                center + longDir_int, // 左上
                center + longDir_int + shortDir_int, // 右上
                center - longDir_int + shortDir_int, // 右下
                center - longDir_int  // 左下
            };
            
            cv::fillConvexPoly(hole_rect, rectPts, 4, 255);

            cv::Mat kernel_noise = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::dilate(hole_rect, hole_rect, kernel_noise);
            obstacle_layer_voronoi.setTo(255, dilated_region);

            // 把这一区域涂成黑色，表示不是障碍
            // obstacle_layer_voronoi.setTo(0, hole_rect);
#ifdef DEBUG
            cv::imshow("check_image_LAST", check_image);
            cv::waitKey(0);
#endif
        }
    }
    // 把开口这一区域不设置为障碍区域
    obstacle_layer_voronoi.setTo(0, hole_rect);
    obstacle_layer_voronoi.setTo(0, obstacle_layer_voronoi_goal);
    LOG(INFO)<<"obstacle_layer_voronoi";
    cv::imshow("obstacle_layer_voronoi", obstacle_layer_voronoi);
    cv::waitKey(0);
    cv::imshow("obstacle_layer_voronoi_goal", obstacle_layer_voronoi_goal);
    cv::waitKey(0);
    cv::imshow("hole_rect", hole_rect);
    cv::waitKey(0);

    // 为障碍添加层
    if (cv::countNonZero(goal_obstacle) > 0)
    {
        map.add("obstacle_goal_X", 0);
        map.add("obstacle_goal_Y", 0);
        map.add("obstacle_goal_FLAG", 0);
    }
    for (int i = 0; i < obstacles.size(); i++)
    {
        map.add("obstacle_"+std::to_string(i) + "_X", 0);
        map.add("obstacle_"+std::to_string(i) + "_Y", 0);
        map.add("obstacle_"+std::to_string(i) + "_FLAG", 0);
    }
    LOG(INFO)<<"OVER";
}


bool PathPlanning::getNearestPointInContourNew(vector<cv::Point> & contour, cv::Point & nearestPoint)
{
    // 用于统计两侧点数的变量
    int countSide1 = 0, countSide2 = 0;
    double A = seg_point2.y - seg_point1.y;
    double B = seg_point1.x - seg_point2.x;
    double C = seg_point2.x * seg_point1.y - seg_point1.x * seg_point2.y;
    // 分割轮廓成两部分
    std::vector<cv::Point> side1, side2;
    std::vector<int> side1_indices, side2_indices;
    for (int i = 0; i < contour.size(); ++i) 
    {
        const auto& point = contour[i];
        double D = A * point.x + B * point.y + C;
        if (D > 0) {
            side1.push_back(point);
            side1_indices.push_back(i);
        } else if (D < 0) {
            side2.push_back(point);
            side2_indices.push_back(i);
        }
    }
    std::vector<cv::Point> smallerPart = (side1.size() < side2.size()) ? side1 : side2;
    std::vector<int> smallerIndices = (side1.size() < side2.size()) ? side1_indices : side2_indices;

    // 找到较少部分中与直线最近的点
    int recent_index = 0;  // 原始轮廓中的索引
    double min_dist = std::numeric_limits<double>::max();
    for (int i = 0; i < smallerIndices.size(); ++i) {
        const auto& point = contour[smallerIndices[i]];
        double dist = std::abs(A * point.x + B * point.y + C) / std::sqrt(A * A + B * B);
        if (dist < min_dist) {
            min_dist = dist;
            recent_index = smallerIndices[i];
        }
    }

    // 按照与最近点的环形索引差排序
    int N = contour.size();
    std::sort(smallerIndices.begin(), smallerIndices.end(),
              [recent_index, N](int idx1, int idx2) {
                  int diff1 = std::min((idx1 - recent_index + N) % N, (recent_index - idx1 + N) % N);
                  int diff2 = std::min((idx2 - recent_index + N) % N, (recent_index - idx2 + N) % N);
                  return diff1 < diff2;
              });

    // 找到排序后的中间点
    int median_index = smallerIndices[smallerIndices.size() / 2];
    cv::Point medianPoint = contour[median_index];
    nearestPoint = medianPoint;
    if (nearestPoint.x == -1 || nearestPoint.y == -1)
    {
        return false;
    }
    else
    {
        return true;
    }
}


// 对goalregion和非goalregion内的斥力需要使用不同参数
// 非goalregion的距离参数应该更小，但是goalregion的距离参数更大
void PathPlanning::computeRepObstacleGoal()
{
    if (cv::countNonZero(goal_obstacle) > 0)
    {
#ifdef DEBUG 
        cv::imshow("current image", goal_obstacle);
        cv::waitKey(0);
#endif
        string obstacle_goal_flag = "obstacle_goal_FLAG";
        string x_obstacle_goal = "obstacle_goal_X";
        string y_obstacle_goal = "obstacle_goal_Y";
        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(1, 1));

        auto goal_obstacle_bk = goal_obstacle.clone();
        std::vector<std::vector<cv::Point>> raw_contours;
        std::vector<cv::Vec4i> raw_hierarchy;
        cv::findContours(goal_obstacle_bk, raw_contours, raw_hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        std::vector<double> x_start, y_start, u, v;
        for (int inflation_radius = 1; inflation_radius <= planning_param.d_noinflu_rad_goal; inflation_radius++)
        {
            double d =  planning_param.resolution * (inflation_radius);
            double d_rel;
            if (d < planning_param.d_safe_goal)
            {
                d_rel = 0;
            }
            else if (d > planning_param.d_inf_goal)
            {
                d_rel = 1;
            }
            else
            {
                d_rel = (d - planning_param.d_safe_goal) / (2 * (planning_param.d_vort_goal - planning_param.d_safe_goal));
            }
            double r;
            if (d_rel <= 0.5)
            {
                r = 3.1415926 * d_rel;
            }
            else
            {
                r = 3.1415926 * (1 - d_rel);
            }

            double F_SAPF;
            if (d <= planning_param.d_noinflu_goal)
            {
                F_SAPF = planning_param.goal_obstacle_cof * (1/(d) - 1/planning_param.d_noinflu_goal) * (1/(planning_param.d_noinflu_goal * planning_param.d_noinflu_goal));
            }
            else
            {
                F_SAPF = 0;
            }
            cv::dilate(goal_obstacle_bk, goal_obstacle_bk, element);
            std::vector<std::vector<cv::Point>> single_contours;
            std::vector<cv::Vec4i> single_hierarchy;
            cv::findContours(goal_obstacle_bk, single_contours, single_hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            if (inflation_radius <= planning_param.d_safe_rad_goal || inflation_radius > planning_param.d_inf_rad_goal)
            {
                cv::Mat RefImage = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
                for (size_t i = 0; i < single_contours[0].size(); i++) 
                {
                    // cv::Point x是列 y是行
                    cv::Point2f pt1 = single_contours[0][i];
                    cv::Point2f pt2 = single_contours[0][(i + 1) % single_contours[0].size()];
                    cv::Point2f tangent = pt2 - pt1; // 列行
                    cv::Point2f normal = cv::Point2f(tangent.y, -tangent.x);
                    // LOG(INFO)<<normal;
                    float length = cv::norm(normal);
                    if (length > 0) 
                    {
                        normal /= length;  
                    }
                    
                    cv::Point2f F_SAPF_V = F_SAPF * normal;

                    // 在 cv::Point2f 中，第一个元素代表列（也就是 X 方向），第二个元素代表行（Y 方向）
                    if (check_image.at<uchar>(pt1) != 255)
                    {
                        // 通过像素坐标获得地图index坐标 还需要乘以本身的力
                        grid_map::Index index(pt1.y, pt1.x);
                        if (map[obstacle_goal_flag](index.x(), index.y()) == 0)
                        {
                            // 这个地方可能不对
                            map[x_obstacle_goal](index.x(), index.y()) += F_SAPF_V.y;
                            map[y_obstacle_goal](index.x(), index.y()) += F_SAPF_V.x;
                            map[obstacle_goal_flag](index.x(), index.y()) = 1;
                            if (i%18 == 0 && inflation_radius%10 == 0)
                            {
                                cv::Point start(pt1.x, pt1.y);
                                cv::Point end(start.x + map[x_obstacle_goal](index.x(), index.y()) * 5, start.y + map[y_obstacle_goal](index.x(), index.y()) * 5);
                                cv::arrowedLine(RefImage, start, end, cv::Scalar(255), 2, 8, 0, 0.1);
                                // LOG(INFO)<<"normal: "<<normal;   
                                grid_map::Position p2;
                                if (map.getPosition(index, p2))
                                {
                                    x_start.emplace_back(p2.x());
                                    y_start.emplace_back(p2.y());
                                    u.emplace_back(normal.y);
                                    v.emplace_back(normal.x);
                                }                
                            }
                        }
                    } 
                }
            }
            else
            {
                cv::Mat clock_wise_mat = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
                cv::Mat counter_clock_wise_mat = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
                cv::Point nearest_point(-1, -1);
                if (getNearestPointInContourNew(single_contours[0], nearest_point))
                {
#ifdef DEBUG
                    LOG(INFO)<<"nearest_point: "<<nearest_point;
#endif
                    // 根据开口分割点来确定
                    int open_index = -1;
                    for (int i = 0; i < single_contours[0].size(); i++)
                    {
                        if (nearest_point == single_contours[0][i])
                        {
                            open_index = i;
                        }
                    }
                    CHECK(open_index != -1);
#ifdef DEBUG
                    LOG(INFO)<<"open_index: "<<open_index;
#endif
                    int index_count_clock = open_index;
                    int index_clock = open_index - 1 >= 0 ? open_index - 1 : single_contours[0].size() - 1;
                    
                    for (int i = 1; i < single_contours[0].size()/2; i++)
                    {
                        index_count_clock = index_count_clock + 1 >= single_contours[0].size() ? (index_count_clock + 1)%single_contours[0].size() : index_count_clock + 1;
                        index_clock = index_clock - 1 < 0 ? (index_clock - 1 + single_contours[0].size())%single_contours[0].size() : index_clock - 1;
                        clock_wise_mat.at<uchar>(single_contours[0][index_count_clock]) = 255;
                        counter_clock_wise_mat.at<uchar>(single_contours[0][index_clock]) = 255;
                    }
                }
                cv::Mat RefImage = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
                for (size_t i = 0; i < single_contours[0].size(); i++) 
                {
                    cv::Point2f pt1 = single_contours[0][i];
                    cv::Point2f pt2 = single_contours[0][(i + 1) % single_contours[0].size()];
                    cv::Point2f tangent = pt2 - pt1;
                    cv::Point2f normal = cv::Point2f(tangent.y, -tangent.x);
                    float length = cv::norm(normal);
                    if (length > 0) 
                    {
                        normal /= length;  
                    }
                    cv::Point2f normal_adj;
                    if (counter_clock_wise_mat.at<uchar>(single_contours[0][i]) == 255)
                    {
                        normal_adj.x = cos(-r)*normal.x - sin(-r)*normal.y;
                        normal_adj.y = sin(-r)*normal.x + cos(-r)*normal.y;
                    }
                    else
                    {
                        normal_adj.x = cos(r)*normal.x - sin(r)*normal.y;
                        normal_adj.y = sin(r)*normal.x + cos(r)*normal.y;
                    }
                    cv::Point2f F_SAPF_V = F_SAPF * normal_adj;
                    if (check_image.at<uchar>(pt1) != 255)
                    {
                        grid_map::Index index(pt1.y, pt1.x);
                        // 只有这个点没有被赋值，才能行
                        if (map[obstacle_goal_flag](index.x(), index.y()) == 0)
                        {
                            map[x_obstacle_goal](index.x(), index.y()) += F_SAPF_V.y;
                            map[y_obstacle_goal](index.x(), index.y()) += F_SAPF_V.x; 
                            map[obstacle_goal_flag](index.x(), index.y()) = 1;
                            if (i%10 == 0 && inflation_radius%8 == 0)
                            {
                                cv::Point start(pt1.x, pt1.y);
                                cv::Point end(start.x + map[x_obstacle_goal](index.x(), index.y()) * 5, start.y + map[y_obstacle_goal](index.x(), index.y()) * 5);
                                cv::arrowedLine(RefImage, start, end, cv::Scalar(255), 2, 8, 0, 0.1);
                                // LOG(INFO)<<"normal: "<<normal;   
                                grid_map::Position p2;
                                if (map.getPosition(index, p2))
                                {
                                    // LOG(INFO)<<"p2: "<<p2.transpose();
                                    x_start.emplace_back(p2.x());
                                    y_start.emplace_back(p2.y());
                                    u.emplace_back(normal_adj.y);
                                    v.emplace_back(normal_adj.x);
                                }
                            }    
                        }
                    }
                }   
            }
        }
        LOG(INFO)<<"show goal apf";
        matplotlibcpp::cla();
        matplotlibcpp::quiver(x_start, y_start, u, v);
        matplotlibcpp::axis("equal");  
#ifdef SHOW_POTENTIAL_FIELD
        matplotlibcpp::show(); 
#endif

        matplotlibcpp::save(experiment_path + "goal_apf.pdf", 800);
    }
}

void PathPlanning::computeRepObstacle()
{
    LOG(INFO)<<"computeRepObstacle";
    // cv::Mat element_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    for (int i = 0; i < obstacles.size(); i++)
    {
        string hull_obstacle_string = "obstacle_"+std::to_string(i) + "_FLAG";
        string x_sapf = "obstacle_"+std::to_string(i) + "_X";
        string y_sapf = "obstacle_"+std::to_string(i) + "_Y";
        cv::Mat obstacle = obstacles.at(i).clone();
#ifdef DEBUG
        cv::imshow("obstacle", obstacle);
        cv::waitKey(0);
#endif
        std::vector<double> x_start, y_start, u, v;
        cv::Mat contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
        for (int inflation_radius = 1; inflation_radius <= planning_param.d_noinflu_rad_gen; inflation_radius++)
        {
            double d = planning_param.resolution * (inflation_radius);
            double F_SAPF;
            if (d <= planning_param.d_noinflu_gen)
            {
                F_SAPF = planning_param.gen_obstacle_cof * (1/(d) - 1/planning_param.d_noinflu_gen) * (1/(planning_param.d_noinflu_gen * planning_param.d_noinflu_gen));
            }
            else
            {
                F_SAPF = 0;
            }
            cv::dilate(obstacle, obstacle, element);
            // cv::imshow("obstacle", obstacle);
            // cv::waitKey(0);
            std::vector<std::vector<cv::Point>> single_contours;
            std::vector<cv::Vec4i> single_hierarchy;
            cv::findContours(obstacle, single_contours, single_hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            if (inflation_radius <= planning_param.d_safe_rad_gen || inflation_radius >= planning_param.d_inf_rad_gen)
            {
                // LOG(INFO)<<"GENERAL DIRECT";
#ifdef DEBUG
#endif
                cv::Mat RefImage = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
                for (size_t i = 0; i < single_contours[0].size(); i++) 
                {
                    
                    cv::Point2f pt1 = single_contours[0][i];
                    cv::Point2f pt2 = single_contours[0][(i + 1) % single_contours[0].size()];
                    cv::Point2f tangent = pt2 - pt1;
                    cv::Point2f normal = cv::Point2f(tangent.y, -tangent.x);

                    float length = cv::norm(normal);
                    if (length > 0) 
                    {
                        normal /= length;  
                    }
                    cv::Point2f F_SAPF_V = F_SAPF * normal;
                    // 通过像素坐标获得地图index坐标 还需要乘以本身的力
                    if (check_image.at<uchar>(pt1) != 255)
                    {
                        grid_map::Index index(pt1.y, pt1.x);
                        if (map[hull_obstacle_string](index.x(), index.y()) == 0)
                        { 
                            map[x_sapf](index.x(), index.y()) += F_SAPF_V.y;
                            map[y_sapf](index.x(), index.y()) += F_SAPF_V.x;
                            if (i%18 == 0 && inflation_radius%10 == 0)
                            {
                                cv::Point start(pt1.x, pt1.y);
                                cv::Point end(start.x + map[x_sapf](index.x(), index.y()) * 5, start.y + map[y_sapf](index.x(), index.y()) * 5);
                                cv::arrowedLine(RefImage, start, end, cv::Scalar(255), 2, 8, 0, 0.1);

                                grid_map::Position p2;
                                if (map.getPosition(index, p2))
                                {
                                    x_start.emplace_back(p2.x());
                                    y_start.emplace_back(p2.y());
                                    // u.emplace_back(map[x_sapf](index.x(), index.y()));
                                    // v.emplace_back(map[y_sapf](index.x(), index.y()));
                                    u.emplace_back(normal.y);
                                    v.emplace_back(normal.x);
                                }
                            }
                            map[hull_obstacle_string](index.x(), index.y()) = 1;
                        }
                    }
                }
            }
            else
            {
                double d_rel;
                if (d < planning_param.d_safe_gen)
                {
                    d_rel = 0;
                }
                else if (d > planning_param.d_inf_gen)
                {
                    d_rel = 1;
                }
                else
                {
                    d_rel = (d - planning_param.d_safe_gen) / (2 * (planning_param.d_vort_gen - planning_param.d_safe_gen));
                }
                double r;
                if (d_rel <= 0.5)
                {
                    r = 3.1415926 * d_rel;
                }
                else
                {
                    r = 3.1415926 * (1 - d_rel);
                }

                cv::Moments moments = cv::moments(single_contours[0]);
                // 计算重心坐标
                if (moments.m00 == 0) 
                {  // 防止除零错误
                    continue;
                }
                double cx = moments.m10 / moments.m00;
                double cy = moments.m01 / moments.m00;
                cv::Point2f centroid(cx, cy);
                // 假设另一个点为 (px, py)，可以根据实际需要设定
                cv::Point2f goal_point(goal_index_.y(), goal_index_.x()); 
#ifdef DEBUG 
#endif
                cv::Mat RefImage = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
                for (size_t i = 0; i < single_contours[0].size(); i++)
                {
                    cv::Point2f pt1 = single_contours[0][i];
                    cv::Point2f pt2 = single_contours[0][(i + 1) % single_contours[0].size()];
                    cv::Point2f tangent = pt2 - pt1;
                    cv::Point2f normal = cv::Point2f(tangent.y, -tangent.x);
                    float length = cv::norm(normal);
                    if (length > 0) 
                    {
                        normal /= length;  
                    }
                    cv::Point2f normal_adj;

                    // 终点与质心的连线，质心与像素的夹角
                    cv::Point2f v1 = goal_point - centroid;
                    cv::Point2f v2 = pt1 - centroid;
                    if (v1.x * v2.y - v1.y * v2.x > 0)
                    {
                        normal_adj.x = cos(-r)*normal.x - sin(-r)*normal.y;
                        normal_adj.y = sin(-r)*normal.x + cos(-r)*normal.y;
                    }
                    else
                    {
                        normal_adj.x = cos(r)*normal.x - sin(r)*normal.y;
                        normal_adj.y = sin(r)*normal.x + cos(r)*normal.y;
                    }
                    cv::Point2f F_SAPF_V = F_SAPF * normal_adj;
                    // LOG(INFO)<<"normal_adj: "<<normal_adj;
                    // LOG(INFO)<<"pt1: "<<pt1;
                    if (check_image.at<uchar>(pt1) != 255)
                    {
                        // contour.at<uchar>(single_contours[0][i]) = 255;
                        grid_map::Index index(pt1.y, pt1.x);
                        // LOG(INFO)<<"index: "<<index.transpose();
                        if (map[hull_obstacle_string](index.x(), index.y()) == 0)
                        {
                            // map[x_sapf](index.x(), index.y()) += normal_adj.y;
                            // map[y_sapf](index.x(), index.y()) += normal_adj.x; 
                            map[x_sapf](index.x(), index.y()) += F_SAPF_V.y;
                            map[y_sapf](index.x(), index.y()) += F_SAPF_V.x; 
                            if (i%10 == 0 && inflation_radius%8 == 0)
                            {
                                cv::Point start(pt1.x, pt1.y);
                                cv::Point end(start.x + map[x_sapf](index.x(), index.y()) * 5, start.y + map[y_sapf](index.x(), index.y()) * 5);
                                cv::arrowedLine(RefImage, start, end, cv::Scalar(255), 2, 8, 0, 0.1);
                                grid_map::Position p2;
                                if (map.getPosition(index, p2))
                                {
                                    // LOG(INFO)<<"p2: "<<p2.transpose();
                                    x_start.emplace_back(p2.x());
                                    y_start.emplace_back(p2.y());
                                    // u.emplace_back(map[x_sapf](index.x(), index.y()));
                                    // v.emplace_back(map[y_sapf](index.x(), index.y()));
                                    u.emplace_back(normal_adj.y);
                                    v.emplace_back(normal_adj.x);
                                }
                            }
                            map[hull_obstacle_string](index.x(), index.y()) = 1;
                        }
                    }
                }
            }
        }
// #ifdef DEBUG
        matplotlibcpp::cla();
        matplotlibcpp::quiver(x_start, y_start, u, v);
        matplotlibcpp::axis("equal");
#ifdef SHOW_POTENTIAL_FIELD
        matplotlibcpp::show();
#endif
        matplotlibcpp::save(experiment_path + std::to_string(i) +"_gen_obsatcle.pdf", 800);
// #endif
    }
}

// 合并所有障碍的势场
void PathPlanning::mergeAllObstacle()
{
    // LOG(INFO)<<"OKKK";
    SAPF_X = "SAPF_X";
    SAPF_Y = "SAPF_Y";
    map.add(SAPF_X, 0);
    map.add(SAPF_Y, 0);
    
    // LOG(INFO)<<"OKKK";
    bool flag = cv::countNonZero(goal_obstacle) > 0;
    // LOG(INFO)<<flag;
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            if (flag)
            {
                map[SAPF_X](i, j) += map.get("obstacle_goal_X")(i, j);
                map[SAPF_Y](i, j) += map.get("obstacle_goal_Y")(i, j);
            }
            // LOG(INFO)<<"OKKK";
            for (int k = 0; k < obstacles.size(); k++)
            {
                string obstacle_x = "obstacle_"+std::to_string(k) + "_X";
                string obstacle_y = "obstacle_"+std::to_string(k) + "_Y";
                map[SAPF_X](i, j) += map.get(obstacle_x)(i, j);
                map[SAPF_Y](i, j) += map.get(obstacle_y)(i, j);
                // LOG(INFO)<<i<<" "<<j<<" "<<map[SAPF_X](i, j)<<" "<<map[SAPF_Y](i, j);
            }
        }
    }
    
    // if (cv::countNonZero(goal_obstacle) > 0)
    // {
    //     map[SAPF_X] = map.get("obstacle_goal_X");
    //     map[SAPF_Y] = map.get("obstacle_goal_Y");
    // }
    // LOG(INFO)<<"OKKK";

    // for (int i = 0; i < obstacles.size(); i++)
    // {
    //     string obstacle_x = "obstacle_"+std::to_string(i) + "_X";
    //     string obstacle_y = "obstacle_"+std::to_string(i) + "_Y";
    // LOG(INFO)<<"OKKK";

    //     map[SAPF_X] += map[obstacle_x];
    // LOG(INFO)<<"OKKK";

    //     map[SAPF_Y] += map[obstacle_y];
    // }
    // LOG(INFO)<<"OKKK";
}

void PathPlanning::showSAPFMatplotlib()
{
    std::vector<double> x_start, y_start, u, v;
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            if (i%18 == 0 && j%10 ==0)
            {
                grid_map::Index index(i, j);
                if (map[SAPF_X](index.x(), index.y()) != 0 || map[SAPF_Y](index.x(), index.y()) != 0)
                {
                    grid_map::Position p2;
                    if (map.getPosition(index, p2))
                    {
                        x_start.emplace_back(p2.x());
                        y_start.emplace_back(p2.y());
                        // LOG(INFO)<<p2.transpose()<<" "<<map.get(SAPF_X)(i, j)<<" "<<map[SAPF_Y](i, j)<<" "<<i<<" "<<j;
                        Eigen::Vector2d normal_(map[SAPF_X](i, j), map[SAPF_Y](i, j));
                        // LOG(INFO)<<"normal_: "<<normal_.transpose();
                        normal_.normalize();
                        // LOG(INFO)<<"normal_: "<<normal_.transpose();
                        u.emplace_back(normal_.x());
                        v.emplace_back(normal_.y());
                    }
                }
            }
        }
    }
    LOG(INFO)<<x_start.size()<<" "<<y_start.size()<<" "<<u.size()<<" "<<v.size();
    matplotlibcpp::cla();
    matplotlibcpp::quiver(x_start, y_start, u, v);

    matplotlibcpp::axis("equal");

    // 显示图像
#ifdef SHOW_POTENTIAL_FIELD
    matplotlibcpp::show();
#endif

    matplotlibcpp::save(experiment_path + "obstacle_all.pdf", 800);
}

bool PathPlanning::AttPotential(Node & node, Eigen::Vector2d & AttForce)
{
    if (map.isInside(node.position))
    {
        grid_map::Index index;
        if (map.getIndex(node.position, index))
        {
            if (obstacle_layer.at<uchar>(index.x(), index.y()) == 0) // 位于不可通行区域外部
            {
                double d = (goal_.head(2) - node.position).norm();
                if (d < planning_param.d_g_att)
                {
                    // 计算目标位置与当前节点位置的差值
                    Eigen::Vector2d position_diff = goal_.head(2) - node.position;

                    // 检查差值向量的长度是否为0
                    double diff_length = position_diff.norm();
                    if (diff_length == 0) {
                        // 处理向量长度为0的情况，例如返回一个默认值或抛出异常
                        throw std::runtime_error("Position difference vector is zero, cannot normalize.");
                    }
                    // 归一化向量并乘以吸引力阈值
                    AttForce = position_diff / diff_length * planning_param.d_g_att_cof;
                }
                else
                {
                    // 计算目标位置与当前节点位置的差值
                    Eigen::Vector2d position_diff = goal_.head(2) - node.position;

                    // 检查差值向量的长度是否为0
                    double diff_length = position_diff.norm();
                    if (diff_length == 0) {
                        // 处理向量长度为0的情况，例如返回一个默认值或抛出异常
                        throw std::runtime_error("Position difference vector is zero, cannot normalize.");
                    }

                    // 归一化向量并乘以吸引力阈值
                    AttForce = (position_diff / diff_length) * planning_param.d_g_att_cof * planning_param.d_g_att / d;
                }
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}

void PathPlanning::showAttPotential()
{
    std::vector<double> x_start, y_start, u, v;
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            if (i%18 == 0 && j%10 ==0)
            {
                grid_map::Position p2;
                if (map.getPosition(grid_map::Index(i, j), p2))
                {
                    Node node;
                    node.position = p2;
                    Eigen::Vector2d SAPF;
                    if (AttPotential(node, SAPF))
                    {
                        SAPF.normalize();
                        x_start.emplace_back(p2.x());
                        y_start.emplace_back(p2.y());
                        u.emplace_back(SAPF.x());
                        v.emplace_back(SAPF.y());
                    }
                }
            }
            
        }
    }
    matplotlibcpp::cla();
    matplotlibcpp::quiver(x_start, y_start, u, v);

    matplotlibcpp::axis("equal");

    // 显示图像
#ifdef SHOW_POTENTIAL_FIELD
    matplotlibcpp::show();
#endif
    matplotlibcpp::save(experiment_path + "att.pdf", 800);
    
}

void PathPlanning::showSAPF()
{
    std::vector<double> x_start, y_start, u, v;
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            if (i%18 == 0 && j%10 ==0)
            {
                grid_map::Position p2;
                if (map.getPosition(grid_map::Index(i, j), p2))
                {
                    Node node;
                    node.position = p2;
                    Eigen::Vector2d SAPF;
                    if (getSAPF(node, SAPF))
                    {
                        SAPF.normalize(); 
                        x_start.emplace_back(p2.x());
                        y_start.emplace_back(p2.y());
                        u.emplace_back(SAPF.x());
                        v.emplace_back(SAPF.y());
                    }
                }
            }
        }
    }
    matplotlibcpp::cla();
    matplotlibcpp::quiver(x_start, y_start, u, v);

    matplotlibcpp::axis("equal");

    // 显示图像
#ifdef SHOW_POTENTIAL_FIELD
    matplotlibcpp::show();
#endif
    
    matplotlibcpp::save(experiment_path + "SAPF.pdf", 800);
}


bool PathPlanning::processing()
{
    constructPlaneAwareMap();
    constructObstacleLayer(planning_param.obstacle_rad);
    constructFullFeasibleRegion(planning_param.safe_region_radius);
    computeRep();
    return plan();
}


void PathPlanning::computeRep()
{
    computeObstacles();
    computeRepObstacleGoal();
    computeRepObstacle();
    mergeAllObstacle();
    showSAPFMatplotlib();
    showAttPotential();
    showSAPF();
}

bool PathPlanning::getSAPF(Node & node, Eigen::Vector2d & SAPF)
{
    grid_map::Index index;
    if (map.getIndex(node.position, index))
    {
        if (obstacle_layer.at<uchar>(index.x(), index.y()) == 0)
        {
            if (map.exists(SAPF_X) && map.exists(SAPF_Y))
            {
                SAPF.x() = map[SAPF_X](index.x(), index.y());
                SAPF.y() = map[SAPF_Y](index.x(), index.y());
                return true;
            }
            else
            {
                LOG(ERROR)<<"SAPF_X or SAPF_Y not exist";
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
}


bool PathPlanning::ComputeNodeCostAstar(NodePtr NodeP)
{
    if (map.isInside(NodeP->position) && map.isInside(NodeP->PreFootstepNode->position))
    {
        NodeP->g_cost = NodeP->PreFootstepNode->g_cost + (NodeP->position - NodeP->PreFootstepNode->position).norm();
        NodeP->h_cost = (goal_.head(2) - NodeP->position).norm()*1.5 + abs(NodeP->ori - goal_.z()) * 0.5;
        NodeP->cost = NodeP->g_cost + NodeP->h_cost + abs(NodeP->ori - NodeP->PreFootstepNode->ori) * 2; 
        return true;
    }
    else
    {
        return false;
    }
    
}

// 计算下一个节点的代价
// 与合力方向的差距
bool PathPlanning::ComputeNodeCost(NodePtr currentNodeP, NodePtr nextNodeP, double apfForce)
{
    // LOG(INFO)<<"ComputeNodeCost"<<endl;
    if (map.isInside(nextNodeP->position) && map.isInside(currentNodeP->position))
    {
        // 分为贪心项和启发项，贪心项由所行驶的路程决定，启发项由当前点到目标点的距离及角度差决定
        // 整个代价还包括当前节点方向与下一节点方向偏差，
        // LOG(INFO)<<currentNodeP->ori<<" "<<nextNodeP->ori<<endl;
        nextNodeP->g_cost = currentNodeP->g_cost + (currentNodeP->position - nextNodeP->position).norm();
        nextNodeP->h_cost = (goal_.head(2) - nextNodeP->position).norm()*1.5 + abs(nextNodeP->ori - goal_.z()) * 0.2;
        nextNodeP->cost = nextNodeP->g_cost + nextNodeP->h_cost + abs(nextNodeP->ori - currentNodeP->ori) + abs(nextNodeP->ori - apfForce) * 12;
        return true;
    }
    else
    {
        return false;
    }
}

// 画出图中检查的节点是在哪个区域
void PathPlanning::drawCandidateSquare(NodePtr currentNodeP)
{
    Eigen::AngleAxisd ax(currentNodeP->ori, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d mid(currentNodeP->position.x(), currentNodeP->position.y(), 0);
    // 求4个子区域
    Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(planning_param.support_area.Up, 0, 0) + mid;
    Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, planning_param.support_area.Left, 0) + mid_top;
    Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - planning_param.support_area.Right, 0) + mid_top;

    Eigen::Vector3d mid_button = ax.toRotationMatrix() * Eigen::Vector3d(- planning_param.support_area.Button, 0, 0) + mid;
    Eigen::Vector3d button_left = ax.toRotationMatrix() * Eigen::Vector3d(0, planning_param.support_area.Left, 0) + mid_button;
    Eigen::Vector3d button_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - planning_param.support_area.Right, 0) + mid_button;
    auto candidataImage = debug_image.clone();
    if (map.isInside(top_left.head(2)) && map.isInside(top_right.head(2)) && map.isInside(button_left.head(2)) && map.isInside(button_right.head(2)))
    {
        grid_map::Index top_left_idx, top_right_idx, button_left_idx, button_right_idx;
        map.getIndex(top_left.head(2), top_left_idx);
        map.getIndex(top_right.head(2), top_right_idx);
        map.getIndex(button_left.head(2), button_left_idx);
        map.getIndex(button_right.head(2), button_right_idx);
        vector<cv::Point> points;
        points.push_back(cv::Point(top_left_idx(1), top_left_idx(0)));
        points.push_back(cv::Point(top_right_idx(1), top_right_idx(0)));
        points.push_back(cv::Point(button_right_idx(1), button_right_idx(0)));
        points.push_back(cv::Point(button_left_idx(1), button_left_idx(0)));
        cv::polylines(candidataImage, points, true, cv::Scalar(0, 0, 255), 2);
    }
    cv::imshow("candidataImage", candidataImage);
    cv::waitKey(0);
}

bool PathPlanning::isCorss(NodePtr currentNodeP)
{
    grid_map::Index current_index;
    grid_map::Index pre_index;
    if (map.getIndex(currentNodeP->position, current_index) && map.getIndex(currentNodeP->PreFootstepNode->position, pre_index))
    {
        cv::Mat corss_image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
        cv::line(corss_image, cv::Point(pre_index(1), pre_index(0)), cv::Point(current_index(1), current_index(0)), 2);
        cv::Mat insection;
        cv::bitwise_and(obstacle_layer, corss_image, insection);
        vector<cv::Point> Nozeros;
        cv::findNonZero(insection, Nozeros);
        if (!Nozeros.empty() > 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return true;
    }
    
}


// 四面八方都打分，与yaw越一致的地方打分越高
// 先按四面八方来获取所有的节点

bool PathPlanning::transitions(NodePtr currentNodeP, double yaw, std::vector<NodePtr> & nodes)
{
    Eigen::Vector3d raw_length(planning_param.step, 0, 0);
    Eigen::Vector3d raw_point = Eigen::Vector3d::Zero();
    raw_point.head(2) = currentNodeP->position;
    
    // for (auto & angle : current_angles)
    // {
    //     LOG(INFO)<<"angle:"<<angle;
    // }
    // for (int i = 0; i < current_angles.size(); i++)
    // {
    //     LOG(INFO)<<"i = "<<i<<" "<<current_angles.at(i);
    // }

    // vector<Eigen::AngleAxisd> rots;
    // for (auto & angle : current_angles)
    // {
    //     rots.emplace_back(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
    // }
    grid_map::Index index;
    if (map.getIndex(currentNodeP->position, index))
    {
        if (full_feasible_region.at<uchar>(index.x(), index.y()) == 255) // 如果在一个安全裕度范围内，则使用原始的节点扩展
        {      
            auto current_angles = angles_safe;
            std::transform(current_angles.begin(), current_angles.end(), current_angles.begin(), [yaw](double x) {
                return yaw + x;
            });
            // LOG(INFO)<<"IN FULL FEASIBLE REGION";   
            for (auto & angle : current_angles)
            {
                Eigen::Vector3d point = raw_point + Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix() * raw_length;
                Node node;
                node.position = point.head(2);
                node.ori = angle;
                NodePtr next_node_p = std::make_shared<Node>(node);
                next_node_p->PreFootstepNode = currentNodeP;
                if (!isCorss(next_node_p))
                {
                    // LOG(INFO)<<"position: "<<node.position.transpose();
                    // LOG(INFO)<<"angle: "<<angle;
                    if (ComputeNodeCost(currentNodeP, next_node_p, yaw))
                    {
                        
                        // LOG(INFO)<<"g cost: "<<next_node_p->g_cost<<", h cost: "<<next_node_p->h_cost<<", cost: "<<next_node_p->cost;
                        nodes.emplace_back(next_node_p);
                    }
                    // 检查node的可通行性
                }
                
            }
            return true;
        }
        else // 如果不在安全裕度内，则根据方向来寻找节点
        {
            auto current_angles = angle_dang;
            std::transform(current_angles.begin(), current_angles.end(), current_angles.begin(), [yaw](double x) {
                return yaw + x;
            });
            // LOG(INFO)<<"NOT IN FULL FEASIBLE REGION";
            // LOG(INFO)<<"currentNodeP POSITION: "<<currentNodeP->position.transpose();
            // LOG(INFO)<<"currentNodeP angle: "<<currentNodeP->ori;
            // LOG(INFO)<<"currentNodeP cost: "<<currentNodeP->cost;   
            for (auto & angle : current_angles)
            {
                // LOG(INFO)<<"angle: "<<angle;
                // 找到第一个不为可通行的节点，还需要好好规划这个节点
                
                // double d_i = 0.4;
                for (double d_l = 0.5; d_l > 0.01; d_l -= 0.05)
                {
                    Node n_l;
                    // Node n_l, n_i;
                    // LOG(INFO)<<"d_l: "<<d_l;
                    Eigen::Vector3d length_l(d_l, 0, 0);
                    // Eigen::Vector3d length_i(d_i, 0, 0);
                    n_l.position = (raw_point + Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix() * length_l).head(2);
                    n_l.ori = angle;

                    // 在图上画出检查的区域
                    n_l.PreFootstepNode = currentNodeP;

                    if (!isCorss(std::make_shared<Node>(n_l)))
                    {
                        // n_i.position = (raw_point + Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix() * length_i).head(2);
                        // n_i.ori = angle;
                        if (CheckTraversability(n_l))
                        {
                            NodePtr next_node_p = std::make_shared<Node>(n_l);
                            if (ComputeNodeCost(currentNodeP, next_node_p, yaw))
                            {
                                next_node_p->PreFootstepNode = currentNodeP;
                                // LOG(INFO)<<"position: "<<next_node_p->position.transpose();
                                // LOG(INFO)<<"angle: "<<next_node_p->ori;
                                // LOG(INFO)<<"cost: "<<next_node_p->cost;
                                nodes.emplace_back(next_node_p);
                                break;
                            }
                            // else
                            // {
                                // LOG(INFO)<<"not in map";
                            // }
                        }
                        // else
                        // {
                        //     LOG(INFO)<<"untraversable";
                        // }
                        // if (CheckTraversability(n_i))
                        // {
                        //     NodePtr next_node_p = std::make_shared<Node>(n_i);
                        //     if (ComputeNodeCost(currentNodeP, next_node_p, yaw))
                        //     {
                        //         next_node_p->PreFootstepNode = currentNodeP;
                        //         nodes.emplace_back(next_node_p);
                        //         break;
                        //     }
                        // }
                        // else
                        // {
                        //     LOG(INFO)<<"untraversable";
                        // }    
                    }
                }
            }
            // LOG(INFO)<<"nodes size: "<<nodes.size();
            // LOG(INFO)<<"cost: "<<currentNodeP->cost;
            if (nodes.empty())
            {
                return false;
            }
            else
            {
                return true;
            }
            
        }
    }
    else
    {
        return false;
    }
}


bool PathPlanning::SqurePoints(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, vector<Eigen::Vector3d> & points)
{
    if(map.isInside(TL) && map.isInside(TR) && map.isInside(BL) && map.isInside(BR))
    {
        grid_map::Index top_left_index, top_right_index, down_right_index, down_left_index;
        map.getIndex(TL, top_left_index);
        map.getIndex(TR, top_right_index);
        map.getIndex(BL, down_left_index);
        map.getIndex(BR, down_right_index);
        vector<cv::Point> rectPoints;
        rectPoints.emplace_back(cv::Point(top_left_index.y(), top_left_index.x()));
        rectPoints.emplace_back(cv::Point(top_right_index.y(), top_right_index.x()));
        rectPoints.emplace_back(cv::Point(down_right_index.y(), down_right_index.x()));
        rectPoints.emplace_back(cv::Point(down_left_index.y(), down_left_index.x()));
        cv::Mat simage = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
        const cv::Point* pts = rectPoints.data(); // 获取顶点数组指针
        int numPoints = rectPoints.size();
        cv::polylines(simage, &pts, &numPoints, 1, true, 255, 2);
        cv::fillPoly(simage, std::vector<std::vector<cv::Point>>{rectPoints}, 255);
        std::vector<cv::Point> whitePixels;
        cv::findNonZero(simage, whitePixels);
        for (auto & p : whitePixels)
        {
            grid_map::Index index(p.y, p.x);
            grid_map::Position3 position;
            if (map.getPosition3("elevation", index, position))
            {
                points.emplace_back(position);
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}

// 由于全局点云在构建时误差存在，导致台阶平面并不能时别，导致这种平面感知的高程图无法用来确定支撑平面。改用传统方法
bool PathPlanning::isTraversbilityTra(Node & node)
{
    if (map.isInside(node.position))
    {
        Eigen::AngleAxisd ax(node.ori, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d mid(node.position.x(), node.position.y(), 0);
        // 求4个子区域
        Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(planning_param.support_area.Up, 0, 0) + mid;
        Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, planning_param.support_area.Left, 0) + mid_top;
        Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - planning_param.support_area.Right, 0) + mid_top;


        Eigen::Vector3d mid_left = ax.toRotationMatrix() * Eigen::Vector3d(0, planning_param.support_area.Left, 0) + mid;
        Eigen::Vector3d mid_right = ax.toRotationMatrix() * Eigen::Vector3d(0, -planning_param.support_area.Right, 0) + mid;

        Eigen::Vector3d mid_button = ax.toRotationMatrix() * Eigen::Vector3d(- planning_param.support_area.Button, 0, 0) + mid;
        Eigen::Vector3d button_left = ax.toRotationMatrix() * Eigen::Vector3d(0, planning_param.support_area.Left, 0) + mid_button;
        Eigen::Vector3d button_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - planning_param.support_area.Right, 0) + mid_button;

        if (map.isInside(top_left.head(2)) && map.isInside(top_right.head(2)) && map.isInside(button_left.head(2)) && map.isInside(button_right.head(2)))
        {
            Eigen::Vector3d mid_top_subregion = ax.toRotationMatrix() * Eigen::Vector3d(planning_param.support_area.Up - 0.08, 0, 0) + mid;
            Eigen::Vector3d top_left_subregion = ax.toRotationMatrix() * Eigen::Vector3d(0, planning_param.support_area.Left - 0.025, 0) + mid_top_subregion;
            Eigen::Vector3d top_right_subregion = ax.toRotationMatrix() * Eigen::Vector3d(0, - (planning_param.support_area.Right - 0.025), 0) + mid_top_subregion;

            Eigen::Vector3d mid_button_subregion = ax.toRotationMatrix() * Eigen::Vector3d(- (planning_param.support_area.Button - 0.04), 0, 0) + mid;
            Eigen::Vector3d button_left_subregion = ax.toRotationMatrix() * Eigen::Vector3d(0, planning_param.support_area.Left - 0.025, 0) + mid_button_subregion;
            Eigen::Vector3d button_right_subregion = ax.toRotationMatrix() * Eigen::Vector3d(0, - (planning_param.support_area.Right - 0.025), 0) + mid_button_subregion;

            vector<Eigen::Vector3d> points;
            if (!SqurePoints(top_left_subregion.head(2), top_right_subregion.head(2), button_left_subregion.head(2), button_right_subregion.head(2), points))
            {
                return false;
            }
            
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            for (auto & point : points)
            {
                sum += point;
            }
            // 如果不行这里还可以用pcl里的ransac算法来拟合平面。可能可以到达更好的效果
            Eigen::Vector3d center = sum / points.size();
            Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
            for (auto & point : points)
            {
                M += (point - center)*(point - center).transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(M);  
            Eigen::Vector3d eigenvalues = es.eigenvalues();  
            Eigen::Matrix3d eigenvectors = es.eigenvectors();
            Eigen::Vector3d plane_normal = Eigen::Vector3d::Zero(); 
            if (eigenvalues(0)/eigenvalues.sum() < 0.1)
            {
                if (eigenvectors.col(0).z() < 0 )
                {
                    plane_normal = - eigenvectors.col(0);
                }
                else
                {
                    plane_normal = eigenvectors.col(0);
                }
                vector<Eigen::Vector3d> all_points;
                Eigen::Vector3d all_mid_top = ax.toRotationMatrix() * Eigen::Vector3d(planning_param.support_area.Up - 0.04, 0, 0) + mid;
                Eigen::Vector3d all_mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- (planning_param.support_area.Button - 0.02), 0, 0) + mid;
                Eigen::Vector3d all_top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, planning_param.support_area.Left, 0) + all_mid_top;
                Eigen::Vector3d all_top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - planning_param.support_area.Right, 0) + all_mid_top;
                Eigen::Vector3d all_down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, planning_param.support_area.Left, 0) + all_mid_down;
                Eigen::Vector3d all_down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - planning_param.support_area.Right, 0) + all_mid_down;
                if (SqurePoints(all_top_left.head(2), all_top_right.head(2), all_down_left.head(2), all_down_right.head(2), all_points))
                {
                    // int max_size = 0;
                    // int above_points = 0;
                    for (auto & point : all_points)
                    {
                        double dis = (point - center).dot(plane_normal);
                        if (dis > 0.03)
                        {
#ifdef DEBUG
                            LOG(INFO)<<"dis is not right";
#endif
                            return false;   
                        }
                    }
                    return true;
                }
                else
                {
#ifdef DEBUG
                    LOG(INFO)<<"all_points is not right";
#endif
                    return false;
                }
            }
            else
            {
#ifdef DEBUG
                LOG(INFO)<<"eigenvalues is not right";
#endif
                return false;
            }
        }
        else
        {
#ifdef DEBUG
            LOG(INFO)<<"corner is not in map";
#endif
            return false;
        }
    }
    else
    {
#ifdef DEBUG
        LOG(INFO)<<"node is not in map";
#endif
        return false;
    }
}

// 把support area根据脚踝点和中心点分为4部分，求四部分的支撑平面并保证一致来确定整个支撑区域的支撑平面，并根据有没有点穿过支撑区域来确定是否可通行
bool PathPlanning::isTraversbility(Node & node)
{
    // drawCandidateSquare(std::make_shared<Node>(node));
    Eigen::AngleAxisd ax(node.ori, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d mid(node.position.x(), node.position.y(), 0);
    // 求4个子区域
    Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(planning_param.support_area.Up, 0, 0) + mid;
    Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, planning_param.support_area.Left, 0) + mid_top;
    Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - planning_param.support_area.Right, 0) + mid_top;


    Eigen::Vector3d mid_left = ax.toRotationMatrix() * Eigen::Vector3d(0, planning_param.support_area.Left, 0) + mid;
    Eigen::Vector3d mid_right = ax.toRotationMatrix() * Eigen::Vector3d(0, -planning_param.support_area.Right, 0) + mid;

    Eigen::Vector3d mid_button = ax.toRotationMatrix() * Eigen::Vector3d(- planning_param.support_area.Button, 0, 0) + mid;
    Eigen::Vector3d button_left = ax.toRotationMatrix() * Eigen::Vector3d(0, planning_param.support_area.Left, 0) + mid_button;
    Eigen::Vector3d button_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - planning_param.support_area.Right, 0) + mid_button;

    if (map.isInside(top_left.head(2)) && map.isInside(top_right.head(2)) && map.isInside(button_left.head(2)) && map.isInside(button_right.head(2)))
    {
        // LOG(INFO)<<"in ...";
        int top_left_support_plane, top_right_support_plane, button_left_support_plane, button_right_support_plane;
        bool top_left_flag = subRegionSupportPlane(top_left.head(2), mid_top.head(2), mid_left.head(2), mid.head(2), top_left_support_plane);
        bool top_right_flag = subRegionSupportPlane(mid_top.head(2), mid_right.head(2), mid.head(2), mid_right.head(2), top_right_support_plane);
        bool button_left_flag = subRegionSupportPlane(mid_left.head(2), mid.head(2), button_left.head(2), mid_button.head(2), button_left_support_plane);
        bool button_right_flag = subRegionSupportPlane(mid.head(2), mid_right.head(2), mid_button.head(2), button_right.head(2), button_right_support_plane);
        // LOG(INFO)<<"in ...";
        // LOG(INFO)<<"top_left_support_plane: "<<top_left_support_plane<<", top_right_support_plane: "<<button_right_support_plane<<", button_left_support_plane: "<<button_left_support_plane<<", button_right_support_plane: "<<button_right_support_plane;
        if (top_left_flag && top_right_flag && button_left_flag && button_right_flag)
        {
            if (top_left_support_plane == top_right_support_plane && top_right_support_plane == button_left_support_plane && button_left_support_plane == button_right_support_plane)
            {
                Eigen::Vector3d normal = pd.planes_info.at(top_left_support_plane).normal;
                Eigen::Vector3d center = pd.planes_info.at(top_left_support_plane).center;
                vector<Eigen::Vector3d> points;
                if (getAllPoints(top_left.head(2), top_right.head(2), button_left.head(2), button_right.head(2), points))
                {
                    for (auto & point : points)
                    {
                        if ((point - center).dot(normal) > 0.01)
                        {
                            // LOG(INFO)<<"not in support plane";
                            // LOG(INFO)<<"normal:"<<normal.transpose();
                            // LOG(INFO)<<"point:"<<point.transpose();
                            // LOG(INFO)<<"center:"<<center.transpose();
                            return false;
                        }
                    }
                    return true;
                }
                else
                {
                    LOG(INFO)<<"can not get all points";
                    return false;
                }
            }
            else
            {
                // LOG(ERROR)<<"SUPPORT PLANE: "<<top_left_support_plane<<" "<<top_right_support_plane<<" "<<button_left_support_plane<<" "<<button_right_support_plane;
                return false;
            }
        }
        else
        {
            // LOG(ERROR)<<"IS SUPPORT: "<<top_left_flag<<" "<<top_right_flag<<" "<<button_left_flag<<" "<<button_right_flag;
            return false;
        }
    }
    else
    {
        // LOG(INFO)<<"corner out of range";
        return false;
    }
}

bool PathPlanning::getAllPoints(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, vector<Eigen::Vector3d> & points)
{
    if (map.isInside(TL) && map.isInside(TR) && map.isInside(BL) && map.isInside(BR))
    {
        grid_map::LineIterator iterator_start(map, BR, BL);
        grid_map::LineIterator iterator_end(map, TR, TL);
        for (; !iterator_start.isPastEnd()&&!iterator_end.isPastEnd(); ++iterator_start, ++iterator_end)
        {
            grid_map::Index start_index(*iterator_start);
            grid_map::Index end_index(*iterator_end);
            for (grid_map::LineIterator iterator_l(map, start_index, end_index); !iterator_l.isPastEnd(); ++iterator_l)
            {
                const grid_map::Index index_l(*iterator_l);
                grid_map::Position3 cor_position;
                if (map.getPosition3("elevation", index_l, cor_position))
                {
                    if (!std::isnan(cor_position.z()))
                    {
                        points.emplace_back(cor_position);
                    }
                }
            }
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool PathPlanning::subRegionSupportPlane(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, int & plane_index)
{
    if (map.isInside(TL) && map.isInside(TR) && map.isInside(BL) && map.isInside(BR))
    {
        std::map<int, int> countMap;
        int NanNumber = 0;
        Eigen::Vector2d mid_point = (TL + TR + BL + BR) / 4;
        grid_map::LineIterator iterator_start(map, BR, BL);
        grid_map::LineIterator iterator_end(map, TR, TL);
        for (; !iterator_start.isPastEnd()&&!iterator_end.isPastEnd(); ++iterator_start, ++iterator_end)
        {
            grid_map::Index start_index(*iterator_start);
            grid_map::Index end_index(*iterator_end);
            for (grid_map::LineIterator iterator_l(map, start_index, end_index); !iterator_l.isPastEnd(); ++iterator_l)
            {
                const grid_map::Index index_l(*iterator_l);
                grid_map::Position3 cor_position;
                if (map.getPosition3("elevation", index_l, cor_position))
                {
                    if (!std::isnan(cor_position.z()))
                    {
                        if (!std::isnan(map["label"](index_l.x(), index_l.y())))
                        {
                            int label_index = static_cast<int>(map["label"](index_l.x(), index_l.y()));
                            // LOG(INFO)<<"label_index: "<<label_index;
                            countMap[label_index]++;
                        }
                        else
                        {
                            NanNumber++;// 编号是nan
                        }
                    }
                    else
                    {
                        NanNumber++; // 点是nan
                    }
                }
                else
                {
                    NanNumber++; // 不能得到此栅格处的点
                }
            }
        }
        if (NanNumber > 8)
        {
#ifdef DEBUG
            LOG(INFO)<<"too many nan";
#endif
            return false;
        }
        // LOG(INFO)<<"NanNumber: "<<NanNumber;
        // for (auto & pair : countMap)
        // {
            // LOG(INFO)<<"pair.first: "<<pair.first<<", pair.second: "<<pair.second;
        // }

        double tmp_height = -std::numeric_limits<double>::infinity();
        int tmp_plane_index = -1;
        // 逻辑注意：一定是先判断最高的平面，再判断最高平面的栅格数是否够
        for (auto & pair : countMap)
        {
            // 问题还是在这，为什么平面终点的高度会是一样的？
            double height = pd.planes_info.at(pair.first).getZ(mid_point);
            // LOG(INFO)<<"mid_point: "<<mid_point.transpose();
            // LOG(INFO)<<"normal: "<<pd.planes_info.at(pair.first).normal.transpose();
            // LOG(INFO)<<"center: "<<pd.planes_info.at(pair.first).center.transpose();
            // LOG(INFO)<<pair.first<<" height: "<<height;
            if (height > tmp_height)
            {
                tmp_plane_index = pair.first;
                tmp_height = height;
            }

            // if (pair.second > support_num_th)
            // {
            //     // LOG(INFO)<<"pair.second: "<<pair.second;
            // }
            // else
            // {
            //     LOG(INFO) << "pair.second: " << pair.second;
            // }
        }

        

        if (tmp_plane_index == -1)
        {
            LOG(INFO) << "No plane found";
            return false;
        }
        else
        {
            if (countMap[tmp_plane_index] > planning_param.support_num_th)
            {
                plane_index = tmp_plane_index;
                // LOG(INFO)<<"plane_index: "<<plane_index;
                return true;
            }
            else
            {
                // LOG(INFO)<<"support area too less";
                return false;
            }
            
        }   
    }
    else
    {
        return false;
    }
    
}

// 对终点所在的终点所在的区域求其方向
bool PathPlanning::CheckTraversability(Node & node)
{
    // 如果位于安全范围内，就是可通行的
    grid_map::Index index;
    if (map.getIndex(node.position, index))
    {
        if (check_Mat.at<uchar>(index.x(), index.y()) == 255)
        {
            return true;
        }
        else
        {
            // 以整个脚板，确定支撑平面，再确定可通行性
            return isTraversbilityTra(node);
        }
    }
    else
    {
        // LOG(INFO)<<"node position is out of map";
        return false;
    }
}

// 根据 引力、终点吸引力确定合力方向，当在终点障碍涡流范围内时，只考虑引力，否则考虑引力+引力对终点吸引力的影响
bool PathPlanning::getYaw(Node & node, double & yaw)
{
    grid_map::Index index;
    if (map.getIndex(node.position, index))
    {
        // cv::imshow("goal_vortex_region", goal_vortex_region);
        // cv::waitKey(0);
        // // 位于终点的涡流域内
        // if (goal_vortex_region.at<uchar>(index(0), index(1)) == 255)
        // {
        //     // 只返回涡流场的力
        //     LOG(INFO)<<"vortex only";
        //     Eigen::Vector2d AttForce;
        //     if (AttPotential(node, AttForce))
        //     {
        //         LOG(INFO)<<"AttForce: "<<AttForce.transpose();
        //         yaw = std::atan2(AttForce(1), AttForce(0)) + M_PI;
        //         return true;
        //     }
        //     else
        //     {
        //         return false;
        //     } 
        // }
        // else
        // {
        //     // 考虑SAPF和终点的合力
        //     Eigen::Vector2d AttForce;
        //     Eigen::Vector2d SAPFForce;
        //     if (AttPotential(node, AttForce))
        //     {
        //         Eigen::Vector2d SAPFForce;
        //         if (getSAPF(node, SAPFForce))
        //         {
        //             yaw = std::atan2(AttForce(1) + SAPFForce(1), AttForce(0) + SAPFForce(0));
        //             return true;
        //         }
        //     }
        // }
        Eigen::Vector2d AttForce;
        Eigen::Vector2d SAPFForce;
        if (AttPotential(node, AttForce))
        {
            Eigen::Vector2d SAPFForce;
            if (getSAPF(node, SAPFForce))
            {
                yaw = std::atan2(AttForce(1) + SAPFForce(1), AttForce(0) + SAPFForce(0));
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }

        return true;
    }
    else
    {
        return false;
    }
}


bool PathPlanning::reachGoal(Node & node)
{
    Eigen::Vector2d diff = goal_.head(2) - node.position;
    return diff.norm() < 0.4;
}


bool PathPlanning::getNodeString(NodePtr node, std::string & str)
{
    str.clear();
    auto iter = node;
    while (iter != nullptr && iter != startP)
    {
        grid_map::Index index;
        if (map.getIndex(iter->position, index))
        {
            string angle = std::to_string((int)(iter->ori * 57.3));
            str += std::to_string(index(0)) + "_" + std::to_string(index(1)) + "_" + angle;
        }
        else
        {
            return false;
        }
        iter = iter->PreFootstepNode;   
    }
    return true;
}


// 图像里的方向与高程图里的方向
bool PathPlanning::plan()
{
    LOG(INFO)<<"using sapf A star";
    auto start = std::chrono::high_resolution_clock::now();
    // debug_image.setTo(cv::Scalar(0, 0, 0), obstacle_layer);
#ifdef SHOW_PLANNING_PROCESS
    LOG(INFO)<<"start planning";
    cv::imshow("debug", debug_image);
    cv::waitKey(0);
#endif
    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNode> minHeap;
    minHeap.push(startP);
    while (!minHeap.empty())
    {
        NodePtr currentNode = minHeap.top();
        // LOG(INFO)<<"currentNode: "<<currentNode->position.transpose();
        cv::Mat tmp_image = debug_image.clone();
        grid_map::Index index;
        map.getIndex(currentNode->position, index);
#ifdef SHOW_PLANNING_PROCESS
        // LOG(INFO)<<"index: "<<index.transpose();
        cv::Point start_point(index(1), index(0));
        cv::Point end_point(index(1) - 10 * std::sin(currentNode->ori), index(0) - 10 * std::cos(currentNode->ori));
        cv::arrowedLine(tmp_image, start_point, end_point, cv::Scalar(255, 0, 0), 2, cv::LINE_AA, 0, 0.3);
        cv::circle(tmp_image, cv::Point(index(1), index(0)), 5, cv::Scalar(255, 0, 0), -1);
        cv::imshow("debug", tmp_image);
        cv::waitKey(0);
#endif
        minHeap.pop();

        string ss;
        if (getNodeString(currentNode, ss))
        {
            if (close_set.find(ss) == close_set.end())
            {
                close_set.insert(ss);
            }
            else
            {
                continue;
            }
        }
        else
        {
            continue;
        }
        if (reachGoal(*currentNode))
        {
            cout << "find goal" << endl;
            Node goal_node;
            goal_node.position = goal_.head(2);
            goal_node.PreFootstepNode = currentNode;
            goal_node.ori = goal_.z();
            NodePtr goal_node_ptr = std::make_shared<Node>(goal_node);
            computePath(goal_node_ptr);
            break;
        }
        double yaw;
        if (getYaw(*currentNode, yaw))
        {
#ifdef DEBUG
            LOG(INFO)<<"yaw: "<<yaw;
#endif
            // 力决定的方向
#ifdef SHOW_PLANNING_PROCESS
            cv::Point direct_start(index(1), index(0));
            cv::Point direct_end(index(1) - 10 * std::sin(yaw), index(0) - 10 * std::cos(yaw));
            cv::arrowedLine(tmp_image, direct_start, direct_end, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.3);
            cv::imshow("debug", tmp_image);
            cv::waitKey(0);
#endif
#ifdef DEBUG
            LOG(INFO)<<"...";
#endif
            std::vector<NodePtr> nodes;
            if (transitions(currentNode, yaw, nodes))
            {
                if (nodes.empty())
                {
                    LOG(ERROR)<<"transitions is empty.";
                }
#ifdef SHOW_PLANNING_PROCESS
                // 所有扩展的nodes
                for (auto & node : nodes)
                {
                    grid_map::Index node_index;
                    map.getIndex(node->position, node_index);
                    cv::circle(tmp_image, cv::Point(node_index(1), node_index(0)), 1, cv::Scalar(0, 255, 0), -1);
                }
                cv::imshow("debug", tmp_image);
                cv::waitKey(0);
#endif
                for (auto & node : nodes)
                {
                    minHeap.push(node);
                }
            }
        }
        else
        {
            LOG(ERROR)<<"can not get yaw";
            return false;
        }
#ifdef DEBUG
        LOG(INFO)<<"******next iteration-------";
#endif
    }
    auto end = std::chrono::high_resolution_clock::now();

    // 计算时间差
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "耗时: " << duration.count() << " 微秒" << std::endl;
    return true;
}

PathPlanning::~PathPlanning()
{
}