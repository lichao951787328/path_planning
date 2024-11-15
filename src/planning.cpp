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
    // strictChecksMat = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    // Full_feasible.setTo(cv::Scalar(255));
#ifdef DEBUG
    // cv::imshow("Full_feasible", Full_feasible);
    // cv::waitKey(0);
#endif
    // checks_Mat.resize(72);// 72 * 5 = 360
    // for (auto & image : checks_Mat)
    // {
    //     image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    // }
    // 这个值的选取是一个非常谨慎的事
    // strict_section = 2;
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

void PathPlanning::constructPlaneAwareMap()
{
    pcl::PointCloud<pcl::PointXYZ> pc = gridMap2Pointcloud(map);
    pcl::io::savePCDFile(package_path + "/data/pc.pcd", pc);

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
    // LOG(INFO)<<"...";
    full_feasible_region = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    int radius = safe_length / planning_param.resolution;
    // LOG(INFO)<<"planning_param.resolution: "<<planning_param.resolution;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(radius*2 + 1, radius*2 + 1), cv::Point(1, 1));
    // cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(radius * 2 + 1, radius * 2 + 1), cv::Point(1, 1));
    // LOG(INFO)<<"...";
    for (int i = 0; i < pd.planes.size(); i++)
    {
        cv::Mat image = pd.planes.at(i).clone();
        // cv::imshow("image", image);
        // cv::waitKey(0);
        // LOG(INFO)<<"...";
        cv::erode(image, image, element);
        // LOG(INFO)<<"...";
        full_feasible_region.setTo(255, image);
        // LOG(INFO)<<"...";
    }
    // LOG(INFO)<<"...";
    full_feasible_region.setTo(0, obstacle_layer);
    // #ifdef DEBUG
    cv::imshow("full_feasible_region", full_feasible_region);
    cv::waitKey(0);
    // #endif
    LOG(INFO)<<"OVER";
}

// void PathPlanning::constructGoalVortexRegion()
// {
//     goal_vortex_region = cv::Mat::zeros(map.getSize()(1), map.getSize()(0), CV_8UC1);
//     if (cv::countNonZero(goal_obstacle) > 0)
//     {
//         cv::Mat element_1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(d_inf_rad*2 + 1, d_inf_rad*2 + 1), cv::Point(1, 1));
//         cv::Mat element_2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(d_safe_rad*2 + 1, d_safe_rad*2 + 1), cv::Point(1, 1));
//         cv::Mat mat_1, mat_2;
//         cv::dilate(goal_obstacle, mat_1, element_1);
//         cv::dilate(goal_obstacle, mat_2, element_2);
//         mat_1.setTo(0, mat_2);
//         goal_vortex_region = mat_1;
//     }
//     else
//     {
//         LOG(INFO)<<"do not have goal vortex region.";
//     }
// }

// 变更思路，不再只对平面的边远区域进行寻找确定障碍，而是除开平面较为内部的区域都寻找，确定障碍区域
void PathPlanning::constructObstacleLayer(int chect_scope)
{
    obstacle_layer = cv::Mat(map.getSize().x(), map.getSize().y(), CV_8UC1, cv::Scalar(255));
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(chect_scope + 1, chect_scope + 1), cv::Point(1, 1));
    for (int i = 0; i < pd.planes.size(); i++)
    {
        cv::Mat image = pd.planes.at(i).clone();
        cv::erode(image, image, element);
        obstacle_layer.setTo(0, image);
    }
    // 找到所有白色像素
    std::vector<cv::Point> white_points;
    cv::findNonZero(obstacle_layer, white_points);
    for (auto cv_point : white_points)
    {
        double max_height = - std::numeric_limits<double>::max();
        double min_height = std::numeric_limits<double>::max();
        if (cv_point.x < 10 || cv_point.y < 10 || cv_point.x > obstacle_layer.cols -10 || cv_point.y > obstacle_layer.rows - 10)
        {
            continue;
        }
        for (int ky = -chect_scope / 2; ky <= chect_scope / 2; ++ky) 
        {
            for (int kx = -chect_scope / 2; kx <= chect_scope / 2; ++kx) 
            {
                int nx = cv_point.x + kx;
                int ny = cv_point.y + ky;
                // 检查邻域内的点是否在图像边界内
                if (nx >= 0 && ny >= 0 && nx < obstacle_layer.cols && ny < obstacle_layer.rows) 
                {
                    grid_map::Position3 p3;
                    grid_map::Position3 plane_label;
                    if (map.getPosition3("elevation", grid_map::Index(ny, nx), p3))
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
        if (abs(max_height - min_height) < 0.3)
        {
            // LOG(INFO)<<max_height<<" "<<min_height;
            obstacle_layer.at<uchar>(cv_point.y, cv_point.x) = 0;
        }
    }
    // cv::imshow("obstacle_layer", obstacle_layer);
    // cv::waitKey(0);
    //     // cv::imshow("image", image);
    //     // cv::waitKey(0);
    //     // 存储轮廓的向量
    //     std::vector<std::vector<cv::Point>> contours;
    //     // 查找所有轮廓，并且保存轮廓上的所有点
    //     cv::findContours(image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    //     int kernelSize = chect_scope; // 定义核的大小，例如3x3
    //     //  定义最大高度，最小高度，及所有平面编号
    //     for (auto & contour : contours)
    //     {
    //         for (auto & cv_point : contour)
    //         {
    //             double max_height = - std::numeric_limits<double>::max();
    //             double min_height = std::numeric_limits<double>::max();
    //             if (cv_point.x < 10 || cv_point.y < 10 || cv_point.x > image.cols -10 || cv_point.y > image.rows - 10)
    //             {
    //                 continue;
    //             }
    //             std::unordered_map<int, Eigen::Vector3d> normals;
    //             for (int ky = -kernelSize / 2; ky <= kernelSize / 2; ++ky) 
    //             {
    //                 for (int kx = -kernelSize / 2; kx <= kernelSize / 2; ++kx) 
    //                 {
    //                     int nx = cv_point.x + kx;
    //                     int ny = cv_point.y + ky;
    //                     // 检查邻域内的点是否在图像边界内
    //                     if (nx >= 0 && ny >= 0 && nx < image.cols && ny < image.rows) 
    //                     {
    //                         grid_map::Position3 p3;
    //                         grid_map::Position3 plane_label;
    //                         if (map.getPosition3("elevation", grid_map::Index(ny, nx), p3))
    //                         {
    //                             // LOG(INFO)<<p3.transpose();
    //                             if (!std::isnan(p3.z()))
    //                             {
    //                                 if (p3.z() > max_height)
    //                                 {
    //                                     max_height = p3.z();
    //                                 }
    //                                 if (p3.z() < min_height)
    //                                 {
    //                                     min_height = p3.z();
    //                                 }
    //                             }
    //                             if (!std::isnan(map["label"](ny, nx)))
    //                             {
    //                                 int label_index = static_cast<int>(map["label"](ny, nx));
    //                                 normals[label_index] = pd.planes_info.at(label_index).normal;
    //                             }
    //                         }
    //                     }
    //                 }
    //             }
    //             // 最大迈不高度不超过0.15
    //             if (abs(max_height - min_height) > 0.3)
    //             {
    //                 // LOG(INFO)<<max_height<<" "<<min_height;
    //                 obstacle_layer.at<uchar>(cv_point.y, cv_point.x) = 255;
    //             }
    //             else
    //             {
    //                 double max_angle = 0;
    //                 for (auto & normal1 : normals)
    //                 {
    //                     for (auto & normal2 : normals)
    //                     {
    //                         double tmp_angle = std::acos(abs(normal1.second.dot(normal2.second)));
    //                         if (tmp_angle > max_angle)
    //                         {
    //                             max_angle = tmp_angle;
    //                         }
    //                     }
    //                 }
    //                 // 大于25度，则认为为障碍
    //                 if (max_angle > 25/57.3)
    //                 {
    //                     obstacle_layer.at<uchar>(cv_point.y, cv_point.x) = 255;
    //                 }
    //                 // else
    //                 // {
    //                 //     // 如果角度和高度差都满足，那证明该点内的各平面是可联通的
    //                 //     for (auto it1 = normals.begin(); it1 != normals.end(); ++it1)
    //                 //     {
    //                 //         auto it2 = it1;  // 从 it1 开始，避免重复的边
    //                 //         for (++it2; it2 != normals.end(); ++it2)
    //                 //         {
    //                 //             if (it1->first != it2->first)
    //                 //             {
    //                 //                 // 这几个平面是相互连通的
    //                 //                 plane_graph.addEdge(it1->first, it2->first);
    //                 //             }
    //                 //         }
    //                 //     }
    //                 // }
    //             }
    //         }
    //     }
    // }
    

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
            // if (cv::countNonZero(temp_image) < (planning_param.obstacle_length * planning_param.obstacle_length)/(planning_param.resolution * planning_param.resolution))
            // {
            //     // 如果是障碍区域，且障碍区域的面积较小，那么将其内部都设置为白色，也就都是障碍区域
            //     int g_nStructBLlementSize = planning_param.obstacle_inflation_radius/planning_param.resolution;
            //     cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * g_nStructBLlementSize + 1, 2 * g_nStructBLlementSize + 1));
            //     cv::dilate(temp_image, temp_image, element);
            //     obstacle_layer.setTo(255, temp_image);
            // }
        }
    }

    LOG(INFO)<<"construct obstacle map finish";
    cv::imshow("obstacle_layer", obstacle_layer);
    cv::waitKey(0);
    // cv::imwrite(package_path + "/data/obstacle_layer.png", obstacle_layer);
    // 障碍的定义：如果保持原来的定义，则适应的场景有限.
    // 如果对每个平面进行膨胀，一层层的膨胀，每个膨胀层都是障碍，那这个点就是障碍
    LOG(INFO)<<"over";
}

// 计算射线和点之间的有向面积（叉积）
float crossProduct(cv::Point2f rayOrigin, cv::Point2f rayDir, cv::Point2f point) 
{
    cv::Point2f relativePoint = point - rayOrigin;
    return rayDir.x * relativePoint.y - rayDir.y * relativePoint.x;
}

// 判断两个点是否位于射线的两侧
bool arePointsOnOppositeSides(cv::Point2f rayOrigin, cv::Point2f rayDir, cv::Point2f pt1, cv::Point2f pt2) 
{
    float cross1 = crossProduct(rayOrigin, rayDir, pt1);
    float cross2 = crossProduct(rayOrigin, rayDir, pt2);

    // 若叉积符号相反，两个点位于射线两侧
    return (cross1 * cross2 < 0);
}

bool arePointsOnOppositeSidesOfLine(cv::Point2f linePt1, cv::Point2f linePt2, cv::Point2f pt1, cv::Point2f pt2) 
{
    float cross1 = crossProduct(linePt1, linePt2 - linePt1, pt1);
    float cross2 = crossProduct(linePt1, linePt2 - linePt1, pt2);

    // 若叉积符号相反，两个点位于连线的两侧
    return (cross1 * cross2 < 0);
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

// std::pair<cv::Mat, cv::Mat> PathPlanning::regionSeg(vector<cv::Point> & contour, cv::Point segPoint, cv::Point2f Dir)
// {
//     cv::Mat region_more = cv::Mat::zeros(region.size(), CV_8UC1);
//     cv::Mat region_less = cv::Mat::zeros(region.size(), CV_8UC1);
//     cv::Point2f segPoint_2f(segPoint.x, segPoint.y);
//     cv::Point2f segPoint_2f_1(segPoint.x + Dir.x, segPoint.y + Dir.y);
// }
double angleBetweenVectors(const cv::Point& v1, const cv::Point& v2) 
{
    double dotProduct = v1.x * v2.x + v1.y * v2.y;
    double magnitudeV1 = std::sqrt(v1.x * v1.x + v1.y * v1.y);
    double magnitudeV2 = std::sqrt(v2.x * v2.x + v2.y * v2.y);
    double cosTheta = dotProduct / (magnitudeV1 * magnitudeV2);
    return std::acos(cosTheta) * 180.0 / CV_PI;
}

// 取处射线前方的一小块
vector<cv::Point> PathPlanning::raySeg(cv::Point2f rayOrigin, vector<cv::Point> & contour)
{
    cv::Mat need = cv::Mat(map.getSize().x(), map.getSize().y(), CV_8UC1, cv::Scalar(0));
    vector<cv::Point> over_contour;
    for (auto &point : contour)
    {
        cv::Point2f point_f(point.x, point.y);
        cv::Point2f vectorToPoint = point_f - rayOrigin;
        // LOG(INFO)<<"ray_Dir: "<<ray_Dir;
        double angle = angleBetweenVectors(ray_Dir, vectorToPoint);
        if (angle < 30)
        {
            need.at<uchar>(point) = 255;
            over_contour.emplace_back(point);
        }
    }
    cv::imshow("need", need);
    cv::waitKey(0);
    return over_contour;
}


// 计算向量叉积
float crossProduct(const cv::Point2f& v1, const cv::Point2f& v2) 
{
    return v1.x * v2.y - v1.y * v2.x;
}


// 将取出的一小块分成左右两部分
std::pair<vector<cv::Point>, vector<cv::Point> > PathPlanning::raySegLeftRight(cv::Point2f rayOrigin, vector<cv::Point> & contour)
{
    cv::Mat need_left = cv::Mat(map.getSize().x(), map.getSize().y(), CV_8UC1, cv::Scalar(0));
    cv::Mat need_right = cv::Mat(map.getSize().x(), map.getSize().y(), CV_8UC1, cv::Scalar(0));
    vector<cv::Point> left_contour, right_contour;
    for (auto & point : contour)
    {
        cv::Point2f point_f(point.x, point.y);
        cv::Point2f vectorToPoint = point_f - rayOrigin;
        float cross = crossProduct(ray_Dir, vectorToPoint);
        if (cross > 0)
        {
            // 在射线的左边
            need_left.at<uchar>(point) = 255;
            left_contour.emplace_back(point);
        }
        else
        {
            // 在射线的右边
            need_right.at<uchar>(point) = 255;
            right_contour.emplace_back(point);
        }
    }
    cv::imshow("need_left", need_left);
    cv::waitKey(0);
    cv::imshow("need_right", need_right);
    cv::waitKey(0);
    return std::make_pair(left_contour, right_contour);
}

// 计算点到射线的距离
double distanceToRay(const cv::Point& rayStart, const cv::Point& rayDirection, const cv::Point& point) 
{
    // 计算点到射线起点的向量
    cv::Point vectorToPoint = point - rayStart;

    // 计算叉积
    int crossProduct = rayDirection.x * vectorToPoint.y - rayDirection.y * vectorToPoint.x;

    // 计算射线方向的长度
    double length = std::sqrt(rayDirection.x * rayDirection.x + rayDirection.y * rayDirection.y);

    // 计算点到射线的垂直距离
    double distance = std::abs(crossProduct) / length;

    return distance;
}

// 找到射线和轮廓最近的点
cv::Point PathPlanning::getNearestPoint(cv::Point2f rayOrigin, vector<cv::Point> & contour)
{
    double min_distance = std::numeric_limits<double>::max();
    cv::Point nearest_point(-1, -1);
    for (auto & point : contour)
    {
        double tmp_distance = distanceToRay(rayOrigin, ray_Dir, point);
        if (tmp_distance < min_distance)
        {
            nearest_point = point;
        }
    }
    return nearest_point;
}

// 将目标点的轮廓和目标点所在区域进行切割
std::pair<cv::Mat, cv::Mat> PathPlanning::getSegMoreAndLess(cv::Point2f rayOrigin, vector<cv::Point> & contour, cv::Point segPoint)
{    
    // 取出轮廓与终点方向相同的一部分
    vector<cv::Point> direct_contour = raySeg(rayOrigin, contour);
    // 将上述一部分分成左右两部分
    std::pair<vector<cv::Point>, vector<cv::Point> > left_right_contour = raySegLeftRight(rayOrigin, direct_contour);
    LOG(INFO)<<"left_right_contour.first.size(): "<<left_right_contour.first.size()<<" left_right_contour.second.size(): "<<left_right_contour.second.size();
    // cv::imshow("left", cv::Mat(left_right_contour.first));
    // cv::waitKey(0);
    // cv::imshow("right", cv::Mat(left_right_contour.second));
    // cv::waitKey(0);
    // 找出左右两部分最近的点
    cv::Point left_nearest_point = getNearestPoint(rayOrigin, left_right_contour.first);
    cv::Point right_nearest_point = getNearestPoint(rayOrigin, left_right_contour.second);
    LOG(INFO)<<"left_nearest_point: "<<left_nearest_point<<" right_nearest_point: "<<right_nearest_point;
    // 左侧对应的是逆时针旋转，右侧对应的是顺时针旋转
    // 找到外轮廓都是逆时针的
    cv::Mat anticlockwise_region = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    cv::Mat clockwise_region = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    int left_index = -1, right_index = -1, seg_index = -1;
    for (int i = 0; i < contour.size(); i++)
    {
        // 找到与目标点最近的点
        if (contour[i] == segPoint)
        {
            seg_index = i;
        }
        // 找到与目标点方向相同的轮廓点
        if (left_nearest_point == contour[i])
        {
            left_index = i;
        }
        if (right_nearest_point == contour[i])
        {
            right_index = i;
        }
    }
    LOG(INFO)<<"left_index: "<<left_index<<" right_index: "<<right_index<<" seg_index: "<<seg_index;
    CHECK(left_index != -1 && right_index != -1 && seg_index != -1);
    if (left_index < seg_index)
    {
        for (size_t i = left_index; i <= seg_index; i++)
        {
            anticlockwise_region.at<uchar>(contour[i]) = 255;
        }
    }
    else
    {
        for (size_t i = left_index; i < contour.size(); i++)
        {
            anticlockwise_region.at<uchar>(contour[i]) = 255;
        }
        for (size_t i = 0; i <= seg_index; i++)
        {
            anticlockwise_region.at<uchar>(contour[i]) = 255;
        }
    }
    if (right_index > seg_index)
    {
        for (size_t i = seg_index + 1; i <= right_index; i++)
        {
            clockwise_region.at<uchar>(contour[i]) = 255;
        }
    }
    else
    {
        for (size_t i = seg_index + 1; i < contour.size(); i++)
        {
            clockwise_region.at<uchar>(contour[i]) = 255;
        }
        for (size_t i = 0; i <= right_index; i++)
        {
            clockwise_region.at<uchar>(contour[i]) = 255;
        }
    }
    cv::imshow("anticlockwise_region", anticlockwise_region);
    cv::waitKey(0);
    cv::imshow("clockwise_region", clockwise_region);
    cv::waitKey(0);
    return std::make_pair(anticlockwise_region, clockwise_region);
}
// region：膨胀之后的轮廓围成的区域
// goal_region：终点所在区域的凹陷的那块区域
std::pair<cv::Mat, cv::Mat> PathPlanning::getSegMoreAndLess(cv::Mat region, cv::Mat goal_region, cv::Point2f Dir)
{ 
    // 表明可能是竖直方向
    if (std::abs(Dir.x) < 1e-4)
    {
        // LOG(INFO)<<"Dir.x is zero.";
        std::priority_queue<cv_point_dis, std::vector<cv_point_dis>, cv_pointDisCompare> p_queue_more;
        std::priority_queue<cv_point_dis, std::vector<cv_point_dis>, cv_pointDisCompare> p_queue_less;
        // cv::Mat color_image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC3);
        // color_image.setTo(cv::Scalar(255, 255, 255), region);
        // cv::circle(color_image, cv::Point(goal_index_.y(), goal_index_.x()), 5, cv::Scalar(255, 0, 0), 1);
        // cv::imshow("color_image", color_image);
        // cv::waitKey(0);
        // 如果终点不在region内部
        if (region.at<uchar>(goal_index_.x(), goal_index_.y()) != 255)
        {
            LOG(INFO)<<"not in region";
            std::vector<std::vector<cv::Point>> single_contours;
            std::vector<cv::Vec4i> single_hierarchy;
            cv::findContours(region, single_contours, single_hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
            cv::Mat above_image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
            cv::Mat less_image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
            cv::Mat more_image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
            for (int i = 0; i < single_contours[0].size(); i++)
            {
                cv::Point2f pt= single_contours[0][i];
                above_image.at<uchar>(single_contours[0][i]) = 255;
                // 同向
                if ((pt.y - goal_index_.x()) * (Dir.y) > 0)
                {
                    above_image.at<uchar>(single_contours[0][i]) = 255;
                    if (pt.x - goal_index_.y() > 0)
                    {
                        cv_point_dis tmpnode;
                        tmpnode.index = i;
                        tmpnode.dis = std::abs(pt.x - goal_index_.y());
                        p_queue_more.push(tmpnode);
                        more_image.at<uchar>(single_contours[0][i]) = 255;
                    }
                    if (pt.x - goal_index_.y() < 0)
                    {
                        less_image.at<uchar>(single_contours[0][i]) = 255;
                        cv_point_dis tmpnode;
                        tmpnode.index = i;
                        tmpnode.dis = std::abs(pt.x - goal_index_.y());
                        p_queue_less.push(tmpnode);
                    } 
                }
            }

            
            // cv::imshow("above_image", above_image);
            // cv::waitKey(0);
            // cv::imshow("less_image", less_image);
            // cv::waitKey(0);
            // cv::imshow("more_image", more_image);
            // cv::waitKey(0);
            int morethan_index1, morethan_index2;
            if (p_queue_more.size() >= 2)
            {
                morethan_index1 = p_queue_more.top().index;
                p_queue_more.pop();
                morethan_index2 = p_queue_more.top().index;
            }
            int lessthan_index1, lessthan_index2;
            if (p_queue_less.size() >= 2)
            {
                lessthan_index1 = p_queue_less.top().index;
                p_queue_less.pop();
                lessthan_index2 = p_queue_less.top().index;
            }
            
            int order_lessthan_index1 = min(lessthan_index1, lessthan_index2);
            int order_lessthan_index2 = max(lessthan_index1, lessthan_index2);

            int order_morethan_index1 = min(morethan_index1, morethan_index2);
            int order_morethan_index2 = max(morethan_index1, morethan_index2);

            cv::Mat more_contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            cv::Mat less_contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            if (order_morethan_index1 > order_lessthan_index1 && order_morethan_index2 < order_lessthan_index2)
            {
                for (int order_moreindex = order_morethan_index1; order_moreindex <= order_morethan_index2; order_moreindex++)
                {
                    more_contour.at<uchar>(single_contours[0].at(order_moreindex)) = 255;
                    // cv::circle(more_contour, single_contours[0].at(order_moreindex), 5, cv::Scalar(255), 1);
                    // cv::imshow("more_contour", more_contour);
                    // cv::waitKey(2);

                }
                for (int order_lessindex = order_lessthan_index2; order_lessindex < single_contours[0].size(); order_lessindex++)
                {
                    less_contour.at<uchar>(single_contours[0].at(order_lessindex)) = 255;
                    // cv::circle(less_contour, single_contours[0].at(order_lessindex), 5, cv::Scalar(255), 1);
                    // cv::imshow("less_contour", less_contour);
                    // cv::waitKey(2);
                }
                for (int order_lessindex = 0; order_lessindex <= order_lessthan_index1; order_lessindex++)
                {
                    less_contour.at<uchar>(single_contours[0].at(order_lessindex)) = 255;
                    // cv::circle(less_contour, single_contours[0].at(order_lessindex), 5, cv::Scalar(255), 1);
                    // cv::imshow("less_contour", less_contour);
                    // cv::waitKey(2);
                }
            }
            else
            {
                for (int order_lessindex = order_lessthan_index1; order_lessindex <= order_lessthan_index2; order_lessindex++)
                {
                    less_contour.at<uchar>(single_contours[0].at(order_lessindex)) = 255;
                    // cv::circle(less_contour, single_contours[0].at(order_lessindex), 5, cv::Scalar(255), 1);
                    // cv::imshow("less_contour", less_contour);
                    // cv::waitKey(2);
                }
                for (int order_moreindex = order_morethan_index2; order_moreindex < single_contours[0].size(); order_moreindex++)
                {
                    more_contour.at<uchar>(single_contours[0].at(order_moreindex)) = 255;
                    // cv::circle(more_contour, single_contours[0].at(order_moreindex), 5, cv::Scalar(255), 1);
                    // cv::imshow("more_contour", more_contour);
                    // cv::waitKey(2);
                }
                for (int order_moreindex = 0; order_moreindex <= order_morethan_index1; order_moreindex++)
                {
                    more_contour.at<uchar>(single_contours[0].at(order_moreindex)) = 255;
                    // cv::circle(more_contour, single_contours[0].at(order_moreindex), 5, cv::Scalar(255), 1);
                    // cv::imshow("more_contour", more_contour);
                    // cv::waitKey(2);
                }
            }
        
            return std::make_pair(less_contour, more_contour);
        }
        else
        {
            // LOG(INFO)<<"IN REGION";
            region.setTo(0, goal_region);
            // cv::imshow("region", region);
            // cv::waitKey(0);
            std::vector<std::vector<cv::Point>> single_contours;
            std::vector<cv::Vec4i> single_hierarchy;
            cv::findContours(region, single_contours, single_hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
            // LOG(INFO)<<single_hierarchy[0][2];
            cv::Mat tmp_image = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            cv::drawContours(tmp_image, single_contours, 0, cv::Scalar(255), 1);
            // cv::imshow("tmp_image", tmp_image);
            // cv::waitKey(0);
            std::priority_queue<cv_point_dis, std::vector<cv_point_dis>, cv_pointDisCompare> p_queue_more;
            std::priority_queue<cv_point_dis, std::vector<cv_point_dis>, cv_pointDisCompare> p_queue_less;

            cv::Mat above_image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
            cv::Mat less_image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
            cv::Mat more_image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);

            for (int i = 0; i < single_contours[0].size(); i++)
            {
                cv::Point2f pt= single_contours[0][i];
                
                // 同向
                if ((pt.y - goal_index_.x()) * (Dir.y) > 0)
                {
                    above_image.at<uchar>(single_contours[0][i]) = 255;
                    if (pt.x - goal_index_.y() > 0)
                    {
                        cv_point_dis tmpnode;
                        tmpnode.index = i;
                        tmpnode.dis = std::abs(pt.x - goal_index_.y());
                        p_queue_more.push(tmpnode);
                        more_image.at<uchar>(single_contours[0][i]) = 255;
                    }
                    if (pt.x - goal_index_.y() < 0)
                    {
                        less_image.at<uchar>(single_contours[0][i]) = 255;
                        cv_point_dis tmpnode;
                        tmpnode.index = i;
                        tmpnode.dis = std::abs(pt.x - goal_index_.y());
                        p_queue_less.push(tmpnode);
                    } 
                }
            }
            // cv::imshow("above_image", above_image);
            // cv::waitKey(0);
            // cv::imshow("less_image", less_image);
            // cv::waitKey(0);
            // cv::imshow("more_image", more_image);
            // cv::waitKey(0);
            if (single_hierarchy[0][2] == -1) // 没有内轮廓
            {
                // LOG(INFO)<<"no inliner";
                int morethan_index1, morethan_index2;
                if (p_queue_more.size() >= 2)
                {
                    morethan_index1 = p_queue_more.top().index;
                    p_queue_more.pop();
                    morethan_index2 = p_queue_more.top().index;
                }
                int lessthan_index1, lessthan_index2;
                if (p_queue_less.size() >= 2)
                {
                    lessthan_index1 = p_queue_less.top().index;
                    p_queue_less.pop();
                    lessthan_index2 = p_queue_less.top().index;
                }

                // 在最近的点中根据大小关系，如果从小到大的过程中需要经过对面的点，则认为需要以反向的方式聚集所有区域
                int order_lessthan_index1 = min(lessthan_index1, lessthan_index2);
                int order_lessthan_index2 = max(lessthan_index1, lessthan_index2);

                int order_morethan_index1 = min(morethan_index1, morethan_index2);
                int order_morethan_index2 = max(morethan_index1, morethan_index2);

                // 根据叉乘大于0还是小于0确定左右侧，确定旋转方向
                cv::Mat more_contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
                cv::Mat less_contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
                if (order_morethan_index1 > order_lessthan_index1 && order_morethan_index2 < order_lessthan_index2)
                {
                    for (int order_moreindex = order_morethan_index1; order_moreindex <= order_morethan_index2; order_moreindex++)
                    {
                        more_contour.at<uchar>(single_contours[0].at(order_moreindex)) = 255;
                        // cv::circle(more_contour, single_contours[0].at(order_moreindex), 5, cv::Scalar(255), 1);
                        // cv::imshow("more_contour", more_contour);
                        // cv::waitKey(2);
                    }
                    for (int order_lessindex = order_lessthan_index2; order_lessindex < single_contours[0].size(); order_lessindex++)
                    {
                        less_contour.at<uchar>(single_contours[0].at(order_lessindex)) = 255;
                        // cv::circle(less_contour, single_contours[0].at(order_lessindex), 5, cv::Scalar(255), 1);
                        // cv::imshow("less_contour", less_contour);
                        // cv::waitKey(2);
                    }
                    for (int order_lessindex = 0; order_lessindex <= order_lessthan_index1; order_lessindex++)
                    {
                        less_contour.at<uchar>(single_contours[0].at(order_lessindex)) = 255;
                        // cv::circle(less_contour, single_contours[0].at(order_lessindex), 5, cv::Scalar(255), 1);
                        // cv::imshow("less_contour", less_contour);
                        // cv::waitKey(2);
                    }
                }
                else
                {
                    for (int order_lessindex = order_lessthan_index1; order_lessindex <= order_lessthan_index2; order_lessindex++)
                    {
                        less_contour.at<uchar>(single_contours[0].at(order_lessindex)) = 255;
                        // cv::circle(less_contour, single_contours[0].at(order_lessindex), 5, cv::Scalar(255), 1);
                        // cv::imshow("less_contour", less_contour);
                        // cv::waitKey(2);
                    }
                    for (int order_moreindex = order_morethan_index2; order_moreindex < single_contours[0].size(); order_moreindex++)
                    {
                        more_contour.at<uchar>(single_contours[0].at(order_moreindex)) = 255;
                        // cv::circle(more_contour, single_contours[0].at(order_moreindex), 5, cv::Scalar(255), 1);
                        // cv::imshow("more_contour", more_contour);
                        // cv::waitKey(2);
                    }
                    for (int order_moreindex = 0; order_moreindex <= order_morethan_index1; order_moreindex++)
                    {
                        more_contour.at<uchar>(single_contours[0].at(order_moreindex)) = 255;
                        // cv::circle(more_contour, single_contours[0].at(order_moreindex), 5, cv::Scalar(255), 1);
                        // cv::imshow("more_contour", more_contour);
                        // cv::waitKey(2);
                    }
                }
                // cv::imshow("less_contour", less_contour);
                // cv::waitKey(0);
                // cv::imshow("more_contour", more_contour);
                // cv::waitKey(0);
                return std::make_pair(less_contour, more_contour);
            }
            else // 有内轮廓
            {
                // LOG(INFO)<<"INLINE";
                int lessthan_index1 = p_queue_less.top().index;
                int morethan_index1 = p_queue_more.top().index;
                // 如果有内轮廓,从lessthan_index1 和 morethan_index1 往两侧扩展
                cv::Mat more_contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
                cv::Mat less_contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
                if (lessthan_index1 < morethan_index1)
                {
                    for (int k = 0; k < single_contours[0].size()/2; k++)
                    {
                        int index = lessthan_index1 - k >= 0 ? lessthan_index1 - k : lessthan_index1 - k + single_contours[0].size();
                        less_contour.at<uchar>(single_contours[0].at(index)) = 255;
                        // cv::circle(less_contour, single_contours[0].at(index), 5, cv::Scalar(255), 1);
                        // cv::imshow("less_contour", less_contour);
                        // cv::waitKey(2);

                        index = morethan_index1 + k < single_contours[0].size() ? morethan_index1 + k : morethan_index1 + k - single_contours[0].size();
                        more_contour.at<uchar>(single_contours[0].at(index)) = 255;
                        // cv::circle(more_contour, single_contours[0].at(index), 5, cv::Scalar(255), 1);
                        // cv::imshow("more_contour", more_contour);
                        // cv::waitKey(2);
                    }
                    
                }
                else
                {
                    for (int k = 0; k < single_contours[0].size()/2; k++)
                    {
                        int index = morethan_index1 - k >= 0 ? morethan_index1 - k : morethan_index1 - k + single_contours[0].size();
                        
                        more_contour.at<uchar>(single_contours[0].at(index)) = 255;
                        // cv::circle(more_contour, single_contours[0].at(index), 5, cv::Scalar(255), 1);
                        // cv::imshow("more_contour", more_contour);
                        // cv::waitKey(2);

                        index = lessthan_index1 + k < single_contours[0].size() ? lessthan_index1 + k : lessthan_index1 + k - single_contours[0].size();
                        less_contour.at<uchar>(single_contours[0].at(index)) = 255;
                        // cv::circle(less_contour, single_contours[0].at(index), 5, cv::Scalar(255), 1);
                        // cv::imshow("less_contour", less_contour);
                        // cv::waitKey(2);
                    }
                }  
                // cv::imshow("less_contour", less_contour);
                // cv::waitKey(0);
                // cv::imshow("more_contour", more_contour);
                // cv::waitKey(0);  
                return std::make_pair(less_contour, more_contour);
            }
        }
    }
    else // 针对方向不是竖直的情况
    {
        // 如果终点不在region内部
        if (region.at<uchar>(goal_index_.x(), goal_index_.y()) != 255)
        {
            LOG(INFO)<<"not in region";
            std::vector<std::vector<cv::Point>> single_contours;
            std::vector<cv::Vec4i> single_hierarchy;
            cv::findContours(region, single_contours, single_hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);

            vector<int> morethanZeroSide, lessthanZeroSide;

            for (int i = 0; i < single_contours[0].size(); i++)
            {
                cv::Point2f pt1 = single_contours[0][i%single_contours[0].size()];
                cv::Point2f pt2 = single_contours[0][(i + 1)%single_contours[0].size()];
                if (arePointsOnOppositeSides(cv::Point2f(goal_index_.y(), goal_index_.x()), ray_Dir, pt1, pt2))  
                {
                    std::cout << "这两个点位于射线的两侧" << std::endl;
                    // 判断这两个点是否是射线同向的方向
                    // 如果两者之积大于0，证明x方向同向，即可得出整个同向
                    if ( (pt1.x-goal_index_.y()) * ray_Dir.x  > 0) // 表明同向
                    {
                        if (crossProduct(cv::Point2f(goal_index_.y(), goal_index_.x()), ray_Dir, pt1) > 0)
                        {
                            morethanZeroSide.emplace_back(i%single_contours[0].size());
                        }
                        else
                        {
                            lessthanZeroSide.emplace_back(i%single_contours[0].size());
                        }
                        if (crossProduct(cv::Point2f(goal_index_.y(), goal_index_.x()), ray_Dir, pt2) > 0)
                        {
                            morethanZeroSide.emplace_back((i + 1)%single_contours[0].size());
                        }
                        else
                        {
                            lessthanZeroSide.emplace_back((i + 1)%single_contours[0].size());
                        }
                    }
                }
            }
            CHECK(morethanZeroSide.size() >= 2);
            CHECK(lessthanZeroSide.size() >= 2);
            // 找出距离最近的两个点，没有覆盖终点的情况下
            std::priority_queue<cv_point_dis, std::vector<cv_point_dis>, cv_pointDisCompare> p_queue;
            // 这里用的距离是像素距离
            Eigen::Vector2i goal_cv(goal_index_.y(), goal_index_.x());
            for (auto & index : morethanZeroSide)
            {
                Eigen::Vector2i point(single_contours[0][index].x, single_contours[0][index].y);
                double dis = (point - goal_cv).norm();
                cv_point_dis tmpnode;
                tmpnode.index = index;
                tmpnode.dis = dis;
                p_queue.push(tmpnode);
            }
            // cv::Mat more2Image = cv::Mat::zeros(obstacle_layer.size(), CV_8UC3);
            // cv::drawContours(more2Image, single_contours, 0, cv::Scalar(255, 0  ,0), 1);
            int morethan_index1 = p_queue.top().index;
            // cv::circle(more2Image, single_contours[0].at(morethan_index1), 5, cv::Scalar(255, 0, 0), 1);
            // cv::imshow("more2Image", more2Image);
            // cv::waitKey(0);
            p_queue.pop();
            int morethan_index2 = p_queue.top().index;
            // cv::circle(more2Image, single_contours[0].at(morethan_index2), 5, cv::Scalar(255, 0, 0), 1);
            // cv::imshow("more2Image", more2Image);
            // cv::waitKey(0);
            while(!p_queue.empty()) { //只要队列不为空 
                p_queue.pop();  //一直出队 
            }  
            for (auto & index : lessthanZeroSide)
            {
                Eigen::Vector2i point(single_contours[0][index].x, single_contours[0][index].y);
                double dis = (point - goal_cv).norm();
                cv_point_dis tmpnode;
                tmpnode.index = index;
                tmpnode.dis = dis;
                p_queue.push(tmpnode);
            }
            // cv::Mat less2Image = cv::Mat::zeros(obstacle_layer.size(), CV_8UC3);
            // cv::drawContours(less2Image, single_contours, 0, cv::Scalar(255, 0  ,0), 1);
            int lessthan_index1 = p_queue.top().index;
            // cv::circle(less2Image, single_contours[0].at(lessthan_index1), 5, cv::Scalar(255, 0, 0), 1);
            // cv::imshow("less2Image", less2Image);
            // cv::waitKey(0);
            p_queue.pop();
            int lessthan_index2 = p_queue.top().index;
            // cv::circle(less2Image, single_contours[0].at(lessthan_index2), 5, cv::Scalar(255, 0, 0), 1);
            // cv::imshow("less2Image", less2Image);
            // cv::waitKey(0);
            while(!p_queue.empty()) { //只要队列不为空 
                p_queue.pop();  //一直出队 
            }

            int order_lessthan_index1 = min(lessthan_index1, lessthan_index2);
            int order_lessthan_index2 = max(lessthan_index1, lessthan_index2);

            int order_morethan_index1 = min(morethan_index1, morethan_index2);
            int order_morethan_index2 = max(morethan_index1, morethan_index2);


            cv::Mat more_contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            cv::Mat less_contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            if (order_morethan_index1 > order_lessthan_index1 && order_morethan_index2 < order_lessthan_index2)
            {
                for (int order_moreindex = order_morethan_index1; order_moreindex <= order_morethan_index2; order_moreindex++)
                {
                    more_contour.at<uchar>(single_contours[0].at(order_moreindex)) = 255;
                    // cv::circle(more_contour, single_contours[0].at(order_moreindex), 5, cv::Scalar(255), 1);
                    // cv::imshow("more_contour", more_contour);
                    // cv::waitKey(2);

                }
                for (int order_lessindex = order_lessthan_index2; order_lessindex < single_contours[0].size(); order_lessindex++)
                {
                    less_contour.at<uchar>(single_contours[0].at(order_lessindex)) = 255;
                    // cv::circle(less_contour, single_contours[0].at(order_lessindex), 5, cv::Scalar(255), 1);
                    // cv::imshow("less_contour", less_contour);
                    // cv::waitKey(2);
                }
                for (int order_lessindex = 0; order_lessindex <= order_lessthan_index1; order_lessindex++)
                {
                    less_contour.at<uchar>(single_contours[0].at(order_lessindex)) = 255;
                    // cv::circle(less_contour, single_contours[0].at(order_lessindex), 5, cv::Scalar(255), 1);
                    // cv::imshow("less_contour", less_contour);
                    // cv::waitKey(2);
                }
            }
            else
            {
                for (int order_lessindex = order_lessthan_index1; order_lessindex <= order_lessthan_index2; order_lessindex++)
                {
                    less_contour.at<uchar>(single_contours[0].at(order_lessindex)) = 255;
                    // cv::circle(less_contour, single_contours[0].at(order_lessindex), 5, cv::Scalar(255), 1);
                    // cv::imshow("less_contour", less_contour);
                    // cv::waitKey(2);
                }
                for (int order_moreindex = order_morethan_index2; order_moreindex < single_contours[0].size(); order_moreindex++)
                {
                    more_contour.at<uchar>(single_contours[0].at(order_moreindex)) = 255;
                    // cv::circle(more_contour, single_contours[0].at(order_moreindex), 5, cv::Scalar(255), 1);
                    // cv::imshow("more_contour", more_contour);
                    // cv::waitKey(2);
                }
                for (int order_moreindex = 0; order_moreindex <= order_morethan_index1; order_moreindex++)
                {
                    more_contour.at<uchar>(single_contours[0].at(order_moreindex)) = 255;
                    // cv::circle(more_contour, single_contours[0].at(order_moreindex), 5, cv::Scalar(255), 1);
                    // cv::imshow("more_contour", more_contour);
                    // cv::waitKey(2);
                }
            }
            
            return std::make_pair(less_contour, more_contour);
        }
        else // 如果终点在region内
        {   
            LOG(INFO)<<"in region";  
            region.setTo(0, goal_region);
            cv::imshow("region", region);
            cv::waitKey(0);
            std::vector<std::vector<cv::Point>> single_contours;
            std::vector<cv::Vec4i> single_hierarchy;
            cv::findContours(region, single_contours, single_hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
            LOG(INFO)<<"....";
            cv::Mat tmp_image = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            cv::drawContours(tmp_image, single_contours, 0, cv::Scalar(255), 1);
            cv::imshow("tmp_image", tmp_image);
            cv::waitKey(0);
            vector<int> morethanZeroSide, lessthanZeroSide;
            // 遍历最外层轮廓的点
            for (int j = 0; j < single_contours[0].size(); j++)
            {
                cv::Mat contourImage = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
                cv::Point2f pt1 = single_contours[0][j%single_contours[0].size()];
                cv::Point2f pt2 = single_contours[0][(j + 1)%single_contours[0].size()];
                cv::circle(contourImage, pt1, 5, cv::Scalar(255), 1);
                cv::circle(contourImage, pt2, 5, cv::Scalar(255), 1);
                cv::imshow("contourImage", contourImage);
                cv::waitKey(0);
                LOG(INFO)<<"ray_Dir: "<<ray_Dir;
                if (arePointsOnOppositeSides(cv::Point2f(goal_index_.y(), goal_index_.x()), ray_Dir, pt1, pt2))  
                {
                    std::cout << "这两个点位于射线的两侧" << std::endl;
                    // 判断这两个点是否是射线同向的方向
                    // 如果两者之积大于0，证明x方向同向，即可得出整个同向
                    // 这里要注意：可能某个像素正好与射线起点的x一样，导致下述的判断方法失效
                    // if (abs(ray_Dir.x) < 1e-4 && )
                    // {
                    //     /* code */
                    // }
                    
                    if ((pt1.y-goal_index_.x()) * ray_Dir.y  > 0) // 表明同向
                    {
                        
                        if (crossProduct(cv::Point2f(goal_index_.y(), goal_index_.x()), ray_Dir, pt1) > 0)
                        {
                            morethanZeroSide.emplace_back(j%single_contours[0].size());
                        }
                        if (crossProduct(cv::Point2f(goal_index_.y(), goal_index_.x()), ray_Dir, pt1) < 0)
                        {
                            lessthanZeroSide.emplace_back(j%single_contours[0].size());
                        }

                        if (crossProduct(cv::Point2f(goal_index_.y(), goal_index_.x()), ray_Dir, pt2) > 0)
                        {
                            morethanZeroSide.emplace_back((j + 1)%single_contours[0].size());
                        }
                        if (crossProduct(cv::Point2f(goal_index_.y(), goal_index_.x()), ray_Dir, pt2) < 0)
                        {
                            lessthanZeroSide.emplace_back((j + 1)%single_contours[0].size());
                        }
                    }
                }
            }
            LOG(INFO)<<"....";
            LOG(INFO)<<morethanZeroSide.size()<<" "<<lessthanZeroSide.size();

            // 如果膨胀没有到覆盖终点，就按原来的方向继续，如果到了终点，则根据原来的凹区域只选择一部分
            // 选择最近的点，根据原轮廓方向及是左端还是右端，
            std::priority_queue<cv_point_dis, std::vector<cv_point_dis>, cv_pointDisCompare> p_queue;
            // 这里用的距离是像素距离
            Eigen::Vector2i goal_cv(goal_index_.y(), goal_index_.x());
            for (auto & index : morethanZeroSide)
            {
                Eigen::Vector2i point(single_contours[0][index].x, single_contours[0][index].y);
                double dis = (point - goal_cv).norm();
                cv_point_dis tmpnode;
                tmpnode.index = index;
                tmpnode.dis = dis;
                p_queue.push(tmpnode);
            }
            LOG(INFO)<<"....";
            LOG(INFO)<<p_queue.size();
            // cv::Mat more2Image = cv::Mat::zeros(obstacle_layer.size(), CV_8UC3);
            // cv::drawContours(more2Image, single_contours, 0, cv::Scalar(255, 0  ,0), 1);
            int morethan_index1 = p_queue.top().index;
            LOG(INFO)<<"....";
            // cv::circle(more2Image, single_contours[0].at(morethan_index1), 5, cv::Scalar(255, 0, 0), 1);
            // cv::imshow("more2Image", more2Image);
            // cv::waitKey(0);
            p_queue.pop();
            int morethan_index2 = p_queue.top().index;
            LOG(INFO)<<"....";
            // cv::circle(more2Image, single_contours[0].at(morethan_index2), 5, cv::Scalar(255, 0, 0), 1);
            // cv::imshow("more2Image", more2Image);
            // cv::waitKey(0);
            while(!p_queue.empty()) { //只要队列不为空 
                p_queue.pop();  //一直出队 
            } 
            LOG(INFO)<<"....";

            for (auto & index : lessthanZeroSide)
            {
                Eigen::Vector2i point(single_contours[0][index].x, single_contours[0][index].y);
                double dis = (point - goal_cv).norm();
                cv_point_dis tmpnode;
                tmpnode.index = index;
                tmpnode.dis = dis;
                p_queue.push(tmpnode);
            }
            // cv::Mat less2Image = cv::Mat::zeros(obstacle_layer.size(), CV_8UC3);
            // cv::drawContours(less2Image, single_contours, 0, cv::Scalar(255, 0  ,0), 1);
            int lessthan_index1 = p_queue.top().index;
            // cv::circle(less2Image, single_contours[0].at(lessthan_index1), 5, cv::Scalar(255, 0, 0), 1);
            // cv::imshow("less2Image", less2Image);
            // cv::waitKey(0);
            p_queue.pop();
            int lessthan_index2 = p_queue.top().index;
            // cv::circle(less2Image, single_contours[0].at(lessthan_index2), 5, cv::Scalar(255, 0, 0), 1);
            // cv::imshow("less2Image", less2Image);
            // cv::waitKey(0);
            while(!p_queue.empty()) { //只要队列不为空 
                p_queue.pop();  //一直出队 
            } 
            LOG(INFO)<<"....";

            // 如果去除后的区域轮廓没有内轮廓
            if (single_hierarchy[0][2] == -1)
            {
                // 在最近的点中根据大小关系，如果从小到大的过程中需要经过对面的点，则认为需要以反向的方式聚集所有区域
                int order_lessthan_index1 = min(lessthan_index1, lessthan_index2);
                int order_lessthan_index2 = max(lessthan_index1, lessthan_index2);

                int order_morethan_index1 = min(morethan_index1, morethan_index2);
                int order_morethan_index2 = max(morethan_index1, morethan_index2);

                // 根据叉乘大于0还是小于0确定左右侧，确定旋转方向
                cv::Mat more_contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
                cv::Mat less_contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
                if (order_morethan_index1 > order_lessthan_index1 && order_morethan_index2 < order_lessthan_index2)
                {
                    for (int order_moreindex = order_morethan_index1; order_moreindex <= order_morethan_index2; order_moreindex++)
                    {
                        more_contour.at<uchar>(single_contours[0].at(order_moreindex)) = 255;
                        // cv::circle(more_contour, single_contours[0].at(order_moreindex), 5, cv::Scalar(255), 1);
                        // cv::imshow("more_contour", more_contour);
                        // cv::waitKey(2);
                    }
                    for (int order_lessindex = order_lessthan_index2; order_lessindex < single_contours[0].size(); order_lessindex++)
                    {
                        less_contour.at<uchar>(single_contours[0].at(order_lessindex)) = 255;
                        // cv::circle(less_contour, single_contours[0].at(order_lessindex), 5, cv::Scalar(255), 1);
                        // cv::imshow("less_contour", less_contour);
                        // cv::waitKey(2);
                    }
                    for (int order_lessindex = 0; order_lessindex <= order_lessthan_index1; order_lessindex++)
                    {
                        less_contour.at<uchar>(single_contours[0].at(order_lessindex)) = 255;
                        // cv::circle(less_contour, single_contours[0].at(order_lessindex), 5, cv::Scalar(255), 1);
                        // cv::imshow("less_contour", less_contour);
                        // cv::waitKey(2);
                    }
                }
                else
                {
                    for (int order_lessindex = order_lessthan_index1; order_lessindex <= order_lessthan_index2; order_lessindex++)
                    {
                        less_contour.at<uchar>(single_contours[0].at(order_lessindex)) = 255;
                        // cv::circle(less_contour, single_contours[0].at(order_lessindex), 5, cv::Scalar(255), 1);
                        // cv::imshow("less_contour", less_contour);
                        // cv::waitKey(2);
                    }
                    for (int order_moreindex = order_morethan_index2; order_moreindex < single_contours[0].size(); order_moreindex++)
                    {
                        more_contour.at<uchar>(single_contours[0].at(order_moreindex)) = 255;
                        // cv::circle(more_contour, single_contours[0].at(order_moreindex), 5, cv::Scalar(255), 1);
                        // cv::imshow("more_contour", more_contour);
                        // cv::waitKey(2);
                    }
                    for (int order_moreindex = 0; order_moreindex <= order_morethan_index1; order_moreindex++)
                    {
                        more_contour.at<uchar>(single_contours[0].at(order_moreindex)) = 255;
                        // cv::circle(more_contour, single_contours[0].at(order_moreindex), 5, cv::Scalar(255), 1);
                        // cv::imshow("more_contour", more_contour);
                        // cv::waitKey(2);
                    }
                }
                return std::make_pair(less_contour, more_contour);
            }
            else // 有内轮廓
            {
                // 如果有内轮廓,从lessthan_index1 和 morethan_index1 往两侧扩展
                cv::Mat more_contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
                cv::Mat less_contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
                if (lessthan_index1 < morethan_index1)
                {
                    for (int k = 0; k < single_contours[0].size()/2; k++)
                    {
                        int index = lessthan_index1 - k >= 0 ? lessthan_index1 - k : lessthan_index1 - k + single_contours[0].size();
                        less_contour.at<uchar>(single_contours[0].at(index)) = 255;
                        // cv::circle(less_contour, single_contours[0].at(index), 5, cv::Scalar(255), 1);
                        // cv::imshow("less_contour", less_contour);
                        // cv::waitKey(2);

                        index = morethan_index1 + k < single_contours[0].size() ? morethan_index1 + k : morethan_index1 + k - single_contours[0].size();
                        more_contour.at<uchar>(single_contours[0].at(index)) = 255;
                        // cv::circle(more_contour, single_contours[0].at(index), 5, cv::Scalar(255), 1);
                        // cv::imshow("more_contour", more_contour);
                        // cv::waitKey(2);
                    }
                    
                }
                else
                {
                    for (int k = 0; k < single_contours[0].size()/2; k++)
                    {
                        int index = morethan_index1 - k >= 0 ? morethan_index1 - k : morethan_index1 - k + single_contours[0].size();
                        
                        more_contour.at<uchar>(single_contours[0].at(index)) = 255;
                        // cv::circle(more_contour, single_contours[0].at(index), 5, cv::Scalar(255), 1);
                        // cv::imshow("more_contour", more_contour);
                        // cv::waitKey(2);

                        index = lessthan_index1 + k < single_contours[0].size() ? lessthan_index1 + k : lessthan_index1 + k - single_contours[0].size();
                        less_contour.at<uchar>(single_contours[0].at(index)) = 255;
                        // cv::circle(less_contour, single_contours[0].at(index), 5, cv::Scalar(255), 1);
                        // cv::imshow("less_contour", less_contour);
                        // cv::waitKey(2);
                    }
                }    
                return std::make_pair(less_contour, more_contour);
            }
        }

    }
}

void PathPlanning::showRepImage()
{
    // LOG(INFO)<<map.getSize().transpose();
    // matplotlibcpp::figure_size(map.getSize().y(), map.getSize().x());
    // // for (int i = 0; i < map.getSize().x(); i++)
    // // {
    // //     matplotlibcpp::plot({0, map.getSize().y()}, {i, i}, "k-");
    // // }
    // // for (int i = 0; i <= map.getSize().y(); ++i) 
    // // {
    // //     // 画纵线
    // //     matplotlibcpp::plot({i, i}, {0, map.getSize().x()}, "k-");
    // // }

    // // matplotlibcpp::xlim(0, map.getSize().y());
    // // matplotlibcpp::ylim(0, map.getSize().x());
    // // matplotlibcpp::xticks(matplotlibcpp::linspace(0, map.getSize().y(), map.getSize().y() + 1));
    // // matplotlibcpp::yticks(matplotlibcpp::linspace(0, map.getSize().x(), map.getSize().x() + 1)); 
    // matplotlibcpp::show();

    cv::Mat RefImage = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            grid_map::Index index(i, j);
            grid_map::Position3 p3x, p3y;
            if (map.getPosition3("SAPF_X", index, p3x) && map.getPosition3("SAPF_Y", index, p3y))
            {
                if (p3x.z() != 0 || p3y.z() != 0)
                {
                    // LOG(INFO)<<index.transpose();
                    cv::Point start(j, i);
                    cv::Point end(start.x + p3x.z() * 5, start.y + p3y.z() * 5);
                    cv::arrowedLine(RefImage, start, end, cv::Scalar(255), 2, 8, 0, 0.1);
                    // RefImage.at<uchar>(i, j) = 255;
                }
            }
        }
    }
    cv::imshow("RefImage", RefImage);
    // cv::imwrite("/home/lichao/catkin_pathplanning/src/path_planning/data/RefImage.png", RefImage);
    cv::waitKey(0);
}

// void PathPlanning::showSAPFImage()
// {
//     for (int obstacle_index = 0; i < count; i++)
//     {
//         /* code */
//     }
//     int index = 0;
//     std::vector<double> x_start, y_start, u, v;
//     for (int i = 0; i < map.getSize().x(); i++)
//     {
//         for (int j = 0; j < map.getSize().y(); j++)
//         {
//             if (index%8 == 0)
//             {
//                 if (map[hull_obstacle_string](i, j) != 0)
//                 {
//                     grid_map::Index index(i, j);
//                     grid_map::Position p2;
//                     if (map.getPosition(index, p2))
//                     {
//                         x_start.emplace_back(p2.x());
//                         y_start.emplace_back(p2.y());
//                         u.emplace_back(map["SAPF_X"](index.x(), index.y()));
//                         v.emplace_back(map["SAPF_Y"](index.x(), index.y()));
//                     }
//                 }
//             }
//             index ++;
//         }
//     }
//     matplotlibcpp::quiver(x_start, y_start, u, v);
//     matplotlibcpp::axis("equal");
//     // plt::xlim(4, 0);
//     matplotlibcpp::show();
// }

// 注意要去除一些小的噪声障碍点，这些点是由于噪声引起，但是不能为此障碍附加势场
void PathPlanning::computeObstacles()
{
    LOG(INFO)<<"computeObstacles";
    // cv::Mat obstacle_layer = getObstacleLayer(param.obstacle_rad);
    // cv::Mat check_image = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
    // std::vector<std::vector<cv::Point>> out_contours;
    // std::vector<cv::Vec4i> out_hierarchy;"
    check_image = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> out_contours;
    std::vector<cv::Vec4i> out_hierarchy;
    cv::findContours(obstacle_layer, out_contours, out_hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    for (auto & contour :  out_contours)
    {
        std::vector<cv::Point> hull;
        // 对每个轮廓计算凸包
        cv::convexHull(contour, hull);
        // 使用pointPolygonTest判断点是否在凸包内
        double result = cv::pointPolygonTest(hull, cv::Point2f(goal_index_.y(), goal_index_.x()), false);
        if (result < 0) // 不在凸包内的一般障碍
        {
            cv::Mat obstacle = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            // 对一障碍，不要凸包也可以
            std::vector<std::vector<cv::Point>> Contours = {hull};
            cv::drawContours(obstacle, Contours, 0, cv::Scalar(255), cv::FILLED);

            // 如果障碍太小，就不能加入
            std::vector<cv::Point> whitePixels;
            cv::findNonZero(obstacle, whitePixels);
            if (whitePixels.size() < 35)
            {
                continue;
            }
            

            obstacles.emplace_back(obstacle);
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
        else // 
        {
            // LOG(INFO)<<"goal in obstacle";
            // 如果有终点在障碍内的情况，添加此处标记
            // map.add("SAPF_OBSTACLE_GOAL", 0);
            goal_obstacle = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            // cv::imshow("goal_obstacle", goal_obstacle);
            // cv::waitKey(0);
            std::vector<std::vector<cv::Point>> Contours = {contour};
            // LOG(INFO)<<"goal in obstacle";
            
            // LOG(INFO)<<"mask obstacle";
            // cv::imshow("goal_obstacle", goal_obstacle);
            // cv::waitKey(0);

            // 这样将凹区域也涂上，表明内部也不会被赋值
            // cv::convexHull(obstacle, hull);
            std::vector<std::vector<cv::Point>> hullContours = {hull};
            cv::drawContours(check_image, hullContours, 0, cv::Scalar(255), cv::FILLED);

            std::vector<cv::Point> whitePixels;
            cv::findNonZero(check_image, whitePixels);
            if (whitePixels.size() < 35)
            {
                continue;
            }
            // 终点障碍区域
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
            // 将原始轮廓区域去除
            cv::drawContours(hull_region, Contours, 0, cv::Scalar(0), cv::FILLED);
#ifdef DEBUG 
            LOG(INFO)<<"goal in obstacle";
            cv::imshow("hull_region", hull_region);
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
            cv::Point2f tangant = contour[0] - contour[1];
            // cv::Point2f normal(center.y, -center.x);
            seg_normal = cv::Point2f(-tangant.y, tangant.x);
            // // 计算轮廓的最小外接矩形
            // cv::RotatedRect minRect = cv::minAreaRect(contour);

            // // 获取最小外接矩形的中心点和角度
            // cv::Point2f center = minRect.center;
            // float angle = minRect.angle;
            // // 计算垂线的方向
            // float perpendicular_angle = angle + 90.0f;

            // // 计算垂线的两个端点
            // int length = planning_param.d_noinflu_rad; // 垂线的长度
            // cv::Point2f start_point(center.x - length * cos(perpendicular_angle * CV_PI / 180.0),
            //                         center.y - length * sin(perpendicular_angle * CV_PI / 180.0));
            // cv::Point2f end_point(center.x + length * cos(perpendicular_angle * CV_PI / 180.0),
            //                     center.y + length * sin(perpendicular_angle * CV_PI / 180.0));
            // // 在原图上绘制垂线
            // cv::line(line, start_point, end_point, cv::Scalar(255), 2);

            cv::Moments m = cv::moments(line, true);
            double cX = m.m10 / m.m00;
            double cY = m.m01 / m.m00;
            seg_point = cv::Point(cX, cY);
            cv::Point2f centroid(cX, cY);
#ifdef DEBUG
            cv::Mat center_image = cv::Mat::zeros(obstacle_layer.size(), CV_8UC3);
            center_image.setTo(cv::Scalar(255, 0, 0), line);
            cv::circle(center_image, centroid, 3, cv::Scalar(0, 0, 255), -1);
            cv::imshow("center_image", center_image);
            cv::waitKey(0);
#endif
            // 计算椭圆的短轴长度
            int short_axis = cv::countNonZero(line);

            // 设定椭圆的长轴长度
            int long_axis = 150; // 根据需要设定

            // 计算椭圆的角度
            double angle = atan2(contour[0].x - contour[1].x, contour[0].y - contour[1].y) * 180.0 / CV_PI;

            // 绘制椭圆
            cv::ellipse(check_image, cv::Point(cX, cY), cv::Size(long_axis/2, short_axis/2), angle, 0, 360, cv::Scalar(255), -1);

            // std::vector<cv::Point> white_points;
            // cv::findNonZero(line, white_points);
            // float radius = -1;
            // for (auto & cv_point : white_points)
            // {
            //     cv::Point2f cv_point_(cv_point.x, cv_point.y);
            //     cv::Point2f normal = cv_point_ - centroid;
            //     float length = cv::norm(normal);
            //     if (length > radius)
            //     {
            //         radius = length;
            //     }
            // }
            // cv::circle(line, centroid, radius, cv::Scalar(255), -1);
            // check_image.setTo(255, line);
#ifdef DEBUG
            cv::imshow("check_image_5", check_image);
            cv::waitKey(0);
#endif
        }
    }
    // cv::imwrite("/home/lichao/catkin_pathplanning/src/path_planning/data/check_image.png", check_image);
    // cv::imshow("check_image", check_image);
    // cv::waitKey(0);
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

// 计算两点之间的欧几里得距离
double euclideanDistance(const cv::Point& p1, const cv::Point& p2) 
{
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// 取处射线前方的一小块
vector<cv::Point> raySegOpen(cv::Point2f rayOrigin, vector<cv::Point> & contour, cv::Point2f & ray)
{
    vector<cv::Point> over_contour;
    for (auto &point : contour)
    {
        cv::Point2f point_f(point.x, point.y);
        cv::Point2f vectorToPoint = point_f - rayOrigin;
        double angle = angleBetweenVectors(ray, vectorToPoint);
        if (angle < 10)
        {
            over_contour.emplace_back(point);
        }
    }
    return over_contour;
}

// 将取出的一小块分成左右两部分
std::pair<vector<cv::Point>, vector<cv::Point> > raySegLeftRightOpen(cv::Point2f rayOrigin, vector<cv::Point> & contour, cv::Point2f & ray)
{
    vector<cv::Point> left_contour, right_contour;
    for (auto & point : contour)
    {
        cv::Point2f point_f(point.x, point.y);
        cv::Point2f vectorToPoint = point_f - rayOrigin;
        float cross = crossProduct(ray, vectorToPoint);
        if (cross > 0)
        {
            // 在射线的左边
            left_contour.emplace_back(point);
        }
        else
        {
            // 在射线的右边
            right_contour.emplace_back(point);
        }
    }
    return std::make_pair(left_contour, right_contour);
}

// 找到射线和轮廓最近的点
cv::Point getNearestPointOpen(cv::Point2f rayOrigin, vector<cv::Point> & contour, cv::Point2f & ray)
{
    double min_distance = std::numeric_limits<double>::max();
    cv::Point nearest_point(-1, -1);
    for (auto & point : contour)
    {
        double tmp_distance = distanceToRay(rayOrigin, ray, point);
        if (tmp_distance < min_distance)
        {
            nearest_point = point;
        }
    }
    return nearest_point;
}

// 这是轮廓的分割点，两侧涡流势场的方向不一样
bool PathPlanning::getNearestPointInContour(vector<cv::Point> & contour, cv::Point & nearestPoint)
{
    vector<cv::Point> over_contour = raySegOpen(seg_point, contour, seg_normal);
    std::pair<vector<cv::Point>, vector<cv::Point> > segs = raySegLeftRightOpen(seg_point, over_contour, seg_normal);
    nearestPoint = getNearestPointOpen(seg_point, segs.first, seg_normal);
    if (nearestPoint.x == -1 || nearestPoint.y == -1)
    {
        return false;
    }
    else
    {
        return true;
    }
}

// void PathPlanning::computeRadius(double d_safe_, double d_vort_, double d_noinflu_offset_)
// {
//     d_safe = d_safe_;
//     d_vort = d_vort_;
//     d_noinflu_offset = d_noinflu_offset_;
//     CHECK(d_safe > 0);
//     CHECK(d_vort > d_safe);
//     CHECK(d_noinflu_offset > 0);
//     d_inf = 2*d_vort - d_safe;
//     d_noinflu = d_inf + d_noinflu_offset;
//     d_safe_rad = d_safe/map.getResolution();
//     d_vort_rad = d_vort/map.getResolution();
//     d_inf_rad = d_inf/map.getResolution();
//     d_noinflu_rad = d_noinflu/map.getResolution();
//     LOG(INFO)<<d_safe<<" "<<d_vort<<" "<<d_noinflu_offset<<" "<<d_inf<<" "<<d_noinflu;
//     LOG(INFO)<<d_safe_rad<<" "<<d_vort_rad<<" "<<d_inf_rad<<" "<<d_noinflu_rad;
// }

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
        // std::vector<cv::Point> hull;
        // cv::convexHull(raw_contours[0], hull);
        // cv::Mat hull_mat = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
        // std::vector<std::vector<cv::Point>> hullContours = {hull};
        // // hull_mat 是凸区域
        // cv::drawContours(hull_mat, hullContours, 0, cv::Scalar(255), cv::FILLED);
        // // LOG(INFO)<<"HULL REGION";
        // // cv::imshow("hull_region", hull_mat);
        // // cv::waitKey(0);
        // // cv::Mat canve_mat = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
        // // 将凸区域的部分障碍部分去除，得到内部区域
        // cv::drawContours(hull_mat, raw_contours, 0, cv::Scalar(0), cv::FILLED);
        // // 内部区域再腐蚀一次
        // cv::erode(hull_mat, hull_mat, element);
        // 每3层，显示一次
// #ifdef DEBUG         
//         std::vector<double> x_start, y_start, u, v;
// #endif
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
            // LOG(INFO)<<"F_SAPF: "<<F_SAPF;
            // 膨胀
            cv::dilate(goal_obstacle_bk, goal_obstacle_bk, element);
            // cv::imshow("goal_obstacle_bk", goal_obstacle_bk);
            // cv::waitKey(0);
            // cv::imshow("check_image", check_image);
            // cv::waitKey(0);
            // 找外轮廓
            std::vector<std::vector<cv::Point>> single_contours;
            std::vector<cv::Vec4i> single_hierarchy;
            cv::findContours(goal_obstacle_bk, single_contours, single_hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            // cv::Mat contour_image_xx = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
            // cv::drawContours(contour_image_xx, single_contours, 0, cv::Scalar(255), 1);
            // cv::imshow("contour_image_xx", contour_image_xx);
            // cv::waitKey(0);
            // cv::Mat other_image = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
            if (inflation_radius <= planning_param.d_safe_rad_goal || inflation_radius > planning_param.d_inf_rad_goal)
            {
                // LOG(INFO)<<"GENERAL DORECT";
// #ifdef DEBUG                
//                 cv::Mat RefImage = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
// #endif
                cv::Mat RefImage = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
                // cv::Mat contour_image = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
                // std::vector<double> x_start, y_start, u, v;
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
                            // map[x_obstacle_goal](index.x(), index.y()) += normal.y; 
                            // map[y_obstacle_goal](index.x(), index.y()) += normal.x; 
                            // 表明这个栅格已经被goal所在的障碍赋上值
                            map[obstacle_goal_flag](index.x(), index.y()) = 1;
// #ifdef DEBUG
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
                                    // LOG(INFO)<<map["SAPF_X"](index.x(), index.y())<<" "<<map["SAPF_Y"](index.x(), index.y());
                                    // u.emplace_back(map[x_obstacle_goal](index.x(), index.y()));
                                    // v.emplace_back(map[y_obstacle_goal](index.x(), index.y()));
                                    u.emplace_back(normal.y);
                                    v.emplace_back(normal.x);
                                }                
                            }
// #endif
                        }
                    } 
                }
                // matplotlibcpp::quiver(x_start, y_start, u, v);
                // matplotlibcpp::axis("equal");  
                // matplotlibcpp::show(); 
                // // cv::imshow("other_image", other_image);
                // // cv::waitKey(0);     
                // // // cv::imwrite("/home/lichao/catkin_pathplanning/src/path_planning/data/RefImage.png", RefImage);
                // // cv::imshow("contour_image", contour_image);
                // // cv::waitKey(0);
                // cv::imshow("RefImage", RefImage);
                // cv::waitKey(0);
                // 将地图内的障碍斥力部分转为凸向显示，并标记斥力方向
                // showRepImage();
            }
            else
            {
                // LOG(INFO)<<"IN V";
                // LOG(INFO)<<"NONGENERAL DORECT";
                // cv::imshow("goal_obstacle NONGENERAL", goal_obstacle);
                // cv::waitKey(0);
                // cv::imshow("hull_mat NONGENERAL", hull_mat);
                // cv::waitKey(0);
                cv::Mat clock_wise_mat = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
                cv::Mat counter_clock_wise_mat = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
                cv::Point nearest_point(-1, -1);
                if (getNearestPointInContour(single_contours[0], nearest_point))
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
                        
                        // LOG(INFO)<<"index_count_clock: "<<index_count_clock<<" index_clock: "<<index_clock;
                    }
                    // LOG(INFO)<<"nearest_point: "<<nearest_point;
                    // std::pair<cv::Mat, cv::Mat> segMats = getSegMoreAndLess(cv::Point2f(goal_index_.y(), goal_index_.x()), single_contours[0], nearest_point);
                    // clock_wise_mat = segMats.second;
                    // counter_clock_wise_mat = segMats.first;
                }
                // cv::imshow("clock_wise_mat", clock_wise_mat);
                // cv::waitKey(0);
                // cv::imshow("counter_clock_wise_mat", counter_clock_wise_mat);
                // cv::waitKey(0);
                // std::pair<cv::Mat, cv::Mat> segMats = getSegMoreAndLess(goal_obstacle.clone(), hull_mat, ray_Dir);
                // // 求像素白色像素的质心
                // cv::Moments m = cv::moments(segMats.first, true);
                // double cX = m.m10 / m.m00;
                // double cY = m.m01 / m.m00;
                // cv::Point2f centroid(cX, cY);
                // // 确定方向
                // cv::Point2f v1 = ray_Dir;
                // cv::Point2f v2 = centroid - cv::Point2f(goal_index_.y(), goal_index_.x());
                // cv::Mat clock_wise_mat, counter_clock_wise_mat;
                // if (v1.x * v2.y - v1.y * v2.x > 0)
                // {
                //     clock_wise_mat = segMats.second;
                //     counter_clock_wise_mat = segMats.first;
                // }
                // else
                // {
                //     clock_wise_mat = segMats.first;
                //     counter_clock_wise_mat = segMats.second;
                // }
                // cv::imshow("clock_wise_mat", clock_wise_mat);
                // cv::waitKey(0);
                // cv::imshow("counter_clock_wise_mat", counter_clock_wise_mat);
                // cv::waitKey(0);
                // cv::imshow("goal_obstacle_xx_", goal_obstacle);
                // cv::waitKey(0);
                // cv::imshow("less_image_1", segMats.first);
                // cv::waitKey(0);
                // cv::imshow("more_image_1", segMats.second);
                // cv::waitKey(0);
                
// #ifdef DEBUG
//                 cv::Mat RefImage = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
// #endif
                cv::Mat RefImage = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
                // cv::Mat contour_image = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
                // std::vector<double> x_start, y_start, u, v;
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
                            // map[x_obstacle_goal](index.x(), index.y()) += normal_adj.y;
                            // map[y_obstacle_goal](index.x(), index.y()) += normal_adj.x; 
                            map[x_obstacle_goal](index.x(), index.y()) += F_SAPF_V.y;
                            map[y_obstacle_goal](index.x(), index.y()) += F_SAPF_V.x; 
                            map[obstacle_goal_flag](index.x(), index.y()) = 1;
// #ifdef DEBUG
                            // contour_image.at<uchar>(pt1) = 255;
                            // map["SAPF_Y_FLAG"](index.x(), index.y()) = 1;
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
                                    // u.emplace_back(map[x_obstacle_goal](index.x(), index.y()));
                                    // v.emplace_back(map[y_obstacle_goal](index.x(), index.y()));
                                    u.emplace_back(normal_adj.y);
                                    v.emplace_back(normal_adj.x);
                                }
                            }    
// #endif
                        }
                    }
                }   
                // matplotlibcpp::quiver(x_start, y_start, u, v);
                // matplotlibcpp::axis("equal");  
                // matplotlibcpp::show();    
                // // cv::imshow("contour_image", contour_image);
                // // cv::waitKey(0);
                // cv::imshow("RefImage", RefImage);
                // cv::waitKey(0);
                // cv::imshow("goal_obstacle__", goal_obstacle);
                // cv::waitKey(0);
                // showRepImage();
            }
        }
// #ifdef DEBUG
        LOG(INFO)<<"show goal apf";
        matplotlibcpp::quiver(x_start, y_start, u, v);
        matplotlibcpp::axis("equal");  
        matplotlibcpp::show(); 
// #endif
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
        // 形态学闭运算将直角变成圆角
        // cv::morphologyEx(obstacle, obstacle, cv::MORPH_CLOSE, element);
// #ifdef DEBUG
//         std::vector<double> x_start, y_start, u, v;
// #endif
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
                // 初始化随机数生成器
                // std::random_device rd;   // 用于生成种子
                // std::mt19937 gen(rd());  // 使用Mersenne Twister算法生成随机数
                // // 定义一个均匀分布范围 [20, 25]
                // std::uniform_int_distribution<> distr(20, 25);

                // // 生成一个随机数
                // int random_number = distr(gen);
                // cv::Mat contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
                // std::vector<std::vector<cv::Point>> tmp_contours = {single_contours[0]};
                // cv::drawContours(contour, tmp_contours, 0, cv::Scalar(255, 255, 255), 1);
                
                for (size_t i = 0; i < single_contours[0].size(); i++) 
                {
                    
                    cv::Point2f pt1 = single_contours[0][i];
                    cv::Point2f pt2 = single_contours[0][(i + 1) % single_contours[0].size()];
                    cv::Point2f tangent = pt2 - pt1;
                    cv::Point2f normal = cv::Point2f(tangent.y, -tangent.x);

                    // cv::Mat circle_mat = contour.clone();
                    // cv::circle(circle_mat, single_contours[0][i], 5, cv::Scalar(0, 255, 0), 2);
                    // cv::imshow("circle_mat", circle_mat);
                    // cv::waitKey(0);
                    // LOG(INFO)<<"normal: "<<normal;

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
                            // contour.at<uchar>(single_contours[0][i]) = 255;
                            // map[x_sapf](index.x(), index.y()) += normal.y;
                            // map[y_sapf](index.x(), index.y()) += normal.x;  
                            map[x_sapf](index.x(), index.y()) += F_SAPF_V.y;
                            map[y_sapf](index.x(), index.y()) += F_SAPF_V.x;
// #ifdef DEBUG
                            if (i%10 == 0 && inflation_radius%8 == 0)
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
// #endif
                            map[hull_obstacle_string](index.x(), index.y()) = 1;
                        }
                        // else
                        // {
                        //     LOG(INFO)<<"out";
                        // } 
                    }
                    // else
                    // {
                    //     LOG(INFO)<<"out";
                    // }  
                    // cv::imshow("contour", contour);
                    // cv::waitKey(0);
                }

                // // 使用 quiver 函数在图中画箭头
                // // 循环绘制每个箭头，并为每个箭头设置不同颜色
                // // 循环绘制每个箭头，并为每个箭头设置不同颜色

                // cv::imshow("RefImage", RefImage);
                // cv::waitKey(0);
                // matplotlibcpp::quiver(x_start, y_start, u, v);

                // matplotlibcpp::axis("equal");

                // // 显示图像
                // matplotlibcpp::show();

                // LOG(INFO)<<"SHOW";
                // showRepImage();
            }
            else
            {
                // LOG(INFO)<<"NONGENERAL DIRECT";

                
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
                // 输出重心坐标
                // std::cout << "Contour " << " Cen                    troid: (" << cx << ", " << cy << ")" << std::endl;
                cv::Point2f centroid(cx, cy);
                // 假设另一个点为 (px, py)，可以根据实际需要设定
                cv::Point2f goal_point(goal_index_.y(), goal_index_.x()); 
#ifdef DEBUG 
#endif
                cv::Mat RefImage = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
                // 要考虑质心和终点会不会处于同一数值线上
                // if (abs(goal_point.x - centroid.x) < 1e-3) // 处于同一竖直线
                // {
                    // LOG(INFO)<<"< 1e-3"<<" -"<<r;

                // 初始化随机数生成器
                // std::random_device rd;   // 用于生成种子
                // std::mt19937 gen(rd());  // 使用Mersenne Twister算法生成随机数

                // // 定义一个均匀分布范围 [20, 25]
                // std::uniform_int_distribution<> distr(20, 25);

                // // 生成一个随机数
                // int random_number = distr(gen);
                // std::vector<double> x_start, y_start, u, v;
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
// #ifdef DEBUG
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
// #endif
                            map[hull_obstacle_string](index.x(), index.y()) = 1;
                        }
                    }
                }
                // cv::imshow("contour", contour);
                // cv::waitKey(0);
                // cv::imshow("RefImage", RefImage);
                // cv::waitKey(0);
                // matplotlibcpp::quiver(x_start, y_start, u, v);
                // matplotlibcpp::axis("equal");
                // matplotlibcpp::show();
            }
        }
    
        // cv::Mat check_flag = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
        // for (int i = 0; i < map.getSize().x(); i++)
        // {
        //     for (int j = 0; j < map.getSize().y(); j++)
        //     {
        //         if (map[hull_obstacle_string](i, j) == 1)
        //         {
        //             check_flag.at<uchar>(i, j) = 255;
        //         }
        //     }
        // }
        // cv::imshow("check_flag", check_flag);
        // cv::waitKey(0);
// #ifdef DEBUG
        matplotlibcpp::quiver(x_start, y_start, u, v);
        matplotlibcpp::axis("equal");
        matplotlibcpp::show();
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
            if (i%10 == 0 && j%10 ==0)
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
    matplotlibcpp::quiver(x_start, y_start, u, v);

    matplotlibcpp::axis("equal");

    // 显示图像
    matplotlibcpp::show();
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
            if (i%10 == 0 && j%10 ==0)
            {
                grid_map::Position p2;
                if (map.getPosition(grid_map::Index(i, j), p2))
                {
                    Node node;
                    node.position = p2;
                    Eigen::Vector2d SAPF;
                    if (AttPotential(node, SAPF))
                    {
                        // SAPF.normalize();
                        x_start.emplace_back(p2.x());
                        y_start.emplace_back(p2.y());
                        u.emplace_back(SAPF.x());
                        v.emplace_back(SAPF.y());
                    }
                }
            }
            
        }
    }
    matplotlibcpp::quiver(x_start, y_start, u, v);

    matplotlibcpp::axis("equal");

    // 显示图像
    matplotlibcpp::show();
    
}

void PathPlanning::showSAPF()
{
    std::vector<double> x_start, y_start, u, v;
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            if (i%10 == 0 && j%10 ==0)
            {
                grid_map::Position p2;
                if (map.getPosition(grid_map::Index(i, j), p2))
                {
                    Node node;
                    node.position = p2;
                    Eigen::Vector2d SAPF;
                    if (getSAPF(node, SAPF))
                    {
                        // SAPF.normalize();
                        x_start.emplace_back(p2.x());
                        y_start.emplace_back(p2.y());
                        u.emplace_back(SAPF.x());
                        v.emplace_back(SAPF.y());
                    }
                }
            }
        }
    }
    matplotlibcpp::quiver(x_start, y_start, u, v);

    matplotlibcpp::axis("equal");

    // 显示图像
    matplotlibcpp::show();
}

void PathPlanning::showJoinForceImage()
{
    std::vector<double> x_start, y_start, u, v;
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            if (i%10 == 0 && j%10 ==0)
            {
                grid_map::Position p2;
                if (map.getPosition(grid_map::Index(i, j), p2))
                {
                    Node node;
                    node.position = p2;
                    Eigen::Vector2d SAPF;
                    Eigen::Vector2d AttSAPF;
                    if (getSAPF(node, SAPF))
                    {
                        if (AttPotential(node, AttSAPF))
                        {
                            // SAPF.normalize();
                            x_start.emplace_back(p2.x());
                            y_start.emplace_back(p2.y());
                            Eigen::Vector2d JoinForce = AttSAPF + SAPF;
                            JoinForce.normalize();
                            // u.emplace_back(SAPF.x() + AttSAPF.x());
                            // v.emplace_back(SAPF.y() + AttSAPF.y());
                            u.emplace_back(JoinForce.x());
                            v.emplace_back(JoinForce.y());
                        }
                        
                    }
                }
            }
            
        }
    }
    matplotlibcpp::quiver(x_start, y_start, u, v);

    matplotlibcpp::axis("equal");
    // matplotlibcpp::gcf().canvas.set_window_title("合力");

    // 显示图像
    matplotlibcpp::show();
}

bool PathPlanning::processing()
{
    constructPlaneAwareMap();
    constructObstacleLayer(planning_param.obstacle_rad);
    // LOG(INFO)<<"...";
    constructFullFeasibleRegion(planning_param.safe_region_radius);
    // LOG(INFO)<<"...";
    computeRep();
    // LOG(INFO)<<"...";
    return plan();
    // return planAstar();
}


void PathPlanning::computeRep()
{
    computeObstacles();
    // cv::imshow("check_image_1", check_image);
    // cv::waitKey(0);
    // computeRadius(0.6, 1.5, 0.6);
    // cv::imshow("check_image_2", check_image);
    // cv::waitKey(0);
    computeRepObstacleGoal();
    LOG(INFO)<<"get goal sapf";
    computeRepObstacle();
    LOG(INFO)<<"get gen sapf";
    mergeAllObstacle();
    showSAPFMatplotlib();
    // 展示终点吸引力
    showAttPotential();
    // 展示势场力
    showSAPF();
    LOG(INFO)<<"合力";
    showJoinForceImage();

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
                for (double d_l = 0.4; d_l > 0.01; d_l -= 0.05)
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

// 复现论文“Humanoid Path Planning over Rough Terrain using Traversability Assessment”
bool PathPlanning::transitionsAstar(NodePtr currentNodeP, std::vector<NodePtr> & nodes)
{
    grid_map::Index index;
    if (map.getIndex(currentNodeP->position, index))
    {
        if (full_feasible_region.at<uchar>(index.x(), index.y()) == 255)
        {
            for (int i = 0; i < angles_trans_Astar.size(); i++)
            {
                Eigen::Vector3d length;
                if (i%2 == 0)
                {
                    length = Eigen::Vector3d(0.2, 0, 0);
                }
                else
                {
                    length = Eigen::Vector3d(0.4, 0, 0);
                }
                Eigen::AngleAxisd ad;
                double normalized_angle;
                // 确保 angles_trans_Astar 和 currentNodeP 都已初始化
                try {
                    double angle_sum = angles_trans_Astar.at(i) + currentNodeP->ori;
                    normalized_angle = fmod(angle_sum, 2 * M_PI);
                    if (normalized_angle < 0) {
                        normalized_angle += 2 * M_PI;
                    }
                    ad = Eigen::AngleAxisd(normalized_angle, Eigen::Vector3d::UnitZ());
                } catch (const std::out_of_range& e) {
                    // 处理索引超出范围的情况
                    std::cerr << "Index out of range: " << e.what() << std::endl;
                }

                Eigen::Vector3d tran = ad.toRotationMatrix() * length;
                Node n;
                n.ori = normalized_angle;
                n.position = currentNodeP->position + tran.head(2);
                n.PreFootstepNode = currentNodeP;
                if (!isCorss(std::make_shared<Node>(n)))
                {
                    NodePtr next_node_p = std::make_shared<Node>(n);
                    if (ComputeNodeCostAstar(next_node_p))
                    {
                        next_node_p->PreFootstepNode = currentNodeP;
                        nodes.emplace_back(next_node_p);
                    }
                }
            }
        }
        else
        {
            for (int i = 0; i < angles_trans_Astar.size(); i++)
            {
                Eigen::Vector3d length;
                if (i%2 == 0)
                {
                    length = Eigen::Vector3d(0.2, 0, 0);
                }
                else
                {
                    length = Eigen::Vector3d(0.4, 0, 0);
                }
                Eigen::AngleAxisd ad;
                double normalized_angle;
                // 确保 angles_trans_Astar 和 currentNodeP 都已初始化
                try {
                    double angle_sum = angles_trans_Astar.at(i) + currentNodeP->ori;
                    normalized_angle = fmod(angle_sum, 2 * M_PI);
                    if (normalized_angle < 0) {
                        normalized_angle += 2 * M_PI;
                    }
                    ad = Eigen::AngleAxisd(normalized_angle, Eigen::Vector3d::UnitZ());
                } catch (const std::out_of_range& e) {
                    // 处理索引超出范围的情况
                    std::cerr << "Index out of range: " << e.what() << std::endl;
                }
                Eigen::Vector3d tran = ad.toRotationMatrix() * length;
                Node n;
                n.ori = normalized_angle;
                n.position = currentNodeP->position + tran.head(2);
                n.PreFootstepNode = currentNodeP;
                if (!isCorss(std::make_shared<Node>(n)))
                {                    
                    if (CheckTraversability(n))
                    {
                        NodePtr next_node_p = std::make_shared<Node>(n);
                        if (ComputeNodeCostAstar(next_node_p))
                        {
                            next_node_p->PreFootstepNode = currentNodeP;
                            nodes.emplace_back(next_node_p);
                        }
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
            return isTraversbility(node);
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

bool PathPlanning::planAstar()
{
    LOG(INFO)<<"using a star";
    auto start = std::chrono::high_resolution_clock::now();
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
        // LOG(INFO)<<"index: "<<index.transpose();
#ifdef SHOW_PLANNING_PROCESS
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
        vector<NodePtr> nodes;
        if (transitionsAstar(currentNode, nodes))
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
        else
        {
            LOG(ERROR)<<"can not get yaw";
            return false;
        }
#ifdef SHOW_PLANNING_PROCESS
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