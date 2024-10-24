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
void initial_package_path(string package_name, string & package_path)
{
  package_path = ros::package::getPath(package_name);
  // 检查是否成功找到包路径
  if (package_path.empty()) {
      std::cerr << "Error: Could not find package " << package_name << std::endl;
  }
  cout<<"package path: "<<package_path<<endl;
}


PathPlanning::PathPlanning(ros::NodeHandle & nodehand, grid_map::GridMap & map_, SupportArea support_area_, Eigen::Vector3d & start, Eigen::Vector3d & goal):nh(nodehand), map(map_), support_area(support_area_), start_(start), goal_(goal)
{
    initial_package_path("path_planning", package_path);
    LOG(INFO)<<"package path is: "<<package_path;
    pd.initial(package_path + "/config/plane_fitter_pcd.ini");
    int x_size = ceil((support_area.Button + support_area.Up)/map.getResolution());
    int y_size = ceil((support_area.Left + support_area.Right)/map.getResolution());
    full_size = x_size * y_size;
    map_size_ = map.getSize().x() * map.getSize().y();

    LOG(INFO)<<start_.transpose();
    LOG(INFO)<<goal_.transpose();
    if (!map.getIndex(start_.head(2), start_index_))
    {
        LOG(ERROR)<<"start is out of map";
    }

    if (!map.getIndex(goal_.head(2), goal_index_))
    {
        LOG(ERROR)<<"goal is out of map";
    }
    goalDirectInImage(goal_.z());
    LOG(INFO)<<"pixel direct: "<<ray_Dir;
    Nan_feasible = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
#ifdef DEBUG
    // cv::imshow("Nan_feasible", Nan_feasible);
    // cv::waitKey(0);
#endif
    Full_feasible = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);

    check_Mat = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    strictChecksMat = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    // Full_feasible.setTo(cv::Scalar(255));
#ifdef DEBUG
    // cv::imshow("Full_feasible", Full_feasible);
    // cv::waitKey(0);
#endif
    checks_Mat.resize(72);// 72 * 5 = 360
    for (auto & image : checks_Mat)
    {
        image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    }
    // 这个值的选取是一个非常谨慎的事
    strict_section = 2;
    obstacle_length = 2;
    obstacle_inflation_radius = 0.5;
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
    pd.detect(pc);
    // // 检查法向量的方向，默认朝上。即z>0
    for (int i = 0; i < pd.planes_info.size(); i++)
    {
        if (pd.planes_info.at(i).normal.z() < 0)
        {
            pd.planes_info.at(i).normal = - pd.planes_info.at(i).normal;
        }
    }

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
                    break;
                }
            }
            if (!flag)
            {
                map["label"](i,j) = NAN;
            }
        }
    }
    LOG(INFO)<<"construct plane-aware map finish";
}

void PathPlanning::constructFullFeasibleRegion(double safe_length)
{
    full_feasible_region = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    int radius = safe_length / map.getResolution();
    for (int i = 0; i < pd.planes.size(); i++)
    {
        cv::Mat image = pd.planes.at(i).clone();
        cv::erode(image, image, cv::Mat(), cv::Point(-1, -1), radius);
        full_feasible_region.setTo(255, image);
    }
    // #ifdef DEBUG
    cv::imshow("full_feasible_region", full_feasible_region);
    cv::waitKey(0);
    // #endif
}

void PathPlanning::constructGoalVortexRegion()
{
    goal_vortex_region = cv::Mat::zeros(map.getSize()(1), map.getSize()(0), CV_8UC1);
    if (cv::countNonZero(goal_obstacle) > 0)
    {
        cv::Mat element_1 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(d_inf_rad*2 + 1, d_inf_rad*2 + 1), cv::Point(1, 1));
        cv::Mat element_2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(d_safe_rad*2 + 1, d_safe_rad*2 + 1), cv::Point(1, 1));
        cv::Mat mat_1, mat_2;
        cv::dilate(goal_obstacle, mat_1, element_1);
        cv::dilate(goal_obstacle, mat_2, element_2);
        mat_1.setTo(0, mat_2);
        goal_vortex_region = mat_1;
    }
    else
    {
        LOG(INFO)<<"do not have goal vortex region.";
    }
}

void PathPlanning::constructObstacleLayer(int chect_scope)
{
    obstacle_layer = cv::Mat::zeros(pd.result.size(), CV_8UC1);
    for (int i = 0; i < pd.planes.size(); i++)
    {
        cv::Mat image = pd.planes.at(i).clone();
        // cv::imshow("image", image);
        // cv::waitKey(0);
        // 存储轮廓的向量
        std::vector<std::vector<cv::Point>> contours;

        // 查找所有轮廓，并且保存轮廓上的所有点
        cv::findContours(image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

        int kernelSize = chect_scope; // 定义核的大小，例如3x3
        //  定义最大高度，最小高度，及所有平面编号
        
        for (auto & contour : contours)
        {
            for (auto & cv_point : contour)
            {
                double max_height = - std::numeric_limits<double>::max();
                double min_height = std::numeric_limits<double>::max();
                if (cv_point.x < 10 || cv_point.y < 10 || cv_point.x > image.cols -10 || cv_point.y > image.rows - 10)
                {
                    continue;
                }
                std::unordered_map<int, Eigen::Vector3d> normals;
                for (int ky = -kernelSize / 2; ky <= kernelSize / 2; ++ky) 
                {
                    for (int kx = -kernelSize / 2; kx <= kernelSize / 2; ++kx) 
                    {
                        int nx = cv_point.x + kx;
                        int ny = cv_point.y + ky;
                        // 检查邻域内的点是否在图像边界内
                        if (nx >= 0 && ny >= 0 && nx < image.cols && ny < image.rows) 
                        // if (nx >= 10 && ny >= 10 && nx < image.cols -10 && ny < image.rows - 10) 
                        {
                            // std::cout << "  Neighbor pixel at: (" << nx << ", " << ny << ")" << std::endl;
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
                                if (!std::isnan(map["label"](ny, nx)))
                                {
                                    int label_index = static_cast<int>(map["label"](ny, nx));
                                    normals[label_index] = pd.planes_info.at(label_index).normal;
                                }
                            }
                        }
                    }
                }
                // 最大迈不高度不超过0.15
                if (abs(max_height - min_height) > 0.15)
                {
                    // LOG(INFO)<<max_height<<" "<<min_height;
                    obstacle_layer.at<uchar>(cv_point.y, cv_point.x) = 255;
                }
                else
                {
                    double max_angle = 0;
                    for (auto & normal1 : normals)
                    {
                        for (auto & normal2 : normals)
                        {
                            double tmp_angle = std::acos(abs(normal1.second.dot(normal2.second)));
                            if (tmp_angle > max_angle)
                            {
                                max_angle = tmp_angle;
                            }
                        }
                    }
                    // 大于25度，则认为为障碍
                    if (max_angle > 25/57.3)
                    {
                        obstacle_layer.at<uchar>(cv_point.y, cv_point.x) = 255;
                    }
                    // else
                    // {
                    //     // 如果角度和高度差都满足，那证明该点内的各平面是可联通的
                    //     for (auto it1 = normals.begin(); it1 != normals.end(); ++it1)
                    //     {
                    //         auto it2 = it1;  // 从 it1 开始，避免重复的边
                    //         for (++it2; it2 != normals.end(); ++it2)
                    //         {
                    //             if (it1->first != it2->first)
                    //             {
                    //                 // 这几个平面是相互连通的
                    //                 plane_graph.addEdge(it1->first, it2->first);
                    //             }
                    //         }
                    //     }
                    // }
                }
            }
        }
    }
    

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
            if (cv::countNonZero(temp_image) < (obstacle_length * obstacle_length)/(map.getResolution() * map.getResolution()))
            {
                // 如果是障碍区域，且障碍区域的面积较小，那么将其内部都设置为白色，也就都是障碍区域
                int g_nStructBLlementSize = obstacle_inflation_radius/map.getResolution();
                cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * g_nStructBLlementSize + 1, 2 * g_nStructBLlementSize + 1 ));
                cv::dilate(temp_image, temp_image, element);
                obstacle_layer.setTo(255, temp_image);
            }
        }
    }

    LOG(INFO)<<"construct obstacle map finish";
    cv::imwrite(package_path + "/data/obstacle_layer.png", obstacle_layer);
    // 障碍的定义：如果保持原来的定义，则适应的场景有限.
    // 如果对每个平面进行膨胀，一层层的膨胀，每个膨胀层都是障碍，那这个点就是障碍
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

void PathPlanning::computeObstacles()
{
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
            std::vector<std::vector<cv::Point>> Contours = {contour};
            cv::drawContours(obstacle, Contours, 0, cv::Scalar(255), cv::FILLED);
            obstacles.emplace_back(obstacle);
            // LOG(INFO)<<"show obstacle";
            // cv::imshow("obstacle", obstacle);
            // cv::waitKey(0);
            check_image.setTo(255, obstacle);
            // LOG(INFO)<<"check obstacle";
            // cv::imshow("check_image_3", check_image);
            // cv::waitKey(0);
            // 为每个其他障碍添加标记
            // map.add("SAPF_OBSTACLE_" + std::to_string(hull_index), 0);
        }
        else
        {
            // 如果有终点在障碍内的情况，添加此处标记
            // map.add("SAPF_OBSTACLE_GOAL", 0);
            goal_obstacle = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            std::vector<std::vector<cv::Point>> Contours = {contour};
            cv::drawContours(goal_obstacle, Contours, 0, cv::Scalar(255), cv::FILLED);
            // LOG(INFO)<<"mask obstacle";
            // cv::imshow("goal_obstacle", goal_obstacle);
            // cv::waitKey(0);

            // 这样将凹区域也涂上，表明内部也不会被赋值
            // cv::convexHull(obstacle, hull);
            std::vector<std::vector<cv::Point>> hullContours = {hull};
            cv::drawContours(check_image, hullContours, 0, cv::Scalar(255), cv::FILLED);

            // 将凸区域的部分障碍部分去除，得到内部区域
            cv::Mat hull_region = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            cv::drawContours(hull_region, hullContours, 0, cv::Scalar(255), cv::FILLED);
            cv::drawContours(hull_region, Contours, 0, cv::Scalar(0), cv::FILLED);

            cv::Mat hull_contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
            cv::drawContours(hull_contour, hullContours, -1, 255, 1);
            
            cv::Mat line = cv::Mat::zeros(goal_obstacle.size(), CV_8UC1);
            cv::bitwise_and(hull_region, hull_contour, line);

            // cv::imshow("line", line);
            // cv::waitKey(0);
            cv::Moments m = cv::moments(line, true);
            double cX = m.m10 / m.m00;
            double cY = m.m01 / m.m00;
            cv::Point2f centroid(cX, cY);
            std::vector<cv::Point> white_points;
            cv::findNonZero(line, white_points);
            float radius = -1;
            for (auto & cv_point : white_points)
            {
                cv::Point2f cv_point_(cv_point.x, cv_point.y);
                cv::Point2f normal = cv_point_ - centroid;
                float length = cv::norm(normal);
                if (length > radius)
                {
                    radius = length;
                }
            }
            cv::circle(line, centroid, radius, cv::Scalar(255), -1);
            check_image.setTo(255, line);
            // cv::imshow("check_image_5", check_image);
            // cv::waitKey(0);
        }
    }
    cv::imshow("check_image", check_image);
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
}

void PathPlanning::computeRadius(double d_safe_, double d_vort_, double d_noinflu_offset_)
{
    d_safe = d_safe_;
    d_vort = d_vort_;
    d_noinflu_offset = d_noinflu_offset_;
    CHECK(d_safe > 0);
    CHECK(d_vort > d_safe);
    CHECK(d_noinflu_offset > 0);

    // double d_safe = 0.6;
    // double d_vort = 1.5;
    d_inf = 2*d_vort - d_safe;
    d_noinflu = d_inf + d_noinflu_offset;

    d_safe_rad = d_safe/map.getResolution();
    d_vort_rad = d_vort/map.getResolution();
    d_inf_rad = d_inf/map.getResolution();
    d_noinflu_rad = d_noinflu/map.getResolution();
    LOG(INFO)<<d_safe<<" "<<d_vort<<" "<<d_noinflu_offset<<" "<<d_inf<<" "<<d_noinflu;
    LOG(INFO)<<d_safe_rad<<" "<<d_vort_rad<<" "<<d_inf_rad<<" "<<d_noinflu_rad;
}

// 对goalregion和非goalregion内的斥力需要使用不同参数
// 非goalregion的距离参数应该更小，但是goalregion的距离参数更大
void PathPlanning::computeRepObstacleGoal()
{
    if (cv::countNonZero(goal_obstacle) > 0)
    {
        string obstacle_goal_flag = "obstacle_goal_FLAG";
        string x_obstacle_goal = "obstacle_goal_X";
        string y_obstacle_goal = "obstacle_goal_Y";
        cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3), cv::Point(1, 1));

        auto goal_obstacle_bk = goal_obstacle.clone();
        std::vector<std::vector<cv::Point>> raw_contours;
        std::vector<cv::Vec4i> raw_hierarchy;
        cv::findContours(goal_obstacle_bk, raw_contours, raw_hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        std::vector<cv::Point> hull;
        cv::convexHull(raw_contours[0], hull);
        cv::Mat hull_mat = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> hullContours = {hull};
        // hull_mat 是凸区域
        cv::drawContours(hull_mat, hullContours, 0, cv::Scalar(255), cv::FILLED);
        // LOG(INFO)<<"HULL REGION";
        // cv::imshow("hull_region", hull_mat);
        // cv::waitKey(0);
        // cv::Mat canve_mat = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
        // 将凸区域的部分障碍部分去除，得到内部区域
        cv::drawContours(hull_mat, raw_contours, 0, cv::Scalar(0), cv::FILLED);
        // 内部区域再腐蚀一次
        cv::erode(hull_mat, hull_mat, element);
        // 每3层，显示一次
        std::vector<double> x_start, y_start, u, v;
        for (int inflation_radius = 1; inflation_radius <= d_noinflu_rad; inflation_radius++)
        {
            double d = map.getResolution() * (inflation_radius);
            double d_rel;
            if (d < d_safe)
            {
                d_rel = 0;
            }
            else if (d > d_inf)
            {
                d_rel = 1;
            }
            else
            {
                d_rel = (d - d_safe) / (2 * (d_vort - d_safe));
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
            if (d <= d_noinflu)
            {
                F_SAPF = goal_obstacle_cof * (1/(d) - 1/d_noinflu) * (1/(d_noinflu * d_noinflu));
            }
            else
            {
                F_SAPF = 0;
            }
            // LOG(INFO)<<"F_SAPF: "<<F_SAPF;
            cv::dilate(goal_obstacle, goal_obstacle, element);
            // cv::imshow("goal_obstacle", goal_obstacle);
            // cv::waitKey(0);
            // cv::imshow("check_image", check_image);
            // cv::waitKey(0);
            std::vector<std::vector<cv::Point>> single_contours;
            std::vector<cv::Vec4i> single_hierarchy;
            cv::findContours(goal_obstacle, single_contours, single_hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            // cv::Mat contour_image_xx = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
            // cv::drawContours(contour_image_xx, single_contours, 0, cv::Scalar(255), 1);
            // cv::imshow("contour_image_xx", contour_image_xx);
            // cv::waitKey(0);
            cv::Mat other_image = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
            if (inflation_radius <= d_safe_rad || inflation_radius > d_inf_rad)
            {
                // LOG(INFO)<<"GENERAL DORECT";
                cv::Mat RefImage = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
                // cv::Mat contour_image = cv::Mat::zeros(map.getSize(). x(), map.getSize().y(), CV_8UC1);
                // std::vector<double> x_start, y_start, u, v;
                for (size_t i = 0; i < single_contours[0].size(); i++) 
                {
                    cv::Point2f pt1 = single_contours[0][i];
                    cv::Point2f pt2 = single_contours[0][(i + 1) % single_contours[0].size()];
                    cv::Point2f tangent = pt2 - pt1;
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
                            
                            map[x_obstacle_goal](index.x(), index.y()) += F_SAPF_V.y;
                            map[y_obstacle_goal](index.x(), index.y()) += F_SAPF_V.x;
                            // map[x_obstacle_goal](index.x(), index.y()) += normal.y; 
                            // map[y_obstacle_goal](index.x(), index.y()) += normal.x; 
                            // 表明这个栅格已经被goal所在的障碍赋上值
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
                                    // LOG(INFO)<<map["SAPF_X"](index.x(), index.y())<<" "<<map["SAPF_Y"](index.x(), index.y());
                                    u.emplace_back(map[x_obstacle_goal](index.x(), index.y()));
                                    v.emplace_back(map[y_obstacle_goal](index.x(), index.y()));
                                }                
                            }
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
                std::pair<cv::Mat, cv::Mat> segMats = getSegMoreAndLess(goal_obstacle.clone(), hull_mat, ray_Dir);

                // 求像素白色像素的质心
                cv::Moments m = cv::moments(segMats.first, true);
                double cX = m.m10 / m.m00;
                double cY = m.m01 / m.m00;
                cv::Point2f centroid(cX, cY);

                // 确定方向
                cv::Point2f v1 = ray_Dir;
                cv::Point2f v2 = centroid - cv::Point2f(goal_index_.y(), goal_index_.x());
                cv::Mat clock_wise_mat, counter_clock_wise_mat;
                if (v1.x * v2.y - v1.y * v2.x > 0)
                {
                    clock_wise_mat = segMats.second;
                    counter_clock_wise_mat = segMats.first;
                }
                else
                {
                    clock_wise_mat = segMats.first;
                    counter_clock_wise_mat = segMats.second;
                }
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
                    if (clock_wise_mat.at<uchar>(single_contours[0][i]) == 255)
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
                                    u.emplace_back(map[x_obstacle_goal](index.x(), index.y()));
                                    v.emplace_back(map[y_obstacle_goal](index.x(), index.y()));
                                }
                            }
                                                
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
        matplotlibcpp::quiver(x_start, y_start, u, v);
        matplotlibcpp::axis("equal");  
        matplotlibcpp::show(); 
    }
}

void PathPlanning::computeRepObstacle()
{
    // cv::Mat element_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    for (int i = 0; i < obstacles.size(); i++)
    {
        string hull_obstacle_string = "obstacle_"+std::to_string(i) + "_FLAG";
        string x_sapf = "obstacle_"+std::to_string(i) + "_X";
        string y_sapf = "obstacle_"+std::to_string(i) + "_Y";
        cv::Mat obstacle = obstacles.at(i).clone();

        // 形态学闭运算将直角变成圆角
        // cv::morphologyEx(obstacle, obstacle, cv::MORPH_CLOSE, element);

        std::vector<double> x_start, y_start, u, v;
        cv::Mat contour = cv::Mat::zeros(obstacle_layer.size(), CV_8UC1);
        for (int inflation_radius = 1; inflation_radius <= d_noinflu_rad; inflation_radius++)
        {
            double d = map.getResolution() * (inflation_radius);
            double F_SAPF;
            if (d <= d_noinflu)
            {
                F_SAPF = gen_obstacle_cof * (1/(d) - 1/d_noinflu) * (1/(d_noinflu * d_noinflu));
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
            if (inflation_radius <= d_safe_rad || inflation_radius >= d_inf_rad)
            {
                // LOG(INFO)<<"GENERAL DIRECT";
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
                                    u.emplace_back(map[x_sapf](index.x(), index.y()));
                                    v.emplace_back(map[y_sapf](index.x(), index.y()));
                                }
                            }
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
                if (d < d_safe)
                {
                    d_rel = 0;
                }
                else if (d > d_inf)
                {
                    d_rel = 1;
                }
                else
                {
                    d_rel = (d - d_safe) / (2 * (d_vort - d_safe));
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
                                    u.emplace_back(map[x_sapf](index.x(), index.y()));
                                    v.emplace_back(map[y_sapf](index.x(), index.y()));
                                }
                            }
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
        
        matplotlibcpp::quiver(x_start, y_start, u, v);
        matplotlibcpp::axis("equal");
        matplotlibcpp::show();
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
                        // normal_.normalize();
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
                if (d < d_g_att)
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
                    AttForce = position_diff / diff_length * d_g_att_th;
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
                    AttForce = position_diff / diff_length * d_g_att_th * d_g_att / d;
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

void PathPlanning::computeRep()
{
    computeObstacles();
    // cv::imshow("check_image_1", check_image);
    // cv::waitKey(0);
    computeRadius(0.6, 1.5, 0.6);
    // cv::imshow("check_image_2", check_image);
    // cv::waitKey(0);
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

// 计算下一个节点的代价
bool PathPlanning::ComputeNodeCost(NodePtr currentNodeP, NodePtr nextNodeP)
{
    LOG(INFO)<<"ComputeNodeCost"<<endl;
    if (map.isInside(nextNodeP->position) && map.isInside(currentNodeP->position))
    {
        // 分为贪心项和启发项，贪心项由所行驶的路程决定，启发项由当前点到目标点的距离及角度差决定
        // 整个代价还包括当前节点方向与下一节点方向偏差，
        LOG(INFO)<<currentNodeP->ori<<" "<<nextNodeP->ori<<endl;
        nextNodeP->g_cost = currentNodeP->g_cost + (currentNodeP->position - nextNodeP->position).norm();
        nextNodeP->h_cost = (goal_.head(2) - nextNodeP->position).norm() + abs(nextNodeP->ori - goal_.z()) * 5;
        nextNodeP->cost = nextNodeP->g_cost + nextNodeP->h_cost + abs(nextNodeP->ori - currentNodeP->ori)*2;
        return true;
    }
    else
    {
        return false;
    }
}

// 四面八方都打分，与yaw越一致的地方打分越高
// 先按四面八方来获取所有的节点
bool PathPlanning::transitions(NodePtr currentNodeP, double yaw, std::vector<NodePtr> & nodes)
{
    Eigen::Vector3d raw_length(0.4, 0, 0);
    Eigen::Vector3d raw_point = Eigen::Vector3d::Zero();
    raw_point.head(2) = currentNodeP->position;
    auto current_angles = angles;
    std::transform(current_angles.begin(), current_angles.end(), current_angles.begin(), [yaw](double x) {
        return yaw + x;
    });

    // for (auto & angle : current_angles)
    // {
    //     LOG(INFO)<<"angle:"<<angle;
    // }
    for (int i = 0; i < current_angles.size(); i++)
    {
        LOG(INFO)<<"i = "<<i<<" "<<current_angles.at(i);
    }

    vector<Eigen::AngleAxisd> rots;
    for (auto & angle : current_angles)
    {
        rots.emplace_back(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()));
    }
    grid_map::Index index;
    if (map.getIndex(currentNodeP->position, index))
    {
        if (full_feasible_region.at<uchar>(index.x(), index.y()) == 255) // 如果在一个安全裕度范围内，则使用原始的节点扩展
        {      
            LOG(INFO)<<"IN FULL FEASIBLE REGION";   
            for (auto & angle : current_angles)
            {
                Eigen::Vector3d point = raw_point + Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix() * raw_length;
                Node node;
                node.position = point.head(2);
                node.ori = angle;
                NodePtr next_node_p = std::make_shared<Node>(node);
                LOG(INFO)<<"position: "<<node.position.transpose();
                LOG(INFO)<<"angle: "<<angle;
                if (ComputeNodeCost(currentNodeP, next_node_p))
                {
                    next_node_p->PreFootstepNode = currentNodeP;
                    LOG(INFO)<<"g cost: "<<next_node_p->g_cost<<", h cost: "<<next_node_p->h_cost<<", cost: "<<next_node_p->cost;
                    nodes.emplace_back(next_node_p);
                }
                // 检查node的可通行性
            }
            return true;
        }
        else // 如果不在安全裕度内，则根据方向来寻找节点
        {
            LOG(INFO)<<"NOT IN FULL FEASIBLE REGION";   
            for (auto & angle : current_angles)
            {
                // 找到第一个不为可通行的节点，还需要好好规划这个节点
                double d_l = 0.2;
                double d_i = 0.2;
                for (; d_l > 0, d_i < 0.4; d_l -= 0.05, d_i += 0.05)
                {
                    Node n_l, n_i;
                    Eigen::Vector3d length_l(d_l, 0, 0);
                    Eigen::Vector3d length_i(d_i, 0, 0);
                    n_l.position = (raw_point + Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix() * length_l).head(2);
                    n_l.ori = angle;

                    n_i.position = (raw_point + Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix() * length_i).head(2);
                    n_i.ori = angle;
                    if (CheckTraversability(n_l))
                    {
                        NodePtr next_node_p = std::make_shared<Node>(n_l);
                        if (ComputeNodeCost(currentNodeP, next_node_p))
                        {
                            next_node_p->PreFootstepNode = currentNodeP;
                            nodes.emplace_back(next_node_p);
                            break;
                        }
                    }
                    if (CheckTraversability(n_i))
                    {
                        NodePtr next_node_p = std::make_shared<Node>(n_i);
                        if (ComputeNodeCost(currentNodeP, next_node_p))
                        {
                            next_node_p->PreFootstepNode = currentNodeP;
                            nodes.emplace_back(next_node_p);
                            break;
                        }
                    }
                }
            }
            return true;
        }
    }
    else
    {
        return false;
    }
}

// 把support area根据脚踝点和中心点分为4部分，求四部分的支撑平面并保证一致来确定整个支撑区域的支撑平面，并根据有没有点穿过支撑区域来确定是否可通行
bool PathPlanning::isTraversbility(Node & node)
{
    Eigen::AngleAxisd ax(node.ori, Eigen::Vector3d::UnitZ());
    Eigen::Vector3d mid(node.position.x(), node.position.y(), 0);
    // 求4个子区域
    Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(support_area.Up, 0, 0) + mid;
    Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, support_area.Left, 0) + mid_top;
    Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - support_area.Right, 0) + mid_top;


    Eigen::Vector3d mid_left = ax.toRotationMatrix() * Eigen::Vector3d(0, support_area.Left, 0) + mid;
    Eigen::Vector3d mid_right = ax.toRotationMatrix() * Eigen::Vector3d(0, -support_area.Right, 0) + mid;

    Eigen::Vector3d mid_button = ax.toRotationMatrix() * Eigen::Vector3d(- support_area.Button, 0, 0) + mid;
    Eigen::Vector3d button_left = ax.toRotationMatrix() * Eigen::Vector3d(0, support_area.Left, 0) + mid_button;
    Eigen::Vector3d button_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - support_area.Right, 0) + mid_button;

    if (map.isInside(top_left.head(2)) && map.isInside(top_right.head(2)) && map.isInside(button_left.head(2)) && map.isInside(button_right.head(2)))
    {
        int top_left_support_plane, top_right_support_plane, button_left_support_plane, button_right_support_plane;
        bool top_left_flag = subRegionSupportPlane(top_left.head(2), mid_top.head(2), mid_left.head(2), mid.head(2), top_left_support_plane);
        bool top_right_flag = subRegionSupportPlane(mid_top.head(2), mid_right.head(2), mid.head(2), mid_right.head(2), top_right_support_plane);
        bool button_left_flag = subRegionSupportPlane(mid_left.head(2), mid.head(2), button_left.head(2), mid_button.head(2), button_left_support_plane);
        bool button_right_flag = subRegionSupportPlane(mid.head(2), mid_right.head(2), mid_button.head(2), button_right.head(2), button_right_support_plane);
        
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
                            return false;
                        }
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
    else
    {
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
        Eigen::Vector2d mid_point = (TL + TR + BL + BR)/4;
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
        if (NanNumber > 50)
        {
            return false;
        }
        double tmp_height = -std::numeric_limits<double>::infinity();
        int tmp_plane_index;
        for (auto & pair : countMap)
        {
            if (pair.second > 100)
            {
                int height = pd.planes_info.at(pair.first).getZ(mid_point);
                if (height > tmp_height)
                {
                    tmp_plane_index = pair.first;
                    tmp_height = height;
                }
            }
        }
        plane_index = tmp_plane_index;
        return true;       
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

// 节点扩展
// void NodeExpansion()
// {
//     // 根据机器人合力方向来扩展，涡流场强度更大。transitions函数，按四面八方来获取所有候选节点
//     // 评估每个节点的可通行性
//     // 打分
//     // node应该包括，当前方向，节点位置，节点代价
//     // 得到一些可通行的点之后
//     // 指定节点的父子关系
//     // 打分，到终点的距离，已经走过的距离，与起点偏离的角度，与终点偏离的角度，比较近时来起作用吧。考虑与涡流的比例打分
//     // 打分，分区，在没有SAPF时，使用打分，
// }


bool PathPlanning::reachGoal(Node & node)
{
    Eigen::Vector2d diff = goal_.head(2) - node.position;
    return diff.norm() < 0.4;
}

bool PathPlanning::plan()
{
    debug_image.setTo(cv::Scalar(0, 0, 0), obstacle_layer);
    LOG(INFO)<<"start planning";
    cv::imshow("debug", debug_image);
    cv::waitKey(0);

    std::priority_queue<NodePtr, std::vector<NodePtr>, CompareNode> minHeap;
    Node start;
    start.position = start_.head(2);
    start.ori = start_.z();

    minHeap.push(std::make_shared<Node>(start));
    while (!minHeap.empty())
    {
        NodePtr currentNode = minHeap.top();
        LOG(INFO)<<"currentNode: "<<currentNode->position.transpose();
        cv::Mat tmp_image = debug_image.clone();
        grid_map::Index index;
        map.getIndex(currentNode->position, index);
        LOG(INFO)<<"index: "<<index.transpose();
        cv::Point start_point(index(1), index(0));
        cv::Point end_point(index(1) - 10 * std::sin(currentNode->ori), index(0) - 10 * std::cos(currentNode->ori));
        cv::arrowedLine(tmp_image, start_point, end_point, cv::Scalar(255, 0, 0), 2, cv::LINE_AA, 0, 0.3);
        cv::circle(tmp_image, cv::Point(index(1), index(0)), 5, cv::Scalar(255, 0, 0), -1);
        cv::imshow("debug", tmp_image);
        cv::waitKey(0);

        minHeap.pop();
        if (reachGoal(*currentNode))
        {
            Node goal_node;
            goal_node.position = goal_.head(2);
            goal_node.PreFootstepNode = currentNode;
            goal_node.ori = goal_.z();
            break;
        }
        double yaw;
        if (getYaw(*currentNode, yaw))
        {
            // LOG(INFO)<<"yaw: "<<yaw;
            // // 力决定的方向
            // cv::Point direct_start(index(1), index(0));
            // cv::Point direct_end(index(1) - 10 * std::sin(yaw), index(0) - 10 * std::cos(yaw));
            // cv::arrowedLine(tmp_image, direct_start, direct_end, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.3);
            // cv::imshow("debug", tmp_image);
            // cv::waitKey(0);

            // 获取障碍物势能场
            Eigen::Vector2d SAPf;
            if (getSAPF(*currentNode, SAPf))
            {
                LOG(INFO)<<"SAPF: "<<SAPf.transpose();
                double tmp_yaw = std::atan2(SAPf(1), SAPf(0));
                cv::Point direct_start(index(1), index(0));
                cv::Point direct_end(index(1) - 10 * std::sin(tmp_yaw), index(0) - 10 * std::cos(tmp_yaw));
                cv::arrowedLine(tmp_image, direct_start, direct_end, cv::Scalar(0, 255, 0), 2, cv::LINE_AA, 0, 0.3);
                cv::imshow("debug", tmp_image);
                cv::waitKey(0);
            }
            // 获取终点势能场
            Eigen::Vector2d AttForce;
            if (AttPotential(*currentNode, AttForce))
            {
                LOG(INFO)<<"AttForce: "<<AttForce.transpose();
                double tmp_yaw = std::atan2(AttForce(1), AttForce(0));
                cv::Point direct_start(index(1), index(0));
                cv::Point direct_end(index(1) - 10 * std::sin(tmp_yaw), index(0) - 10 * std::cos(tmp_yaw));
                cv::arrowedLine(tmp_image, direct_start, direct_end, cv::Scalar(0, 0, 255), 2, cv::LINE_AA, 0, 0.3);
                cv::imshow("debug", tmp_image);
                cv::waitKey(0);
            }
            
            std::vector<NodePtr> nodes;
            if (transitions(currentNode, yaw, nodes))
            {
                // 所有扩展的nodes
                for (auto & node : nodes)
                {
                    grid_map::Index node_index;
                    map.getIndex(node->position, node_index);
                    cv::circle(tmp_image, cv::Point(node_index(1), node_index(0)), 1, cv::Scalar(0, 255, 0), -1);
                }
                cv::imshow("debug", tmp_image);
                cv::waitKey(0);

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
        LOG(INFO)<<"next iteration";
    }
    return true;
}


// 允许有一些nan的点，但是不允许有超过的点，
// bool PathPlanning::isFeasible(grid_map::Index & index, double angle)
// {
//     struct FrequencyTracker 
//     {
//         std::unordered_map<int, int> frequencyMap;  // 哈希表，记录每个数的出现次数
//         int mostFrequentNumber;                     // 记录出现次数最多的数
//         int maxFrequency;                           // 记录最大出现次数
//         vector<Eigen::Vector3d> points;
//         FrequencyTracker() : mostFrequentNumber(0), maxFrequency(0) {}
//         void addNumber(int number, Eigen::Vector3d point) {
//             points.emplace_back(point);
//             // 更新哈希表
//             frequencyMap[number]++;
//             // 检查当前数的出现次数是否是最多的
//             if (frequencyMap[number] > maxFrequency) {
//                 mostFrequentNumber = number;
//                 maxFrequency = frequencyMap[number];
//             }
//         }
//         int getMostFrequentNumber() const {
//             return mostFrequentNumber;
//         }
//         int getMaxFrequency() const {
//             return maxFrequency;
//         }
//         vector<Eigen::Vector3d> getPoints()
//         {
//             return points;
//         }        
//     };
// #ifdef DEBUG
//     LOG(INFO)<<"CHECK IMAGE";
//     cv::Mat check_image = pd.result;
// #endif
//     Eigen::AngleAxisd ax(angle, Eigen::Vector3d::UnitZ());
//     grid_map::Position p2;
//     if (map.getPosition(index, p2))
//     {
//         grid_map::Position3 p3 = Eigen::Vector3d::Zero();
//         p3.head(2) = p2;
//         Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(support_area.Up, 0, 0) + p3;
//         Eigen::Vector3d mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- support_area.Button, 0, 0) + p3;
//         Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, support_area.Left, 0) + mid_top;
//         Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - support_area.Right, 0) + mid_top;
//         Eigen::Vector3d down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, support_area.Left, 0) + mid_down;
//         Eigen::Vector3d down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - support_area.Right, 0) + mid_down;
//         grid_map::Position top_left_l(top_left.x(), top_left.y());
//         grid_map::Position top_right_l(top_right.x(), top_right.y());
//         grid_map::Position down_left_l(down_left.x(), down_left.y());
//         grid_map::Position down_right_l(down_right.x(), down_right.y());
//         if (map.isInside(top_left_l) && map.isInside(top_right_l) && map.isInside(down_left_l) && map.isInside(down_right_l))
//         {
//             FrequencyTracker ft;
//             int Nan_number = 0;
//             // 这是一种不合理的，区域内可能有点还没有遍历到，但是还是可以继续采用这种方式
//             grid_map::LineIterator iterator_start(map, down_right_l, down_left_l);
//             grid_map::LineIterator iterator_end(map, top_right_l, top_left_l);
//             for (; !iterator_start.isPastEnd()&&!iterator_end.isPastEnd(); ++iterator_start, ++iterator_end)
//             {
//                 grid_map::Index start_index(*iterator_start);
//                 grid_map::Index end_index(*iterator_end);
//                 for (grid_map::LineIterator iterator_l(map, start_index, end_index); !iterator_l.isPastEnd(); ++iterator_l)
//                 {
//                     const grid_map::Index index_l(*iterator_l);
// #ifdef DEBUG
//                     check_image.at<cv::Vec3b>(index_l.x(), index_l.y()) = cv::Vec3b(0, 0, 0);
//                     // cv::imshow("check_image", check_image);
//                     // cv::imwrite("/home/lichao/catkin_pathplanning/src/path_planning/data/check_image.png", check_image);
//                     // cv::waitKey(0);
// #endif
//                     grid_map::Position3 p3_label;
//                     grid_map::Position3 p3;
//                     if (map.getPosition3("label", index_l, p3_label) && map.getPosition3("elevation", index_l, p3))
//                     {
//                         if (!std::isnan(p3_label.z()) && !std::isnan(p3.z()))
//                         {
//                             int label_index = static_cast<int>(p3_label.z());
//                             ft.addNumber(label_index, p3);
//                         }
//                         else
//                         {
//                             Nan_number++;
//                         }
//                     }
//                     else
//                     {
//                         Nan_number++;
//                     }
//                 }
//             }
//             cv::imshow("check_image", check_image);
//             cv::imwrite("/home/lichao/catkin_pathplanning/src/path_planning/data/check_image.png", check_image);
//             cv::waitKey(0);
//             if (Nan_number < 0.1 * full_size)
//             {
//                 if (ft.getMaxFrequency() > 0.6 * full_size)
//                 {
//                     int plane_index = ft.getMostFrequentNumber();
//                     Eigen::Vector3d normal = pd.planes_info.at(plane_index).normal;
//                     Eigen::Vector3d center = pd.planes_info.at(plane_index).center / 1000.0;
//                     for (auto & point : ft.getPoints())
//                     {
//                         if ((point - center).dot(normal) > 0.01)
//                         {
// #ifdef DEBUG
//                             LOG(INFO)<<"IN foot";
// #endif
//                             return false;
//                         }
//                     }
//                     return true;
//                 }
//                 else
//                 {
// #ifdef DEBUG
//                     LOG(INFO)<<"nan points";
// #endif
//                     return false;
//                 }         
//             }
//             else
//             {
// #ifdef DEBUG
//                 LOG(INFO)<<"support area less";
// #endif
//                 return false;
//             }
//         }
//         else
//         {
// #ifdef DEBUG
//             LOG(INFO)<<"out of map";
// #endif
//             return false;
//         } 
//     }
//     else
//     {
// #ifdef DEBUG
//         LOG(INFO)<<"can not get point";
// #endif
//         return false;
//     }
// }

// bool PathPlanning::getPoints(grid_map::Position TL, grid_map::Position TR, grid_map::Position BL, grid_map::Position BR, vector<Eigen::Vector3d> & return_points)
// {
//     if (map.isInside(TL) && map.isInside(TR) && map.isInside(BL) && map.isInside(BR))
//     {
//         // 这是一种不合理的，区域内可能有点还没有遍历到，但是还是可以继续采用这种方式
//         grid_map::LineIterator iterator_start(map, BR, BL);
//         grid_map::LineIterator iterator_end(map, TR, TL);
//         for (; !iterator_start.isPastEnd()&&!iterator_end.isPastEnd(); ++iterator_start, ++iterator_end)
//         {
//             grid_map::Index start_index(*iterator_start);
//             grid_map::Index end_index(*iterator_end);
//             for (grid_map::LineIterator iterator_l(map, start_index, end_index); !iterator_l.isPastEnd(); ++iterator_l)
//             {
//                 const grid_map::Index index_l(*iterator_l);
//                 grid_map::Position3 p3;
//                 if (map.getPosition3("elevation", index_l, p3))
//                 {
//                     if (!std::isnan(p3.z()))
//                     {
//                         return_points.emplace_back(p3);
//                     }
//                 }
//             }
//         }
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }

// 这种方式的虽然较为真实的表达出支撑面的情况，但是有在我们使用可行方向对应可行域聚类情况会有一个矛盾点
// 在确定非全向的可行域时，将只膨胀脚后跟长度的图-膨胀斜对脚长度的地图相减的区域来检测确定，然而，这种确定方式在上台阶时，假设脚后跟刚好位于某平面上时，他左右转动的角度也是认为可通行的。为了解决这种情况，我们严格要求支撑区域内全部cell都必须位于同一平面上
// bool PathPlanning::isFeasibleNew(grid_map::Index & index, double angle)
// {
//     Eigen::AngleAxisd ax(angle, Eigen::Vector3d::UnitZ());
//     grid_map::Position p2;
//     if (map.getPosition(index, p2))
//     {
//         grid_map::Position3 p3 = Eigen::Vector3d::Zero();
//         p3.head(2) = p2;
//         Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(support_area.Up - 0.08, 0, 0) + p3;
//         Eigen::Vector3d mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- (support_area.Button - 0.03), 0, 0) + p3;
//         Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, support_area.Left -0.04, 0) + mid_top;
//         Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - (support_area.Right - 0.04), 0) + mid_top;
//         Eigen::Vector3d down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, support_area.Left - 0.04, 0) + mid_down;
//         Eigen::Vector3d down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - (support_area.Right - 0.04), 0) + mid_down;
//         grid_map::Position top_left_l(top_left.x(), top_left.y());
//         grid_map::Position top_right_l(top_right.x(), top_right.y());
//         grid_map::Position down_left_l(down_left.x(), down_left.y());
//         grid_map::Position down_right_l(down_right.x(), down_right.y());

//         vector<Eigen::Vector3d> points;
//         if (getPoints(top_left_l, top_right_l, down_left_l, down_right_l, points))
//         {
//             if (points.size() < 4)
//             {
// #ifdef DEBUG
//                 LOG(INFO)<<"points too less";
// #endif
//                 return false;
//             }
//             Eigen::Vector3d sum = Eigen::Vector3d::Zero();
//             for (auto & point : points)
//             {
//                 sum += point;
//             }
//             Eigen::Vector3d center = sum / points.size();
            
//             Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
//             for (auto & point : points)
//             {
//                 M += (point - center)* (point - center).transpose();
//             }
//             Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(M);
//             if (solver.info() == Eigen::Success)
//             {
//                 Eigen::Vector3d eigenvalues = solver.eigenvalues();
//                 Eigen::Matrix3d eigenvectors = solver.eigenvectors();
//                 if (eigenvalues(0)/eigenvalues.sum() < 0.1)
//                 {
//                     Eigen::Vector3d normal = eigenvectors.col(0);
//                     if (eigenvectors.col(0).z() < 0)
//                     {
//                         normal = - eigenvectors.col(0);
//                     }
//                     mid_top = ax.toRotationMatrix() * Eigen::Vector3d(support_area.Up, 0, 0) + p3;
//                     mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- (support_area.Button), 0, 0) + p3;
//                     top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, support_area.Left, 0) + mid_top;
//                     top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - (support_area.Right), 0) + mid_top;
//                     down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, support_area.Left, 0) + mid_down;
//                     down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - (support_area.Right), 0) + mid_down;
//                     top_left_l = grid_map::Position(top_left.x(), top_left.y());
//                     top_right_l = grid_map::Position(top_right.x(), top_right.y());
//                     down_left_l = grid_map::Position(down_left.x(), down_left.y());
//                     down_right_l = grid_map::Position(down_right.x(), down_right.y());
//                     vector<Eigen::Vector3d> all_points;
//                     if (getPoints(top_left_l, top_right_l, down_left_l, down_right_l, all_points))
//                     {
//                         for (auto & point : all_points)
//                         {
//                             if ((point - center).dot(normal) > 0.01 || (point - center).dot(normal) < -0.01)
//                             {
// #ifdef DEBUG
//                                 // LOG(INFO)<<"in foot";
// #endif
//                                 return false;
//                             }
//                         }
//                         return true;
//                     }
//                     else
//                     {
// #ifdef DEBUG
//                             // LOG(INFO)<<"out of map";
// #endif
//                         return false;
//                     }
//                 }
//                 else
//                 {
// #ifdef DEBUG
//                         // LOG(INFO)<<"can not get normal";
// #endif
//                     return false;
//                 }
                
//             }
//             else
//             {
// #ifdef DEBUG
//                 // LOG(INFO)<<"can not get eigen value";
// #endif
//                 return false;
//             }
//         }
//         else
//         {
// #ifdef DEBUG
//             // LOG(INFO)<<"out of map";
// #endif
//             return false;
//         }
//     }
//     else
//     {
// #ifdef DEBUG
//         // LOG(INFO)<<"can not get point";
// #endif
//         return false;
//     }
// }

// 这个过程比较耗时
// void PathPlanning::checkFeasibleDirect()
// {
//     for (int i = 0; i < pd.planes.size(); i++)
//     {
//         cv::Mat image = pd.planes.at(i);
//         // 定义形态学操作的核
//         cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
//         // 应用形态学操作：先膨胀再腐蚀（开操作），去除毛刺
//         cv::Mat smoothedImage;
//         cv::morphologyEx(image, smoothedImage, cv::MORPH_CLOSE, element);
// #ifdef DEBUG
//         // // 找出两个图像中不同的像素
//         // cv::Mat differenceImage;
//         // cv::bitwise_xor(image, smoothedImage, differenceImage);
//         // // 将差异转换为三通道图像以便于可视化
//         // cv::Mat colorDifference;
//         // cv::cvtColor(differenceImage, colorDifference, cv::COLOR_GRAY2BGR);
//         // // 在差异图像上标记不同的像素，使用红色突出显示
//         // for (int i = 0; i < differenceImage.rows; i++) {
//         //     for (int j = 0; j < differenceImage.cols; j++) {
//         //         if (differenceImage.at<uchar>(i, j) != 0) {
//         //             cout<<"draw red"<<endl;
//         //             colorDifference.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);  // 红色
//         //         }
//         //     }
//         // }
//         // cv::imshow("diff_" + std::to_string(i), colorDifference);
//         // cv::waitKey(0);
// #endif
        
//         // 找到0-脚宽/2，这些区域的可通行型为NAN
//         cv::Mat Nan_eroded;
//         cv::Mat half_width_Element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((support_area.Button- 0.05)/map.getResolution() * 2 + 1, (support_area.Button - 0.05)/map.getResolution() * 2 + 1));
//         // 腐蚀erode会导致白色区域变小，黑色区域变大
//         cv::erode(smoothedImage, Nan_eroded, half_width_Element);

// #ifdef DEBUG
        
//         // LOG(INFO)<<cv::countNonZero(smoothedImage)<<" "<<cv::countNonZero(Nan_eroded);
//         // cv::imshow("Nan_eroded"+ std::to_string(i), Nan_eroded);
//         // cv::waitKey(0);
// #endif
//         cv::Mat NanFeasible = smoothedImage - Nan_eroded;
//         // 将所有不可通行的区域合并，只要有一个图像处理是不可通行的，那么就认为该点是不可通行的
//         cv::bitwise_or(Nan_feasible, NanFeasible, Nan_feasible);
// #ifdef DEBUG
//         // cv::imshow("NanFeasible" + std::to_string(i), NanFeasible);
//         // cv::waitKey(0);
// #endif
//         // 找到>前脚掌长度的区域，这些区域的可通行性为full，全向通行
//         cv::Mat Full_eroded;
//         double full_length = std::sqrt(support_area.Left * support_area.Left + support_area.Up * support_area.Up);
//         cv::Mat fore_foot_Element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(full_length/map.getResolution() * 2 + 1, full_length/map.getResolution() * 2 + 1));
//         cv::erode(smoothedImage, Full_eroded, fore_foot_Element);
// #ifdef DEBUG
//         // cv::imshow("Full_eroded"+ std::to_string(i), Full_eroded);
//         // cv::waitKey(0);
// #endif
//         // cv::Mat FullFeasible = smoothedImage - Full_eroded;

//         // 只要一个地方是不全向，那必然不是全向
//         cv::bitwise_or(Full_feasible, Full_eroded, Full_feasible);
// // #ifdef DEBUG
// //         cv::imshow("FullFeasible"+ std::to_string(i), FullFeasible);
// //         cv::waitKey(0);
// // #endif
//         // 对脚宽/2～前脚掌长度，进行
//         cv::Mat CheckImage = Nan_eroded - Full_eroded;

//         // 减去障碍区域，即将CheckImage中与obstacle_layer重叠的区域设置为0
//         CheckImage.setTo(0, obstacle_layer);
// #ifdef DEBUG
//         // cv::imshow("CheckImage"+ std::to_string(i), CheckImage);
//         // cv::waitKey(0);
// #endif
//         std::vector<cv::Point> white_points;
//         cv::findNonZero(CheckImage, white_points);

//         // 这个地方建议加一个openmp   
//         // for (int j = 0; j < white_points.size(); j++)
//         // {
//         //     auto & cv_p = white_points[i];
//         //     LOG(INFO) << cv_p.x << " " << cv_p.y;
//         //     // 定义局部变量以避免线程之间的竞争
//         //     bool Full_feasible_flag = true;
//         //     bool Nan_feasible_flag = false;
//         //      for (int k = 0; k < 72; k++)
//         //     {
//         //         double angle = (k * 5) / 57.3;
//         //         grid_map::Index index(cv_p.y, cv_p.x);
//         //         if (isFeasibleNew(index, angle))
//         //         {
//         //             checks_Mat.at(j).at<uchar>(cv_p) = 255;
//         //             Full_feasible_flag = Full_feasible_flag && true;
//         //             Nan_feasible_flag = Nan_feasible_flag || true;
//         //         }
//         //         else
//         //         {
//         //             checks_Mat.at(j).at<uchar>(cv_p) = 0;
//         //             Full_feasible_flag = Full_feasible_flag && false;
//         //             Nan_feasible_flag = Nan_feasible_flag || false;
//         //         }
//         //     }
//         //     // 对 Full_feasible 和 Nan_feasible 进行并行安全的访问
//         //     if (Full_feasible_flag)
//         //     {
//         //         #pragma omp critical
//         //         {
//         //             if (Full_feasible.at<uchar>(cv_p) == 0)
//         //             {
//         //                 Full_feasible.at<uchar>(cv_p) = 255;
//         //             }
//         //         }
//         //     }
//         //     if (!Nan_feasible_flag)
//         //     {
//         //         #pragma omp critical
//         //         {
//         //             if (Nan_feasible.at<uchar>(cv_p) == 0)
//         //             {
//         //                 Nan_feasible.at<uchar>(cv_p) = 255;
//         //             }
//         //         }
//         //     }
//         // }
//         // #pragma omp parallel for reduction(+:Full_feasible,Nan_feasible)
//         // for (auto & cv_p : white_points)
//         // {
//         //     LOG(INFO)<<cv_p.x<<" "<<cv_p.y;
//         //     // 将其中完全不可通行的点和全向可通行的点挑选出来
//         //     bool Full_feasible_flag = true;
//         //     bool Nan_feasible_flag = false;
//         //     for (int j = 0; j < 72; j++)
//         //     {
//         //         double angle = (j * 5)/57.3;
//         //         grid_map::Index index(cv_p.y, cv_p.x);
//         //         if (isFeasibleNew(index, angle))
//         //         {
//         //             checks_Mat.at(j).at<uchar>(cv_p) = 255;
//         //             Full_feasible_flag = Full_feasible_flag && true;
//         //             Nan_feasible_flag = Nan_feasible_flag || true;
//         //         }
//         //         else
//         //         {
//         //             checks_Mat.at(j).at<uchar>(cv_p) = 0;
//         //             Full_feasible_flag = Full_feasible_flag && false;
//         //             Nan_feasible_flag = Nan_feasible_flag || false;
//         //         }
//         //     }
//         //     if (Full_feasible_flag)
//         //     {
//         //         if (Full_feasible.at<uchar>(cv_p) == 0)
//         //         {
//         //             Full_feasible.at<uchar>(cv_p) = 255;
//         //         }   
//         //     }
//         //     if (!Nan_feasible_flag)
//         //     {
//         //         if (Nan_feasible.at<uchar>(cv_p) == 0)
//         //         {
//         //             Nan_feasible.at<uchar>(cv_p) = 255;
//         //         }  
//         //     }
//         // }
//         auto start = std::chrono::high_resolution_clock::now();
//         // cv::Mat tmp_Full_feasible = cv::Mat::zeros(Full_feasible.size(), CV_8UC1);
//         // cv::Mat tmp_Nan_feasible = cv::Mat::zeros(Nan_feasible.size(), CV_8UC1);
//         // #pragma omp parallel
//         // {
//         //     cv::Mat local_Full_feasible = tmp_Full_feasible.clone();
//         //     cv::Mat local_Nan_feasible = tmp_Nan_feasible.clone();
//         //     #pragma omp for nowait
//         //     for (size_t i = 0; i < white_points.size(); ++i)
//         //     {
//         //         auto & cv_p = white_points[i];
//         //         // LOG(INFO) << cv_p.x << " " << cv_p.y;
//         //         bool Full_feasible_flag = true;
//         //         bool Nan_feasible_flag = false;
//         //         for (int j = 0; j < 72; j++)
//         //         {
//         //             double angle = (j * 5) / 57.3;
//         //             grid_map::Index index(cv_p.y, cv_p.x);
//         //             if (isFeasibleNew(index, angle))
//         //             {
//         //                 checks_Mat.at(j).at<uchar>(cv_p) = 255;
//         //                 Full_feasible_flag = Full_feasible_flag && true;
//         //                 Nan_feasible_flag = Nan_feasible_flag || true;
//         //             }
//         //             else
//         //             {
//         //                 checks_Mat.at(j).at<uchar>(cv_p) = 0;
//         //                 Full_feasible_flag = Full_feasible_flag && false;
//         //                 Nan_feasible_flag = Nan_feasible_flag || false;
//         //             }
//         //         }
//         //         if (Full_feasible_flag)
//         //         {
//         //             if (local_Full_feasible.at<uchar>(cv_p) == 0)
//         //             {
//         //                 local_Full_feasible.at<uchar>(cv_p) = 255;
//         //             }
//         //         }
//         //         if (!Nan_feasible_flag)
//         //         {
//         //             if (local_Nan_feasible.at<uchar>(cv_p) == 0)
//         //             {
//         //                 local_Nan_feasible.at<uchar>(cv_p) = 255;
//         //             }
//         //         }
//         //     }
//         //     #pragma omp critical
//         //     {
//         //         tmp_Full_feasible |= local_Full_feasible;
//         //         tmp_Nan_feasible |= local_Nan_feasible;
//         //     }
//         // } 
//         // Nan_feasible |= tmp_Nan_feasible;
//         // Full_feasible |= tmp_Full_feasible;
//         // 这是不使用openmp加速的代码
//         for (auto & cv_p : white_points)
//         {
//             // LOG(INFO)<<cv_p.x<<" "<<cv_p.y;
//             // 将其中完全不可通行的点和全向可通行的点挑选出来
//             bool Full_feasible_flag = true;
//             bool Nan_feasible_flag = false;
//             for (int j = 0; j < 72; j++)
//             {
//                 double angle = (j * 5)/57.3;
//                 grid_map::Index index(cv_p.y, cv_p.x);
//                 if (isFeasibleNew(index, angle))
//                 {
//                     checks_Mat.at(j).at<uchar>(cv_p) = 255;
//                     Full_feasible_flag = Full_feasible_flag && true;
//                     Nan_feasible_flag = Nan_feasible_flag || true;
//                 }
//                 else
//                 {
//                     checks_Mat.at(j).at<uchar>(cv_p) = 0;
//                     Full_feasible_flag = Full_feasible_flag && false;
//                     Nan_feasible_flag = Nan_feasible_flag || false;
//                 }
//             }
//             if (Full_feasible_flag)
//             {
//                 if (Full_feasible.at<uchar>(cv_p) == 0)
//                 {
//                     Full_feasible.at<uchar>(cv_p) = 255;
//                 }   
//             }
//             // 所有的全部为不可通行时
//             if (!Nan_feasible_flag)
//             {
//                 if (Nan_feasible.at<uchar>(cv_p) == 0)
//                 {
//                     Nan_feasible.at<uchar>(cv_p) = 255;
//                 }  
//             }
//             // 这是不能全向通行只能部分角度通行的像素点
//             if (!Full_feasible_flag && Nan_feasible_flag)
//             {
//                 check_Mat.at<uchar>(cv_p) = 255;
//             }
//         }
//         auto end = std::chrono::high_resolution_clock::now();
//         auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
//         std::cout << "Elapsed time: " << duration << " milliseconds" << std::endl;
//         // 能不能将这里的处理与后续的调换一下顺序，减少计算两
//     }

//     // for (int i = 0; i < checks_Mat.size(); i++)
//     // {
//     //     cv::imshow("check_Mat" + std::to_string(i*5), checks_Mat.at(i));
//     //     cv::waitKey(0);
//     // }
    
//     // 障碍周围的点都为不可通行, 这个怎么还有问题？会导致将可通行平面去除的过多，从而影响可通过性的判断
//     // cv::Mat obstacle_dilated;
//     // double full_length = std::sqrt(support_area.Left * support_area.Left + support_area.Up * support_area.Up);
//     // cv::Mat fore_foot_Element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(full_length/map.getResolution() * 2 + 1, full_length/map.getResolution() * 2 + 1));
//     // cv::dilate(obstacle_layer, obstacle_dilated, fore_foot_Element);
//     // cv::imshow("obstacle_layer", obstacle_layer);
//     // cv::waitKey(0);
//     // cv::imshow("obstacle_dilated", obstacle_dilated);
//     // cv::waitKey(0);
//     // // 将这些不可行的区域筛选出来赋给check_Mat赋0
//     // // cv::imshow("check_Mat onstacle b", check_Mat);
//     // // cv::waitKey(0);
//     check_Mat.setTo(0, obstacle_layer);
//     // cv::imshow("check_Mat onstacle a", check_Mat);
//     // cv::waitKey(0);

//     // 图像的边界区域，将图像边界默认为不可通行，
//     int rows = Nan_feasible.rows;
//     int cols = Nan_feasible.cols;
//     LOG(INFO)<<rows<<" "<<cols;
//     // cv::imshow("Nan_feasible", Nan_feasible);
//     // cv::waitKey(0);
//     // cv::imshow("Full_feasible", Full_feasible);
//     // cv::waitKey(0);
//     double radius = sqrt(support_area.Up * support_area.Up + support_area.Left * support_area.Left);
//     for (int i = 0; i < radius/map.getResolution(); i++)
//     {
//         Nan_feasible.row(i).setTo(255);
//         Nan_feasible.row(rows - i - 1).setTo(255);
//         Nan_feasible.col(i).setTo(255);
//         Nan_feasible.col(cols - i -1).setTo(255);
//         Full_feasible.row(i).setTo(0);
//         Full_feasible.row(rows - i - 1).setTo(0);
//         Full_feasible.col(i).setTo(0);
//         Full_feasible.col(cols - i - 1).setTo(0);
//     }
//     Full_feasible.setTo(0, obstacle_layer);
//     Nan_feasible.setTo(255, obstacle_layer);
//     cv::imshow("check_Mat Nan a", check_Mat);
//     cv::waitKey(0);
//     check_Mat.setTo(0, Nan_feasible);
//     check_Mat.setTo(0, Full_feasible);
//     cv::imshow("check_Mat Nan b", check_Mat);
//     cv::waitKey(0);
// }


// bool PathPlanning::clustering()
// {
//     cv::imshow("check_Mat", check_Mat);
//     cv::waitKey(0);
// // 由于这部分check_Mat还有一些后续处理，所以，feasible_mat与check_Mat不一样
//     // cv::Mat feasible_mat = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
//     // for (int i = 0; i < feasible_mat.rows; i++)
//     // {
//     //     for (int j = 0; j < feasible_mat.cols; j++)
//     //     {
//     //         bool full_flag = true;
//     //         bool Nan_flag = false;
//     //         for (int k = 0; k < 72; k++)
//     //         {
//     //             if (checks_Mat.at(k).at<uchar>(i, j) == 255)
//     //             {
//     //                 full_flag = full_flag && true;
//     //                 Nan_flag = Nan_flag || true; // 为true表示此像素点不是nan
//     //             }
//     //             else
//     //             {
//     //                 full_flag = full_flag && false;
//     //                 Nan_flag = Nan_flag || false;
//     //             }
//     //         }
//     //         // full_flag为false时，表示此像素点不为全向通行
//     //         // Nan_flag 为true时，表示此像素点某些情况下是可通行的
//     //         if (!full_flag && Nan_flag)
//     //         {
//     //             feasible_mat.at<uchar>(i, j) = 255;
//     //         } 
//     //     }
//     // }
//     // cv::imshow("feasible_mat", feasible_mat);
//     // cv::waitKey(0);
//     // cv::Mat region1, region2;
//     // // 第一个图像为 255，第二个图像为 0 的区域
//     // cv::Mat condition1 = (check_Mat == 255) & (feasible_mat == 0);  // 生成满足条件的掩码
//     // condition1.convertTo(region1, CV_8U, 255);  // 将布尔掩码转换为二值图像，白色区域为满足条件的部分
//     // // 第一个图像为 0，第二个图像为 255 的区域
//     // cv::Mat condition2 = (check_Mat == 0) & (feasible_mat == 255);  // 生成满足条件的掩码
//     // condition2.convertTo(region2, CV_8U, 255);  // 转换为二值图像，白色区域为满足条件的部分
//     // // 显示结果
//     // cv::imshow("Region1 (image1=255, image2=0)", region1);
//     // cv::imshow("Region2 (image1=0, image2=255)", region2);
//     // cv::waitKey(0);
//     // return true;
//     // 筛选出更严格的像素点，不能是直接里面总的可通行角度大于strict_section，应该是实际可通行的连续的角度不超过strict_section
//     strictChecksMat = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
//     for (int i = 0; i < strictChecksMat.cols; i++)
//     {
//         for (int j = 0; j < strictChecksMat.rows; j++)
//         {
//             int num = 0;
//             bool flag = false;
//             for(int k = 0; k < 72; k++)
//             {
//                 if (checks_Mat.at(k).at<uchar>(j, i) == 255)
//                 {
//                     flag = true;
//                     num ++;
//                 }
//                 else
//                 {
//                     num = 0; // 会受噪声影响大，先这样吧
//                 }
//                 if (num > strict_section)
//                 {
//                     break;
//                 }
//             }
//             // 不能全部为不可通行
//             if (num <= strict_section && flag)
//             {
//                 if (checks_Mat.at(0).at<uchar>(j, i) == 255 && checks_Mat.at(71).at<uchar>(j, i) == 255)
//                 {
//                     int num_in = 2;
//                     bool flag1 = true;
//                     bool flag2 = true;
//                     int index1 = 1;
//                     int index2 = 70;
//                     while (flag1 || flag2)
//                     {
//                         if (flag1)
//                         {
//                             if (checks_Mat.at(index1).at<uchar>(j, i) == 255)
//                             {
//                                 num_in++;
//                                 index1++;
//                             }
//                             else
//                             {
//                                 flag1 = false;
//                             }
//                         }       
//                         if (flag2)
//                         {
//                             if (checks_Mat.at(index2).at<uchar>(j, i) == 255)
//                             {
//                                 num_in++;
//                                 index2--;
//                             }
//                             else
//                             {
//                                 flag2 = false;
//                             }
//                         }
//                     }
//                     if (num_in <= strict_section)
//                     {
//                         strictChecksMat.at<uchar>(j, i) = 255;
//                     }
//                 }
//                 else
//                 {
//                     strictChecksMat.at<uchar>(j, i) = 255;
//                 }
//             }
//         }
//     }
//     strictChecksMat.setTo(0, check_Mat == 0);
//     cv::imshow("strictChecksMat", strictChecksMat);
//     cv::waitKey(0);
//     std::vector<std::vector<cv::Point>> contours;
//     std::vector<cv::Vec4i> hierarchy;
//     cv::findContours(strictChecksMat, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
//     LOG(INFO)<<contours.size();
//     // 对每一个区域内的每个可通行方向获取其
//     vector<DirectRegion> strict_direct_regions;
//     // 4. 遍历每个轮廓并绘制到结果图像中
//     for (size_t i = 0; i < contours.size(); i++) 
//     {
//         // 3. 创建结果图像，初始化为全黑
//         LOG(INFO)<<"I = "<<i;
//         cv::Mat result = cv::Mat::zeros(check_Mat.size(), CV_8UC1);
//         // 绘制当前轮廓
//         cv::drawContours(result, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED); 
//         cv::bitwise_and(strictChecksMat, result, result);
//         // 对区域极小的区域，就不在考虑，直接不再考虑，对于障碍附近的点也不再考虑
//         if (cv::countNonZero(result) < 10)
//         {
//             continue;
//         }
//         cv::imshow("result" + std::to_string(i), result);
//         cv::waitKey(0);
//         // 每个区域内的像素，根据可通行角度进行聚类
//         std::vector<cv::Point> white_points;
//         cv::findNonZero(result, white_points);
//         // std::unordered_map<int, vector<cv::Point> > histogram;
//         vector< vector<cv::Point> > histogram(72);
//         for (auto cv_point : white_points)
//         {
//             for (int j = 0; j < checks_Mat.size(); j++)
//             {
//                 if (checks_Mat.at(j).at<uchar>(cv_point) == 255)
//                 {
//                     histogram[j].emplace_back(cv_point);
//                 }
//             }
//         }
//         // // 找到最小的，根据投票票数和最小值，取出小于某个阈值的投票，再选取其中间断的，较长的
//         // // for (auto & bin : histogram)
//         // // {
//         // //     LOG(INFO)<<bin.first<<" "<<bin.second.size();
//         // // }
//         // vector<int> x(72);
//         // vector<int> y(72);
//         // for (int j = 0; j < 72; j++)
//         // {
//         //     x.at(j) = j * 5;
//         //     if (!histogram.at(j).empty())
//         //     {
//         //         y.at(j) = histogram[j].size();
//         //     }
//         //     else
//         //     {
//         //         y.at(j) = 0;
//         //     }
//         // }
//         // matplotlibcpp::title("Sample Histogram");
//         // matplotlibcpp::xlabel("Category");
//         // matplotlibcpp::ylabel("Values");
//         // // 绘制图表
//         // matplotlibcpp::plot(x, y);
//         // // 显示图表
//         // matplotlibcpp::show();
//         // // 一个区域可能会有多个直方图
//         vector< std::unordered_map<int, vector<cv::Point> > > histograms;
//         // 只不过多了一个索引，并没有顺序
//         std::unordered_map<int, vector<cv::Point> > single_histogram_tmp;
//         for (int j = 0; j < histogram.size(); j++)
//         {
//             if (histogram.at(j).size() > 4)
//             {
//                 single_histogram_tmp[j] = histogram.at(j);
//             }
//             else
//             {
//                 if (!single_histogram_tmp.empty())
//                 {
//                     histograms.emplace_back(single_histogram_tmp);
//                 }
//                 single_histogram_tmp.clear();
//             }
//         } 
//         // 检查最后一个 histogram 是否被保存
//         if (!single_histogram_tmp.empty())
//         {
//             histograms.emplace_back(single_histogram_tmp);
//         }
//         cout<<"result: "<<endl;
//         cout<<histograms.size()<<endl;
//         for(auto & his : histograms)
//         {
//             for(auto & bin : his)
//             {
//                 std::cout<<"[ "<<bin.first<<", "<<bin.second.size()<<" ]"<<" ";
//             }
//             // std::cout<<std::endl;
//         }
//         cout<<"next........."<<endl; 
//         // 一个区域内会有多个直方图，但是直方图肯定是顺序排列的，所以只需要检查第一个和最后一个是否分别包含0和71，即可判断该区域内的首尾两个直方图是否需要合并
//         // 检查首尾需不需要合并 
//         if (histograms.front().find(0) != histograms.front().end() && histograms.back().find(71) != histograms.back().end())
//         {
//             for (auto bin : histograms.back())
//             {
//                 histograms.front()[bin.first] = bin.second;
//             }
//             histograms.erase(histograms.end() - 1);
//         }
//         for(auto & his : histograms)
//         {
//             for(auto & bin : his)
//             {
//                 std::cout<<"[ "<<bin.first<<", "<<bin.second.size()<<" ]"<<" ";
//             }
//             std::cout<<std::endl;
//         } 
//         // 如何考虑首尾的情况
//         for (auto & his : histograms)
//         {
//             // LOG(INFO)<<"IN...";
//             DirectRegion strict_direct_region;
//             cv::Mat region = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
//             // 如果即包含0也包含71，且size小于72，那么是首尾相连的情况
//             if (his.find(0) != his.end() && his.find(71) != his.end() && his.size() < 72)
//             {
//                 // LOG(INFO)<<"IN---...";
//                 int max = 0;
//                 int min = 71;
//                 int range = 0;
//                 bool min_flag = true;
//                 bool max_flag = true;
//                 while (min_flag || max_flag)
//                 {
//                     if (min_flag)
//                     {
//                         if (his.find(min) != his.end())
//                         {
//                             for (auto & cv_point : his[min])
//                             {
//                                 region.at<uchar>(cv_point) = 255;
//                             }
//                             min--;
//                             range++;
//                         }
//                         else
//                         {
//                             min_flag = false;
//                         }
//                     }
//                     if (max_flag)
//                     {
//                         if (his.find(max) != his.end())
//                         {
//                             for (auto & cv_point : his[max])
//                             {
//                                 region.at<uchar>(cv_point) = 255;
//                             }
//                             max++;
//                             range++;
//                         }
//                         else
//                         {
//                             max_flag = false;
//                         }
//                     }
//                 }
//                 LOG(INFO)<<min<<" "<<max;
//                 strict_direct_region.min = min;
//                 strict_direct_region.range = range;
//                 strict_direct_region.region = region;
//             }
//             else
//             {
//                 // LOG(INFO)<<"IN..DD.";
//                 if (!his.empty())
//                 {
//                     int max = -1;
//                     int min = 72;        
//                     for (auto & bin : his)
//                     {
//                         if (bin.first > max)
//                         {
//                             max = bin.first;
//                         }
//                         if (bin.first < min)
//                         {
//                             min = bin.first;
//                         }
//                         for (auto & cv_point : bin.second)
//                         {
//                             region.at<uchar>(cv_point) = 255;
//                         }
//                     }
//                     LOG(INFO)<<min<<" "<<max;
//                     strict_direct_region.min = min;
//                     strict_direct_region.range = his.size();
//                     LOG(INFO)<<"range: "<<his.size();
//                     strict_direct_region.region = region;
//                     drs.emplace_back(strict_direct_region);
//                 }
//             }   
//         }
//     }
//     for (auto & dr : drs)
//     {
//         LOG(INFO)<<dr.min<<" "<<dr.range<<endl;
//         cv::imshow("region", dr.region);
//         cv::waitKey(0);
//     }
//     // return true;
//     // 获取一般的区域，并根据严格通行区域是否重叠，并根据每个重叠区域内的非重叠栅格来进一步核查region
//     std::vector<std::vector<cv::Point>> contours_feasible;
//     std::vector<cv::Vec4i> hierarchy_feasible;
//     // 只提取外轮廓
//     // LOG(INFO)<<"check_Mat--------------";          
//     // cv::imshow("check_Mat***           ", check_Mat);
//     // cv::waitKey(0);
//     // cv::findContours(strictChecksMat, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
//     // cv::findContours(check_Mat, contours_feasible, hierarchy_feasible, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
//     cv::findContours(check_Mat, contours_feasible, hierarchy_feasible, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
//     for (size_t i = 0; i < contours_feasible.size(); i++) 
//     {
//         // 如果有父轮廓，则不需要管
//         // if (hierarchy[i][3] != -1)
//         // {
//         //     continue;
//         // }   
//         // 3. 创建结果图像，初始化为全黑
//         cv::Mat result = cv::Mat::zeros(check_Mat.size(), CV_8UC1);
//         // 绘制当前轮廓
//         cv::drawContours(result, contours_feasible, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
//         cv::bitwise_and(check_Mat, result, result);
//         // cv::imshow("result", result);
//         // cv::waitKey(0);
//         // 如果有重叠
//         for (int j = 0; j < drs.size(); j++)
//         {
//             cv::Mat intersection;
//             cv::bitwise_and(result, drs.at(j).region, intersection);
//             // 判断交集区域内是否有非零像素
//             if (cv::countNonZero(intersection) > 8)
//             {
//                 cv::Mat nonOverlap = result - drs.at(j).region;
//                 std::vector<cv::Point> white_points;
//                 cv::findNonZero(nonOverlap, white_points);
//                 for (auto & cv_point : white_points)
//                 {
//                     bool flag = false;
//                     for (int k = 0; k < drs.at(j).range; k++)
//                     {
//                         int index;
//                         if (drs.at(j).min + k >= 72)
//                         {
//                             index = drs.at(j).min + k -72;
//                         }
//                         else
//                         {
//                             index = drs.at(j).min + k;
//                         }
//                         if (checks_Mat.at(index).at<uchar>(cv_point) == 0)
//                         {
//                             flag = false;
//                             break;
//                         } 
//                         else
//                         {
//                             flag = true;
//                         }
//                     }
//                     if (flag)
//                     {
//                         drs.at(j).region.at<uchar>(cv_point) = 255;
//                     }
                    
//                 }   
//             }
//         } 
//     }
//     for (auto & dr : drs)
//     {
//         LOG(INFO)<<dr.min<<" "<<dr.range;
//         cv::imshow("region----", dr.region);
//         cv::waitKey(0);
//     }
//     // 将里面的每个region先腐蚀再膨胀，将里面的一个区域转成多个区域
//     // 如果区域本来就比较小，进行小半径膨胀之后变成没有洞的，就直接把这个区域填充
//     auto tmp_drs = drs;
//     drs.clear();
//     for (auto & dr : tmp_drs)
//     {
//         cv::Mat temp_image = dr.region;
//         // 确定该区域的外轮廓
//         std::vector<std::vector<cv::Point>> contours_out;
//         std::vector<cv::Vec4i> hierarchy_out;
//         cv::findContours(temp_image, contours_out, hierarchy_out, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
//         // 如果本身空洞很小，或者无洞，就直接使用原方向
//         double inflation_radius0 = 0.04;
//         int inflation_pixels0 = inflation_radius0/map.getResolution();
//         cv::Mat inflation_element0 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(inflation_pixels0 * 2 + 1, inflation_pixels0 * 2 + 1));
//         // 方向更单一
//         cv::Mat direct_image0;
//         cv::dilate(temp_image, direct_image0, inflation_element0);
//         std::vector<std::vector<cv::Point>> contours0;
//         std::vector<cv::Vec4i> hierarchy0;
//         cv::findContours(direct_image0, contours0, hierarchy0, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
//         bool small_hole_flag = true;
//         for (int i = 0; i < contours0.size(); i++)
//         {
//             if (hierarchy0[i][3] != -1) // != -1, 表示有空洞
//             {
//                 small_hole_flag = false;
//                 break;
//             }
//         }
//         // 不是small hole的都不用要，默认成full
//         if (small_hole_flag)// 直接填充后作为区域
//         {
//             cv::Mat region = cv::Mat::zeros(check_Mat.size(), CV_8UC1);
//             // 求取区域的外轮廓
//             for (int i = 0; i < contours_out.size(); i++)
//             {
//                 cv::drawContours(region, contours_out, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
//             }
//             DirectRegion temp_dr;
//             temp_dr.min = dr.min;
//             temp_dr.range = dr.range;
//             temp_dr.region = region;
//             drs.emplace_back(temp_dr);
//             continue;
//         }
//         // 如果这个区域经过0.15m的膨胀以后变成无动的，那么将此区域填充后的所有区域均为此方向对应的区域，使用扩充的方向
//         double inflation_radius1 = 0.15;
//         int inflation_pixels1 = inflation_radius1/map.getResolution();
//         cv::Mat inflation_element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(inflation_pixels1 * 2 + 1, inflation_pixels1 * 2 + 1));
//         // 方向更单一
//         cv::Mat direct_image1;
//         cv::dilate(temp_image, direct_image1, inflation_element1);
//         std::vector<std::vector<cv::Point>> contours1;
//         std::vector<cv::Vec4i> hierarchy1;
//         cv::findContours(direct_image1, contours1, hierarchy1, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
//         small_hole_flag = true;
//         for (int i = 0; i < contours1.size(); i++)
//         {
//             if (hierarchy1[i][3] != -1)
//             {
//                 small_hole_flag = false;
//                 break;
//             }
//         }
//         // 不是small hole的都不用要，默认成full
//         if (small_hole_flag)// 直接填充后作为区域
//         {
//             cv::Mat region = cv::Mat::zeros(check_Mat.size(), CV_8UC1);
//             for (int i = 0; i < contours_out.size(); i++)
//             {
//                 cv::drawContours(region, contours_out, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
//             }
//             // 重新提取可通行角度， 使用区域内的可通行区域进行拟合得到新的range
//             DirectRegion temp_dr;
//             temp_dr.min = dr.min - 1;
//             temp_dr.range = dr.range + 2;
//             temp_dr.region = region;
//             drs.emplace_back(temp_dr);
//             continue;
//         }
//         // 如果填充0.3m后变成无洞的区域，
//         double inflation_radius2 = 0.3;
//         int inflation_pixels2 = inflation_radius2/map.getResolution();
//         cv::Mat inflation_element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(inflation_pixels2 * 2 + 1, inflation_pixels2 * 2 + 1));
//         cv::Mat direct_image2;
//         cv::dilate(temp_image, direct_image2, inflation_element2);
//         std::vector<std::vector<cv::Point>> contours2;
//         std::vector<cv::Vec4i> hierarchy2;
//         cv::findContours(direct_image2, contours2, hierarchy2, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
//         small_hole_flag = true;  
//         for (int i = 0; i < contours2.size(); i++)
//         {
//             if (hierarchy2[i][3] != -1)
//             {
//                 small_hole_flag = false;
//                 break;
//             }
//         }
//         // 不是small hole的都不用要，默认成full
//         if (small_hole_flag)// 直接填充后作为区域
//         {
//             cv::Mat region = cv::Mat::zeros(check_Mat.size(), CV_8UC1);
//             for (int i = 0; i < contours_out.size(); i++)
//             {
//                 cv::drawContours(region, contours_out, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
//             }
//             // 重新提取可通行角度 使用区域内的可通行区域进行拟合得到新的range
//             DirectRegion temp_dr;
//             temp_dr.min = dr.min - 2;
//             temp_dr.range = dr.range + 4;
//             temp_dr.region = region;
//             drs.emplace_back(temp_dr);
//         }
//         // 如果需要超过0.3m的膨胀半径，这意味着这可能是一个0.6*0.6的平台。可以将此区域视为全向可通行区域
//     }
//     return true;
// }

// 需要将可通行区域合并，一些离得比较近且方向大致相同的区域合并，例如台阶的多级都可以合并，合并区域内不能包含障碍区域。
// 合并后的区域可以使用膨胀后两区域的相交部分内没有障碍区域
// 障碍区域的定义：两个平面在某处的高度差达到某个阈值，所以，之前定义的障碍点信息不充分
// void PathPlanning::clusterFeasibleRegions()
// {
//     Graph g;
//     for(int i = 0; i < drs.size() - 1; i++)
//     {
//         for (int j = 0; j < drs.size(); j++)
//         {
//             // 如果两个区域膨胀后，有没有相交的区域
//             cv::Mat image1 = drs.at(i).region;
//             cv::Mat image2 = drs.at(j).region;
//             int g_nStructBLlementSize = 0.2/map.getResolution();
//             cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * g_nStructBLlementSize + 1, 2 * g_nStructBLlementSize + 1 ));
//             cv::dilate(image1, image1, element);
//             cv::dilate(image2, image2, element);
//             cv::Mat intersection;
//             cv::bitwise_and(image1, image2, intersection);
//             // 判断交集区域内是否有非零像素
//             if (cv::countNonZero(intersection) > 0)
//             {
//                 double angle1 = drs.at(i).min + ((double)(drs.at(i).range) - 1.0)/2.0;
//                 double angle2 = drs.at(j).min + ((double)(drs.at(j).range) - 1.0)/2.0;
//                 LOG(INFO)<<"angle1: "<<angle1<<", angle2: "<<angle2;
//                 if (abs(angle1 - angle2)*5 < 15)//如果小于15度
//                 {
//                     g.addEdge(i, j);
//                     g.addEdge(j, i);
//                 }
//             }
//         }
//     }
//     // 找到所有聚类
//     vector<vector<int>> clusters = g.findClusters();
//     for (auto & cluster : clusters)
//     {
//         for (auto & index : cluster)
//         {
//             cout<<index<<" ";
//         }
//         cout<<endl;
//     }
//     // 根据聚类构造
//     // CompareDR compareDR(map);
//     // for (auto & cluster : clusters)
//     // {
//     //     MergedDirectRegion msr;
//     //     cv::Mat merged_region = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
//     //     for (auto & ds : cluster)
//     //     {
//     //         merged_region.setTo(255, drs.at(ds).region);
//     //         msr.merged_direct_regions.emplace_back(drs.at(ds));
//     //     }
//     //     msr.merged_region = merged_region;
//     //     std::sort(msr.merged_direct_regions.begin(), msr.merged_direct_regions.end(), compareDR);
//     // }
//     // 将聚类的块连成一串，并根据流体的形式引入吸引力，从而将一些不可能到达的区域直接不考虑
// }
// vector<int> PathPlanning::getPlanePath(int start_plane, int goal_plane)
// {
//     struct Node
//     {
//         int index, pre_index;
//     }
//     // 也不是每个平面只能一次
// }
// std::vector<Node> PathPlanning::_convertClosedListToPath(std::unordered_map<int, Node>& closed_list, const Node& start, const Node& goal)
// {
//   std::vector<Node> path;
//   auto current = closed_list.find(goal.id());
//   while (current->second != start)
//   {
//     path.push_back(current->second);
//     auto it = closed_list.find(current->second.pid());
//     if (it != closed_list.end())
//       current = it;
//     else
//       return {};
//   }
//   path.push_back(start);
//   return path;
// }
// void PathPlanning::testisFeasible()
// {
//     grid_map::Index index(200, 92);
//     double angle = 20/57.3;
//     if (isFeasibleNew(index, angle))
//     // if (isFeasible(index, angle))
//     {
//         LOG(INFO)<<"is feasible";
//     }
//     else
//     {
//         LOG(INFO)<<"NOT FEASIBLE";
//     }
// }

// bool PathPlanning::plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand)
// {
//     path.clear();
//     expand.clear();
//     sample_list_.clear();
//     start_ = start, goal_ = goal;
//     sample_list_.insert(std::make_pair(start.id(), start));
//     expand.push_back(start);

//     int iteration = 0;
//     bool optimized = false;
//     while (iteration < sample_num_)
//     {
//         Node sample_node = _generateRandomNode();
//         Node new_node = _findNearestPoint(sample_list_, sample_node);
//         if (new_node.id() == -1)
//             continue;
//         else
//         {
//             sample_list_.insert(std::make_pair(new_node.id(), new_node));
//             expand.push_back(new_node);
//         }
//         if (_checkGoal(new_node))
//         {
//             path = _convertClosedListToPath(sample_list_, start, goal);
//             return true;
//         }
//     }
//     return false;
// }

// Node PathPlanning::_generateRandomNode()
// {
//     // obtain a random number from hardware
//     std::random_device rd;

//     // seed the generator
//     std::mt19937 eng(rd());

//     // define the range
//     std::uniform_real_distribution<float> p(0, 1);

//     // heuristic
//     if (p(eng) > opti_sample_p_)
//     {
//         // generate node
//         std::uniform_int_distribution<int> distr(0, map_size_ - 1);
//         const int id = distr(eng);
//         // 将index转成坐标
//         int index_x = static_cast<int>(id/map.getSize().y());
//         int index_y;
//         if (id/map.getSize().y() == 0)
//         {
//             index_y = static_cast<int>(map.getSize().y() - 1);
//         }
//         else
//         {
//             index_y = id/map.getSize().y() - 1;
//         }

//         grid_map::Index index(index_x, index_y);
//         grid_map::Position p2;
//         if (map.getPosition(index, p2))
//         {
//             return Node(p2.x(), p2.y(), 0, 0, id, -1);
//         }
//         else
//         {
//             LOG(ERROR)<<" cannot get a rand point";

//         }
//     }
//     else
//         return Node(goal_.x(), goal_.y(), 0, 0, goal_.id(), -1);
// }

// Node PathPlanning::_findNearestPoint(std::unordered_map<int, Node>& list, const Node& node)
// {
//     Node nearest_node, new_node(node);
//     double min_dist = std::numeric_limits<double>::max();
//     for (const auto& p : list)
//     {
//         // calculate distance
//         double new_dist = helper::dist(p.second, new_node);

//         // update nearest node
//         if (new_dist < min_dist)
//         {
//             nearest_node = p.second;
//             new_node.set_pid(nearest_node.id());
//             new_node.set_g(new_dist + p.second.g());
//             min_dist = new_dist;
//         }
//     }

//     // distance longer than the threshold
//     if (min_dist > max_dist_)
//     {
//         // connect sample node and nearest node, then move the nearest node
//         // forward to sample node with `max_distance` as result
//         double theta = helper::angle(nearest_node, new_node);
//         new_node.set_x(nearest_node.x() + static_cast<int>(max_dist_ * cos(theta)));
//         new_node.set_y(nearest_node.y() + static_cast<int>(max_dist_ * sin(theta)));
//         new_node.set_id(node2Index(new_node));
//         new_node.set_g(max_dist_ + nearest_node.g());
//     }
//     // already in tree or collide
//     if (list.count(new_node.id()) || _isAnyObstacleInPath(new_node, nearest_node))
//         new_node.set_id(-1);
//     return new_node;
// }

// int PathPlanning::node2Index(Node & n)
// {
//     grid_map::Index index;
//     map.getIndex(grid_map::Position(n.x(), n.y()), index);
//     int id = index.x() * map.getSize().y() + index.y() + 1;
//     return id;
// }
// 将两节点的连线与障碍区域连线，判断其是否与障碍相交
// bool PathPlanning::_isAnyObstacleInPath(const Node& n1, const Node& n2)
// {
//     // double theta = helper::angle(n1, n2);
//     // double dist = helper::dist(n1, n2);

//     // // distance longer than the threshold
//     // if (dist > max_dist_)
//     //     return true;

//     // // sample the line between two nodes and check obstacle
//     // float resolution = costmap_->getResolution();
//     // int n_step = static_cast<int>(dist / resolution);
//     // for (int i = 0; i < n_step; i++)
//     // {
//     //     float line_x = (float)n1.x() + (float)(i * resolution * cos(theta));
//     //     float line_y = (float)n1.y() + (float)(i * resolution * sin(theta));
//     //     if (costmap_->getCharMap()[grid2Index(static_cast<int>(line_x), static_cast<int>(line_y))] >=
//     //         costmap_2d::LETHAL_OBSTACLE * factor_)
//     //     return true;
//     // }

//     grid_map::Index index1, index2;
//     if (map.getIndex(grid_map::Position(n1.x(), n1.y()), index1) && map.getIndex(grid_map::Position(n2.x(), n2.y()), index2))
//     {
//         cv::Mat line = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
//         cv::line(line, cv::Point(index1.y(), index1.x()), cv::Point(index2.y(), index2.x()), 255, 2, cv::LINE_AA);
//         // 进行按位与操作
//         cv::Mat result;
//         cv::bitwise_and(line, obstacle_layer, result);

//         // 检查结果图像是否有白色像素
//         if (cv::countNonZero(result) > 0) 
//         {
//             return true;
//         }
//         else
//         {
//             return false;
//         }

//     }
//     else
//     {
//         LOG(ERROR)<<"error node";
//         return true;
//     }
// }


// bool PathPlanning::_checkGoal(const Node& new_node)
// {
//     auto dist = helper::dist(new_node, goal_);
//     if (dist > max_dist_)
//         return false;

//     if (!_isAnyObstacleInPath(new_node, goal_))
//     {
//         Node goal(goal_.x(), goal_.y(), dist + new_node.g(), 0, node2Index(goal_), new_node.id());
//         sample_list_.insert(std::make_pair(goal.id(), goal_));
//         return true;
//     }
//     return false;
// }


// 根据最近节点，随机节点，终点，障碍，最近节点的方向来生成新的节点
// 后面可以做一个不用考虑当前点方向的方式，并对比。第二种方式需要再进行一步平滑步骤
// Node PathPlanning::APF_newpoint(Node & nearest_point, Node & rand_point)
// {
//     // len = norm((random_point - near_point(1:2)),2);     % The distance between the sampling point and the nearest node
//     // len1 = norm((goal_pose - near_point(1:2)),2);       % The distance between the target point and the nearest node
//     // F1 = REP_F(near_point,random_point,obstacle11);
//     // F2 = REP_F(near_point,random_point,obstacle22);
//     // F3 = REP_F(near_point,random_point,obstacle33);
//     // if (F1 + F2 + F3) == 0
//     //     F = [0,0];
//     // else
//     //     F = krep*(F1 + F2 + F3)/norm((F1 + F2 + F3),2);
//     // end    
//     // if(len < step)
//     //     tem_step = len;
//     // else
//     //     tem_step = step;
//     // end
//     // % 加上机器人当前的朝向
//     // U = (random_point - near_point(1:2))/len+kp*(goal_pose - near_point(1:2))/len1+F;
//     // new_thea = near_point(1:2)+tem_step*U/norm(U,2);
//     double len = helper::dist(nearest_point, rand_point);
//     double len1 = helper::dist(nearest_point, goal_);
//     vector<Eigen::Vector2d> Fs;
//     for (auto obstacle : obstacles)
//     {
//         /* code */
//     }
//     double tem_step;
//     if (len < tem_step)
//     {
//         tem_step = len;
//     }
//     else
//     {
//         tem_step = step;
//     }
//     Eigen::Vector2d random_point(rand_point.x(), rand_point.y());
//     Eigen::Vector2d nearest_point_(nearest_point.x(), nearest_point.y());
//     Eigen::Vector2d goal_point(goal_.x(), goal_.y());

//     // 加上当前方向的偏移量
    
// }

// Eigen::Vector2d PathPlanning::repF()
// {
    
// }

// void PathPlanning::computeObstacles()
// {
//     LOG(INFO)<<"in func computeObstacles";
//     double radius = 0.2;
//     int gridSize = radius/map.getResolution();
//     LOG(INFO)<<"gridSize: "<<gridSize;
//     cv::Mat obstacle_points = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
//     for (int y = 0; y < obstacle_layer.rows; y+=gridSize)
//     {
//         for (int x = 0; x < obstacle_layer.cols; x+=gridSize)
//         {
//             cv::Rect grid(x, y, gridSize, gridSize);
//             grid &= cv::Rect(0, 0, obstacle_layer.cols, obstacle_layer.rows);
//             cv::Mat gridArea = obstacle_layer(grid);
//             // cv::imshow("gridArea", gridArea); 
//             // cv::waitKey(0);
//             cv::Moments m = cv::moments(gridArea, true);
//             if (m.m00 != 0)
//             {
//                 // 计算重心 (cx, cy)
//                 int centerX = static_cast<int>(m.m10 / m.m00);
//                 int centerY = static_cast<int>(m.m01 / m.m00);
//                 // 保证坐标不超出图像边界
//                 int globalCenterX = x + centerX;
//                 int globalCenterY = y + centerY;
//                 globalCenterX = std::min(globalCenterX, obstacle_layer.cols - 1);
//                 globalCenterY = std::min(globalCenterY, obstacle_layer.rows - 1);
//                 // LOG(INFO)<<globalCenterX<<" "<<globalCenterY;
//                 obstacle_points.at<uchar>(globalCenterY, globalCenterX) = 255;
//                 // // 在稀疏图像中标记重心
//                 // sparseImage.at<uchar>(globalCenterY, globalCenterX) = 255;
//                 grid_map::Index globalCenter(globalCenterY, globalCenterX);
//                 grid_map::Position p2;
//                 map.getPosition(globalCenter, p2);
//                 obstacles.emplace_back(p2);
//             }
//         }
//     }
//     cv::imshow("obstacle_points", obstacle_points);
//     cv::waitKey(0);
// }

PathPlanning::~PathPlanning()
{
}