#include <path_planning/planning.h>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <Eigen/Dense>
#include <omp.h>
#include <chrono>
#include <matplotlibcpp.h>
PathPlanning::PathPlanning(ros::NodeHandle & nodehand, grid_map::GridMap & map_, SupportArea support_area_):nh(nodehand), map(map_), support_area(support_area_)
{
    int x_size = ceil((support_area.Button + support_area.Up)/map.getResolution());
    int y_size = ceil((support_area.Left + support_area.Right)/map.getResolution());
    full_size = x_size * y_size;

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
        if (pd.planes_info.at(i).first.z() < 0)
        {
            pd.planes_info.at(i).first = - pd.planes_info.at(i).first;
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
}

void PathPlanning::constructObstacleLayer(int chect_scope)
{
    obstacle_layer = cv::Mat::zeros(pd.result.size(), CV_8UC1);
    for (int i = 0; i < pd.planes.size(); i++)
    {
        cv::Mat image = pd.planes.at(i);
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
                                    normals[label_index] = pd.planes_info.at(label_index).first;
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
                    else
                    {
                        // 如果角度和高度差都满足，那证明该点内的各平面是可联通的
                        for (auto it1 = normals.begin(); it1 != normals.end(); ++it1)
                        {
                            auto it2 = it1;  // 从 it1 开始，避免重复的边
                            for (++it2; it2 != normals.end(); ++it2)
                            {
                                if (it1->first != it2->first)
                                {
                                    // 这几个平面是相互连通的
                                    plane_graph.addEdge(it1->first, it2->first);
                                }
                            }
                        }
                    }
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
}

// 允许有一些nan的点，但是不允许有超过的点，
bool PathPlanning::isFeasible(grid_map::Index & index, double angle)
{
    struct FrequencyTracker 
    {
        std::unordered_map<int, int> frequencyMap;  // 哈希表，记录每个数的出现次数
        int mostFrequentNumber;                     // 记录出现次数最多的数
        int maxFrequency;                           // 记录最大出现次数
        vector<Eigen::Vector3d> points;
        FrequencyTracker() : mostFrequentNumber(0), maxFrequency(0) {}

        void addNumber(int number, Eigen::Vector3d point) {
            points.emplace_back(point);
            // 更新哈希表
            frequencyMap[number]++;

            // 检查当前数的出现次数是否是最多的
            if (frequencyMap[number] > maxFrequency) {
                mostFrequentNumber = number;
                maxFrequency = frequencyMap[number];
            }
        }

        int getMostFrequentNumber() const {
            return mostFrequentNumber;
        }

        int getMaxFrequency() const {
            return maxFrequency;
        }

        vector<Eigen::Vector3d> getPoints()
        {
            return points;
        }        
    };

#ifdef DEBUG
    LOG(INFO)<<"CHECK IMAGE";
    cv::Mat check_image = pd.result;
#endif

    Eigen::AngleAxisd ax(angle, Eigen::Vector3d::UnitZ());
    grid_map::Position p2;
    if (map.getPosition(index, p2))
    {
        grid_map::Position3 p3 = Eigen::Vector3d::Zero();
        p3.head(2) = p2;
        Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(support_area.Up, 0, 0) + p3;
        Eigen::Vector3d mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- support_area.Button, 0, 0) + p3;
        Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, support_area.Left, 0) + mid_top;
        Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - support_area.Right, 0) + mid_top;
        Eigen::Vector3d down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, support_area.Left, 0) + mid_down;
        Eigen::Vector3d down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - support_area.Right, 0) + mid_down;
        grid_map::Position top_left_l(top_left.x(), top_left.y());
        grid_map::Position top_right_l(top_right.x(), top_right.y());
        grid_map::Position down_left_l(down_left.x(), down_left.y());
        grid_map::Position down_right_l(down_right.x(), down_right.y());
        if (map.isInside(top_left_l) && map.isInside(top_right_l) && map.isInside(down_left_l) && map.isInside(down_right_l))
        {
            FrequencyTracker ft;
            int Nan_number = 0;
            // 这是一种不合理的，区域内可能有点还没有遍历到，但是还是可以继续采用这种方式
            grid_map::LineIterator iterator_start(map, down_right_l, down_left_l);
            grid_map::LineIterator iterator_end(map, top_right_l, top_left_l);
            for (; !iterator_start.isPastEnd()&&!iterator_end.isPastEnd(); ++iterator_start, ++iterator_end)
            {
                grid_map::Index start_index(*iterator_start);
                grid_map::Index end_index(*iterator_end);
                for (grid_map::LineIterator iterator_l(map, start_index, end_index); !iterator_l.isPastEnd(); ++iterator_l)
                {
                    const grid_map::Index index_l(*iterator_l);
#ifdef DEBUG
                    check_image.at<cv::Vec3b>(index_l.x(), index_l.y()) = cv::Vec3b(0, 0, 0);
                    // cv::imshow("check_image", check_image);
                    // cv::imwrite("/home/lichao/catkin_pathplanning/src/path_planning/data/check_image.png", check_image);
                    // cv::waitKey(0);
#endif
                    grid_map::Position3 p3_label;
                    grid_map::Position3 p3;
                    if (map.getPosition3("label", index_l, p3_label) && map.getPosition3("elevation", index_l, p3))
                    {
                        if (!std::isnan(p3_label.z()) && !std::isnan(p3.z()))
                        {
                            int label_index = static_cast<int>(p3_label.z());
                            ft.addNumber(label_index, p3);
                        }
                        else
                        {
                            Nan_number++;
                        }
                    }
                    else
                    {
                        Nan_number++;
                    }
                }
            }
            cv::imshow("check_image", check_image);
            cv::imwrite("/home/lichao/catkin_pathplanning/src/path_planning/data/check_image.png", check_image);
            cv::waitKey(0);
            if (Nan_number < 0.1 * full_size)
            {
                if (ft.getMaxFrequency() > 0.6 * full_size)
                {
                    int plane_index = ft.getMostFrequentNumber();
                    Eigen::Vector3d normal = pd.planes_info.at(plane_index).first;
                    Eigen::Vector3d center = pd.planes_info.at(plane_index).second / 1000.0;
                    for (auto & point : ft.getPoints())
                    {
                        if ((point - center).dot(normal) > 0.01)
                        {
#ifdef DEBUG
                            LOG(INFO)<<"IN foot";
#endif
                            return false;
                        }
                    }
                    return true;
                }
                else
                {
#ifdef DEBUG
                    LOG(INFO)<<"nan points";
#endif
                    return false;
                }
                
            }
            else
            {
#ifdef DEBUG
                LOG(INFO)<<"support area less";
#endif
                return false;
            }
        }
        else
        {
#ifdef DEBUG
            LOG(INFO)<<"out of map";
#endif
            return false;
        } 
    }
    else
    {
#ifdef DEBUG
        LOG(INFO)<<"can not get point";
#endif
        return false;
    }
}

bool PathPlanning::getPoints(grid_map::Position TL, grid_map::Position TR, grid_map::Position BL, grid_map::Position BR, vector<Eigen::Vector3d> & return_points)
{
    if (map.isInside(TL) && map.isInside(TR) && map.isInside(BL) && map.isInside(BR))
    {
        // 这是一种不合理的，区域内可能有点还没有遍历到，但是还是可以继续采用这种方式
        grid_map::LineIterator iterator_start(map, BR, BL);
        grid_map::LineIterator iterator_end(map, TR, TL);
        for (; !iterator_start.isPastEnd()&&!iterator_end.isPastEnd(); ++iterator_start, ++iterator_end)
        {
            grid_map::Index start_index(*iterator_start);
            grid_map::Index end_index(*iterator_end);
            for (grid_map::LineIterator iterator_l(map, start_index, end_index); !iterator_l.isPastEnd(); ++iterator_l)
            {
                const grid_map::Index index_l(*iterator_l);
                grid_map::Position3 p3;
                if (map.getPosition3("elevation", index_l, p3))
                {
                    if (!std::isnan(p3.z()))
                    {
                        return_points.emplace_back(p3);
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

// 这种方式的虽然较为真实的表达出支撑面的情况，但是有在我们使用可行方向对应可行域聚类情况会有一个矛盾点
// 在确定非全向的可行域时，将只膨胀脚后跟长度的图-膨胀斜对脚长度的地图相减的区域来检测确定，然而，这种确定方式在上台阶时，假设脚后跟刚好位于某平面上时，他左右转动的角度也是认为可通行的。为了解决这种情况，我们严格要求支撑区域内全部cell都必须位于同一平面上
bool PathPlanning::isFeasibleNew(grid_map::Index & index, double angle)
{
    Eigen::AngleAxisd ax(angle, Eigen::Vector3d::UnitZ());
    grid_map::Position p2;
    if (map.getPosition(index, p2))
    {
        grid_map::Position3 p3 = Eigen::Vector3d::Zero();
        p3.head(2) = p2;
        Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(support_area.Up - 0.08, 0, 0) + p3;
        Eigen::Vector3d mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- (support_area.Button - 0.03), 0, 0) + p3;
        Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, support_area.Left -0.04, 0) + mid_top;
        Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - (support_area.Right - 0.04), 0) + mid_top;
        Eigen::Vector3d down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, support_area.Left - 0.04, 0) + mid_down;
        Eigen::Vector3d down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - (support_area.Right - 0.04), 0) + mid_down;
        grid_map::Position top_left_l(top_left.x(), top_left.y());
        grid_map::Position top_right_l(top_right.x(), top_right.y());
        grid_map::Position down_left_l(down_left.x(), down_left.y());
        grid_map::Position down_right_l(down_right.x(), down_right.y());

        vector<Eigen::Vector3d> points;
        if (getPoints(top_left_l, top_right_l, down_left_l, down_right_l, points))
        {
            if (points.size() < 4)
            {
#ifdef DEBUG
                LOG(INFO)<<"points too less";
#endif
                return false;
            }
            Eigen::Vector3d sum = Eigen::Vector3d::Zero();
            for (auto & point : points)
            {
                sum += point;
            }
            Eigen::Vector3d center = sum / points.size();
            
            Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
            for (auto & point : points)
            {
                M += (point - center)* (point - center).transpose();
            }
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(M);
            if (solver.info() == Eigen::Success)
            {
                Eigen::Vector3d eigenvalues = solver.eigenvalues();
                Eigen::Matrix3d eigenvectors = solver.eigenvectors();
                if (eigenvalues(0)/eigenvalues.sum() < 0.1)
                {
                    Eigen::Vector3d normal = eigenvectors.col(0);
                    if (eigenvectors.col(0).z() < 0)
                    {
                        normal = - eigenvectors.col(0);
                    }
                    mid_top = ax.toRotationMatrix() * Eigen::Vector3d(support_area.Up, 0, 0) + p3;
                    mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- (support_area.Button), 0, 0) + p3;
                    top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, support_area.Left, 0) + mid_top;
                    top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - (support_area.Right), 0) + mid_top;
                    down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, support_area.Left, 0) + mid_down;
                    down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - (support_area.Right), 0) + mid_down;
                    top_left_l = grid_map::Position(top_left.x(), top_left.y());
                    top_right_l = grid_map::Position(top_right.x(), top_right.y());
                    down_left_l = grid_map::Position(down_left.x(), down_left.y());
                    down_right_l = grid_map::Position(down_right.x(), down_right.y());
                    vector<Eigen::Vector3d> all_points;
                    if (getPoints(top_left_l, top_right_l, down_left_l, down_right_l, all_points))
                    {
                        for (auto & point : all_points)
                        {
                            if ((point - center).dot(normal) > 0.01 || (point - center).dot(normal) < -0.01)
                            {
#ifdef DEBUG
                                // LOG(INFO)<<"in foot";
#endif
                                return false;
                            }
                        }
                        return true;
                    }
                    else
                    {
#ifdef DEBUG
                            // LOG(INFO)<<"out of map";
#endif
                        return false;
                    }
                }
                else
                {
#ifdef DEBUG
                        // LOG(INFO)<<"can not get normal";
#endif
                    return false;
                }
                
            }
            else
            {
#ifdef DEBUG
                // LOG(INFO)<<"can not get eigen value";
#endif
                return false;
            }
        }
        else
        {
#ifdef DEBUG
            // LOG(INFO)<<"out of map";
#endif
            return false;
        }
    }
    else
    {
#ifdef DEBUG
        // LOG(INFO)<<"can not get point";
#endif
        return false;
    }
}

// 这个过程比较耗时
void PathPlanning::checkFeasibleDirect()
{
    for (int i = 0; i < pd.planes.size(); i++)
    {
        cv::Mat image = pd.planes.at(i);
        // 定义形态学操作的核
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        // 应用形态学操作：先膨胀再腐蚀（开操作），去除毛刺
        cv::Mat smoothedImage;
        cv::morphologyEx(image, smoothedImage, cv::MORPH_CLOSE, element);

#ifdef DEBUG
        // // 找出两个图像中不同的像素
        // cv::Mat differenceImage;
        // cv::bitwise_xor(image, smoothedImage, differenceImage);
        // // 将差异转换为三通道图像以便于可视化
        // cv::Mat colorDifference;
        // cv::cvtColor(differenceImage, colorDifference, cv::COLOR_GRAY2BGR);
        // // 在差异图像上标记不同的像素，使用红色突出显示
        // for (int i = 0; i < differenceImage.rows; i++) {
        //     for (int j = 0; j < differenceImage.cols; j++) {
        //         if (differenceImage.at<uchar>(i, j) != 0) {
        //             cout<<"draw red"<<endl;
        //             colorDifference.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);  // 红色
        //         }
        //     }
        // }
        // cv::imshow("diff_" + std::to_string(i), colorDifference);
        // cv::waitKey(0);
#endif
        
        // 找到0-脚宽/2，这些区域的可通行型为NAN
        cv::Mat Nan_eroded;
        cv::Mat half_width_Element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size((support_area.Button- 0.05)/map.getResolution() * 2 + 1, (support_area.Button - 0.05)/map.getResolution() * 2 + 1));
        cv::erode(smoothedImage, Nan_eroded, half_width_Element);

#ifdef DEBUG
        // 腐蚀erode会导致白色区域变小，黑色区域变大
        // LOG(INFO)<<cv::countNonZero(smoothedImage)<<" "<<cv::countNonZero(Nan_eroded);
        // cv::imshow("Nan_eroded"+ std::to_string(i), Nan_eroded);
        // cv::waitKey(0);
#endif
        cv::Mat NanFeasible = smoothedImage - Nan_eroded;
        // 将所有不可通行的区域合并，只要有一个图像处理是不可通行的，那么就认为该点是不可通行的
        cv::bitwise_or(Nan_feasible, NanFeasible, Nan_feasible);
#ifdef DEBUG
        // cv::imshow("NanFeasible" + std::to_string(i), NanFeasible);
        // cv::waitKey(0);
#endif
        // 找到>前脚掌长度的区域，这些区域的可通行性为full，全向通行
        cv::Mat Full_eroded;
        double full_length = std::sqrt(support_area.Left * support_area.Left + support_area.Up * support_area.Up);
        cv::Mat fore_foot_Element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(full_length/map.getResolution() * 2 + 1, full_length/map.getResolution() * 2 + 1));
        cv::erode(smoothedImage, Full_eroded, fore_foot_Element);
#ifdef DEBUG
        // cv::imshow("Full_eroded"+ std::to_string(i), Full_eroded);
        // cv::waitKey(0);
#endif
        // cv::Mat FullFeasible = smoothedImage - Full_eroded;

        // 只要一个地方是不全向，那必然不是全向
        cv::bitwise_or(Full_feasible, Full_eroded, Full_feasible);
// #ifdef DEBUG
//         cv::imshow("FullFeasible"+ std::to_string(i), FullFeasible);
//         cv::waitKey(0);
// #endif
        // 对脚宽/2～前脚掌长度，进行
        cv::Mat CheckImage = Nan_eroded - Full_eroded;

        // 减去障碍区域，即将CheckImage中与obstacle_layer重叠的区域设置为0
        CheckImage.setTo(0, obstacle_layer);
#ifdef DEBUG
        // cv::imshow("CheckImage"+ std::to_string(i), CheckImage);
        // cv::waitKey(0);
#endif
        std::vector<cv::Point> white_points;
        cv::findNonZero(CheckImage, white_points);

        // 这个地方建议加一个openmp
        
        // for (int j = 0; j < white_points.size(); j++)
        // {
        //     auto & cv_p = white_points[i];
        //     LOG(INFO) << cv_p.x << " " << cv_p.y;
        //     // 定义局部变量以避免线程之间的竞争
        //     bool Full_feasible_flag = true;
        //     bool Nan_feasible_flag = false;
        //      for (int k = 0; k < 72; k++)
        //     {
        //         double angle = (k * 5) / 57.3;
        //         grid_map::Index index(cv_p.y, cv_p.x);
        //         if (isFeasibleNew(index, angle))
        //         {
        //             checks_Mat.at(j).at<uchar>(cv_p) = 255;
        //             Full_feasible_flag = Full_feasible_flag && true;
        //             Nan_feasible_flag = Nan_feasible_flag || true;
        //         }
        //         else
        //         {
        //             checks_Mat.at(j).at<uchar>(cv_p) = 0;
        //             Full_feasible_flag = Full_feasible_flag && false;
        //             Nan_feasible_flag = Nan_feasible_flag || false;
        //         }
        //     }
        //     // 对 Full_feasible 和 Nan_feasible 进行并行安全的访问
        //     if (Full_feasible_flag)
        //     {
        //         #pragma omp critical
        //         {
        //             if (Full_feasible.at<uchar>(cv_p) == 0)
        //             {
        //                 Full_feasible.at<uchar>(cv_p) = 255;
        //             }
        //         }
        //     }
        //     if (!Nan_feasible_flag)
        //     {
        //         #pragma omp critical
        //         {
        //             if (Nan_feasible.at<uchar>(cv_p) == 0)
        //             {
        //                 Nan_feasible.at<uchar>(cv_p) = 255;
        //             }
        //         }
        //     }
        // }
        
        // #pragma omp parallel for reduction(+:Full_feasible,Nan_feasible)
        // for (auto & cv_p : white_points)
        // {
        //     LOG(INFO)<<cv_p.x<<" "<<cv_p.y;
        //     // 将其中完全不可通行的点和全向可通行的点挑选出来
        //     bool Full_feasible_flag = true;
        //     bool Nan_feasible_flag = false;
        //     for (int j = 0; j < 72; j++)
        //     {
        //         double angle = (j * 5)/57.3;
        //         grid_map::Index index(cv_p.y, cv_p.x);
        //         if (isFeasibleNew(index, angle))
        //         {
        //             checks_Mat.at(j).at<uchar>(cv_p) = 255;
        //             Full_feasible_flag = Full_feasible_flag && true;
        //             Nan_feasible_flag = Nan_feasible_flag || true;
        //         }
        //         else
        //         {
        //             checks_Mat.at(j).at<uchar>(cv_p) = 0;
        //             Full_feasible_flag = Full_feasible_flag && false;
        //             Nan_feasible_flag = Nan_feasible_flag || false;
        //         }
        //     }
        //     if (Full_feasible_flag)
        //     {
        //         if (Full_feasible.at<uchar>(cv_p) == 0)
        //         {
        //             Full_feasible.at<uchar>(cv_p) = 255;
        //         }   
        //     }
        //     if (!Nan_feasible_flag)
        //     {
        //         if (Nan_feasible.at<uchar>(cv_p) == 0)
        //         {
        //             Nan_feasible.at<uchar>(cv_p) = 255;
        //         }  
        //     }
        // }
        auto start = std::chrono::high_resolution_clock::now();
        // cv::Mat tmp_Full_feasible = cv::Mat::zeros(Full_feasible.size(), CV_8UC1);
        // cv::Mat tmp_Nan_feasible = cv::Mat::zeros(Nan_feasible.size(), CV_8UC1);
        // #pragma omp parallel
        // {
        //     cv::Mat local_Full_feasible = tmp_Full_feasible.clone();
        //     cv::Mat local_Nan_feasible = tmp_Nan_feasible.clone();
        //     #pragma omp for nowait
        //     for (size_t i = 0; i < white_points.size(); ++i)
        //     {
        //         auto & cv_p = white_points[i];
        //         // LOG(INFO) << cv_p.x << " " << cv_p.y;
        //         bool Full_feasible_flag = true;
        //         bool Nan_feasible_flag = false;
        //         for (int j = 0; j < 72; j++)
        //         {
        //             double angle = (j * 5) / 57.3;
        //             grid_map::Index index(cv_p.y, cv_p.x);
        //             if (isFeasibleNew(index, angle))
        //             {
        //                 checks_Mat.at(j).at<uchar>(cv_p) = 255;
        //                 Full_feasible_flag = Full_feasible_flag && true;
        //                 Nan_feasible_flag = Nan_feasible_flag || true;
        //             }
        //             else
        //             {
        //                 checks_Mat.at(j).at<uchar>(cv_p) = 0;
        //                 Full_feasible_flag = Full_feasible_flag && false;
        //                 Nan_feasible_flag = Nan_feasible_flag || false;
        //             }
        //         }
        //         if (Full_feasible_flag)
        //         {
        //             if (local_Full_feasible.at<uchar>(cv_p) == 0)
        //             {
        //                 local_Full_feasible.at<uchar>(cv_p) = 255;
        //             }
        //         }
        //         if (!Nan_feasible_flag)
        //         {
        //             if (local_Nan_feasible.at<uchar>(cv_p) == 0)
        //             {
        //                 local_Nan_feasible.at<uchar>(cv_p) = 255;
        //             }
        //         }
        //     }
        //     #pragma omp critical
        //     {
        //         tmp_Full_feasible |= local_Full_feasible;
        //         tmp_Nan_feasible |= local_Nan_feasible;
        //     }
        // } 
        // Nan_feasible |= tmp_Nan_feasible;
        // Full_feasible |= tmp_Full_feasible;
        // 这是不使用openmp加速的代码
        for (auto & cv_p : white_points)
        {
            // LOG(INFO)<<cv_p.x<<" "<<cv_p.y;
            // 将其中完全不可通行的点和全向可通行的点挑选出来
            bool Full_feasible_flag = true;
            bool Nan_feasible_flag = false;
            for (int j = 0; j < 72; j++)
            {
                double angle = (j * 5)/57.3;
                grid_map::Index index(cv_p.y, cv_p.x);
                if (isFeasibleNew(index, angle))
                {
                    checks_Mat.at(j).at<uchar>(cv_p) = 255;
                    Full_feasible_flag = Full_feasible_flag && true;
                    Nan_feasible_flag = Nan_feasible_flag || true;
                }
                else
                {
                    checks_Mat.at(j).at<uchar>(cv_p) = 0;
                    Full_feasible_flag = Full_feasible_flag && false;
                    Nan_feasible_flag = Nan_feasible_flag || false;
                }
            }
            if (Full_feasible_flag)
            {
                if (Full_feasible.at<uchar>(cv_p) == 0)
                {
                    Full_feasible.at<uchar>(cv_p) = 255;
                }   
            }
            if (!Nan_feasible_flag)
            {
                if (Nan_feasible.at<uchar>(cv_p) == 0)
                {
                    Nan_feasible.at<uchar>(cv_p) = 255;
                }  
            }
            if (!Full_feasible_flag && Nan_feasible_flag)
            {
                check_Mat.at<uchar>(cv_p) = 255;
            }
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "Elapsed time: " << duration << " milliseconds" << std::endl;
        // 能不能将这里的处理与后续的调换一下顺序，减少计算两
    }

    // for (int i = 0; i < checks_Mat.size(); i++)
    // {
    //     cv::imshow("check_Mat" + std::to_string(i*5), checks_Mat.at(i));
    //     cv::waitKey(0);
    // }
    
    // 障碍周围的点都为不可通行, 这个怎么还有问题？会导致将可通行平面去除的过多，从而影响可通过性的判断
    // cv::Mat obstacle_dilated;
    // double full_length = std::sqrt(support_area.Left * support_area.Left + support_area.Up * support_area.Up);
    // cv::Mat fore_foot_Element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(full_length/map.getResolution() * 2 + 1, full_length/map.getResolution() * 2 + 1));
    // cv::dilate(obstacle_layer, obstacle_dilated, fore_foot_Element);
    // cv::imshow("obstacle_layer", obstacle_layer);
    // cv::waitKey(0);
    // cv::imshow("obstacle_dilated", obstacle_dilated);
    // cv::waitKey(0);
    // // 将这些不可行的区域筛选出来赋给check_Mat赋0
    // // cv::imshow("check_Mat onstacle b", check_Mat);
    // // cv::waitKey(0);
    check_Mat.setTo(0, obstacle_layer);
    // cv::imshow("check_Mat onstacle a", check_Mat);
    // cv::waitKey(0);

    

    // 图像的边界区域，将图像边界默认为不可通行，
    int rows = Nan_feasible.rows;
    int cols = Nan_feasible.cols;
    LOG(INFO)<<rows<<" "<<cols;
    // cv::imshow("Nan_feasible", Nan_feasible);
    // cv::waitKey(0);
    // cv::imshow("Full_feasible", Full_feasible);
    // cv::waitKey(0);
    double radius = sqrt(support_area.Up * support_area.Up + support_area.Left * support_area.Left);
    for (int i = 0; i < radius/map.getResolution(); i++)
    {
        Nan_feasible.row(i).setTo(255);
        Nan_feasible.row(rows - i - 1).setTo(255);
        Nan_feasible.col(i).setTo(255);
        Nan_feasible.col(cols - i -1).setTo(255);

        Full_feasible.row(i).setTo(0);
        Full_feasible.row(rows - i - 1).setTo(0);
        Full_feasible.col(i).setTo(0);
        Full_feasible.col(cols - i - 1).setTo(0);
    }
    Full_feasible.setTo(0, obstacle_layer);
    Nan_feasible.setTo(255, obstacle_layer);
    cv::imshow("check_Mat Nan a", check_Mat);
    cv::waitKey(0);
    check_Mat.setTo(0, Nan_feasible);
    check_Mat.setTo(0, Full_feasible);
    cv::imshow("check_Mat Nan b", check_Mat);
    cv::waitKey(0);
}

bool PathPlanning::clustering()
{
    cv::imshow("check_Mat", check_Mat);
    cv::waitKey(0);

// 由于这部分check_Mat还有一些后续处理，所以，feasible_mat与check_Mat不一样
    // cv::Mat feasible_mat = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    // for (int i = 0; i < feasible_mat.rows; i++)
    // {
    //     for (int j = 0; j < feasible_mat.cols; j++)
    //     {
    //         bool full_flag = true;
    //         bool Nan_flag = false;
    //         for (int k = 0; k < 72; k++)
    //         {
    //             if (checks_Mat.at(k).at<uchar>(i, j) == 255)
    //             {
    //                 full_flag = full_flag && true;
    //                 Nan_flag = Nan_flag || true; // 为true表示此像素点不是nan
    //             }
    //             else
    //             {
    //                 full_flag = full_flag && false;
    //                 Nan_flag = Nan_flag || false;
    //             }
    //         }
    //         // full_flag为false时，表示此像素点不为全向通行
    //         // Nan_flag 为true时，表示此像素点某些情况下是可通行的
    //         if (!full_flag && Nan_flag)
    //         {
    //             feasible_mat.at<uchar>(i, j) = 255;
    //         }
            
    //     }
    // }
    // cv::imshow("feasible_mat", feasible_mat);
    // cv::waitKey(0);
    

    // cv::Mat region1, region2;

    // // 第一个图像为 255，第二个图像为 0 的区域
    // cv::Mat condition1 = (check_Mat == 255) & (feasible_mat == 0);  // 生成满足条件的掩码
    // condition1.convertTo(region1, CV_8U, 255);  // 将布尔掩码转换为二值图像，白色区域为满足条件的部分

    // // 第一个图像为 0，第二个图像为 255 的区域
    // cv::Mat condition2 = (check_Mat == 0) & (feasible_mat == 255);  // 生成满足条件的掩码
    // condition2.convertTo(region2, CV_8U, 255);  // 转换为二值图像，白色区域为满足条件的部分

    // // 显示结果
    // cv::imshow("Region1 (image1=255, image2=0)", region1);
    // cv::imshow("Region2 (image1=0, image2=255)", region2);
    // cv::waitKey(0);
    // return true;

    // 筛选出更严格的像素点，不能是直接里面总的可通行角度大于strict_section，应该是实际可通行的连续的角度不超过strict_section
    strictChecksMat = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    for (int i = 0; i < strictChecksMat.cols; i++)
    {
        for (int j = 0; j < strictChecksMat.rows; j++)
        {
            int num = 0;
            bool flag = false;
            for(int k = 0; k < 72; k++)
            {
                if (checks_Mat.at(k).at<uchar>(j, i) == 255)
                {
                    flag = true;
                    num ++;
                }
                else
                {
                    num = 0; // 会受噪声影响大，先这样吧
                }
                if (num > strict_section)
                {
                    break;
                }
            }
            // 不能全部为不可通行
            if (num <= strict_section && flag)
            {
                if (checks_Mat.at(0).at<uchar>(j, i) == 255 && checks_Mat.at(71).at<uchar>(j, i) == 255)
                {
                    int num_in = 2;
                    bool flag1 = true;
                    bool flag2 = true;
                    int index1 = 1;
                    int index2 = 70;
                    while (flag1 || flag2)
                    {
                        if (flag1)
                        {
                            if (checks_Mat.at(index1).at<uchar>(j, i) == 255)
                            {
                                num_in++;
                                index1++;
                            }
                            else
                            {
                                flag1 = false;
                            }
                        }       
                        if (flag2)
                        {
                            if (checks_Mat.at(index2).at<uchar>(j, i) == 255)
                            {
                                num_in++;
                                index2--;
                            }
                            else
                            {
                                flag2 = false;
                            }
                        }
                    }
                    if (num_in <= strict_section)
                    {
                        strictChecksMat.at<uchar>(j, i) = 255;
                    }
                }
                else
                {
                    strictChecksMat.at<uchar>(j, i) = 255;
                }
            }
        }
    }
    strictChecksMat.setTo(0, check_Mat == 0);
    cv::imshow("strictChecksMat", strictChecksMat);
    cv::waitKey(0);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(strictChecksMat, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    LOG(INFO)<<contours.size();
  
    // 对每一个区域内的每个可通行方向获取其
    vector<DirectRegion> strict_direct_regions;
    // 4. 遍历每个轮廓并绘制到结果图像中
    for (size_t i = 0; i < contours.size(); i++) 
    {
        // 3. 创建结果图像，初始化为全黑
        LOG(INFO)<<"I = "<<i;
        cv::Mat result = cv::Mat::zeros(check_Mat.size(), CV_8UC1);
        // 绘制当前轮廓
        cv::drawContours(result, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
        
        cv::bitwise_and(strictChecksMat, result, result);
        // 对区域极小的区域，就不在考虑，直接不再考虑，对于障碍附近的点也不再考虑
        if (cv::countNonZero(result) < 10)
        {
            continue;
        }
        
        cv::imshow("result" + std::to_string(i), result);
        cv::waitKey(0);

        // 每个区域内的像素，根据可通行角度进行聚类
        std::vector<cv::Point> white_points;
        cv::findNonZero(result, white_points);
        // std::unordered_map<int, vector<cv::Point> > histogram;
        vector< vector<cv::Point> > histogram(72);

        for (auto cv_point : white_points)
        {
            for (int j = 0; j < checks_Mat.size(); j++)
            {
                if (checks_Mat.at(j).at<uchar>(cv_point) == 255)
                {
                    histogram[j].emplace_back(cv_point);
                }
            }
        }

        // // 找到最小的，根据投票票数和最小值，取出小于某个阈值的投票，再选取其中间断的，较长的
        // // for (auto & bin : histogram)
        // // {
        // //     LOG(INFO)<<bin.first<<" "<<bin.second.size();
        // // }
        // vector<int> x(72);
        // vector<int> y(72);
        // for (int j = 0; j < 72; j++)
        // {
        //     x.at(j) = j * 5;
        //     if (!histogram.at(j).empty())
        //     {
        //         y.at(j) = histogram[j].size();
        //     }
        //     else
        //     {
        //         y.at(j) = 0;
        //     }
        // }
        // matplotlibcpp::title("Sample Histogram");
        // matplotlibcpp::xlabel("Category");
        // matplotlibcpp::ylabel("Values");

        // // 绘制图表
        // matplotlibcpp::plot(x, y);
        // // 显示图表
        // matplotlibcpp::show();

        // // 一个区域可能会有多个直方图
        vector< std::unordered_map<int, vector<cv::Point> > > histograms;

        // 只不过多了一个索引，并没有顺序
        std::unordered_map<int, vector<cv::Point> > single_histogram_tmp;

        for (int j = 0; j < histogram.size(); j++)
        {
            if (histogram.at(j).size() > 4)
            {
                single_histogram_tmp[j] = histogram.at(j);
            }
            else
            {
                if (!single_histogram_tmp.empty())
                {
                    histograms.emplace_back(single_histogram_tmp);
                }
                single_histogram_tmp.clear();
            }
        }
        
        // 检查最后一个 histogram 是否被保存
        if (!single_histogram_tmp.empty())
        {
            histograms.emplace_back(single_histogram_tmp);
        }

        cout<<"result: "<<endl;
        cout<<histograms.size()<<endl;
        for(auto & his : histograms)
        {
            for(auto & bin : his)
            {
                std::cout<<"[ "<<bin.first<<", "<<bin.second.size()<<" ]"<<" ";
            }
            // std::cout<<std::endl;
        }
        cout<<"next........."<<endl; 

        // 一个区域内会有多个直方图，但是直方图肯定是顺序排列的，所以只需要检查第一个和最后一个是否分别包含0和71，即可判断该区域内的首尾两个直方图是否需要合并
        // 检查首尾需不需要合并 
        if (histograms.front().find(0) != histograms.front().end() && histograms.back().find(71) != histograms.back().end())
        {
            for (auto bin : histograms.back())
            {
                histograms.front()[bin.first] = bin.second;
            }
            histograms.erase(histograms.end() - 1);
        }
        for(auto & his : histograms)
        {
            for(auto & bin : his)
            {
                std::cout<<"[ "<<bin.first<<", "<<bin.second.size()<<" ]"<<" ";
            }
            std::cout<<std::endl;
        }
        
        // 如何考虑首尾的情况
        for (auto & his : histograms)
        {
            // LOG(INFO)<<"IN...";
            DirectRegion strict_direct_region;
            cv::Mat region = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
            // 如果即包含0也包含71，且size小于72，那么是首尾相连的情况
            if (his.find(0) != his.end() && his.find(71) != his.end() && his.size() < 72)
            {
                // LOG(INFO)<<"IN---...";
                int max = 0;
                int min = 71;
                int range = 0;
                bool min_flag = true;
                bool max_flag = true;
                while (min_flag || max_flag)
                {
                    if (min_flag)
                    {
                        if (his.find(min) != his.end())
                        {
                            for (auto & cv_point : his[min])
                            {
                                region.at<uchar>(cv_point) = 255;
                            }
                            min--;
                            range++;
                        }
                        else
                        {
                            min_flag = false;
                        }
                    }
                    if (max_flag)
                    {
                        if (his.find(max) != his.end())
                        {
                            for (auto & cv_point : his[max])
                            {
                                region.at<uchar>(cv_point) = 255;
                            }
                            max++;
                            range++;
                        }
                        else
                        {
                            max_flag = false;
                        }
                    }
                }
                LOG(INFO)<<min<<" "<<max;
                strict_direct_region.min = min;
                strict_direct_region.range = range;
                strict_direct_region.region = region;
            }
            else
            {
                // LOG(INFO)<<"IN..DD.";
                if (!his.empty())
                {
                    int max = -1;
                    int min = 72;        
                    for (auto & bin : his)
                    {
                        if (bin.first > max)
                        {
                            max = bin.first;
                        }
                        if (bin.first < min)
                        {
                            min = bin.first;
                        }
                        for (auto & cv_point : bin.second)
                        {
                            region.at<uchar>(cv_point) = 255;
                        }
                    }

                    LOG(INFO)<<min<<" "<<max;
                    strict_direct_region.min = min;
                    strict_direct_region.range = his.size();
                    LOG(INFO)<<"range: "<<his.size();
                    strict_direct_region.region = region;
                    drs.emplace_back(strict_direct_region);
                }
            }   
        }
    }
    for (auto & dr : drs)
    {
        LOG(INFO)<<dr.min<<" "<<dr.range<<endl;
        cv::imshow("region", dr.region);
        cv::waitKey(0);
    }
    // return true;
    // 获取一般的区域，并根据严格通行区域是否重叠，并根据每个重叠区域内的非重叠栅格来进一步核查region
    std::vector<std::vector<cv::Point>> contours_feasible;
    std::vector<cv::Vec4i> hierarchy_feasible;
    // 只提取外轮廓
    // LOG(INFO)<<"check_Mat--------------";          
    // cv::imshow("check_Mat***           ", check_Mat);
    // cv::waitKey(0);
    // cv::findContours(strictChecksMat, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    // cv::findContours(check_Mat, contours_feasible, hierarchy_feasible, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
    cv::findContours(check_Mat, contours_feasible, hierarchy_feasible, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    for (size_t i = 0; i < contours_feasible.size(); i++) 
    {
        // 如果有父轮廓，则不需要管
        // if (hierarchy[i][3] != -1)
        // {
        //     continue;
        // }
        
        // 3. 创建结果图像，初始化为全黑
        cv::Mat result = cv::Mat::zeros(check_Mat.size(), CV_8UC1);
        // 绘制当前轮廓
        cv::drawContours(result, contours_feasible, static_cast<int>(i), cv::Scalar(255), cv::FILLED);

        cv::bitwise_and(check_Mat, result, result);
        // cv::imshow("result", result);
        // cv::waitKey(0);
        // 如果有重叠
        for (int j = 0; j < drs.size(); j++)
        {
            cv::Mat intersection;
            cv::bitwise_and(result, drs.at(j).region, intersection);
            // 判断交集区域内是否有非零像素
            if (cv::countNonZero(intersection) > 8)
            {
                cv::Mat nonOverlap = result - drs.at(j).region;
                std::vector<cv::Point> white_points;
                cv::findNonZero(nonOverlap, white_points);
                for (auto & cv_point : white_points)
                {
                    bool flag = false;
                    for (int k = 0; k < drs.at(j).range; k++)
                    {
                        int index;
                        if (drs.at(j).min + k >= 72)
                        {
                            index = drs.at(j).min + k -72;
                        }
                        else
                        {
                            index = drs.at(j).min + k;
                        }
                        if (checks_Mat.at(index).at<uchar>(cv_point) == 0)
                        {
                            flag = false;
                            break;
                        } 
                        else
                        {
                            flag = true;
                        }
                    }
                    if (flag)
                    {
                        drs.at(j).region.at<uchar>(cv_point) = 255;
                    }
                    
                }   
            }
        } 
    }

    for (auto & dr : drs)
    {
        LOG(INFO)<<dr.min<<" "<<dr.range;
        cv::imshow("region----", dr.region);
        cv::waitKey(0);
    }

    // 将里面的每个region先腐蚀再膨胀，将里面的一个区域转成多个区域
    // 如果区域本来就比较小，进行小半径膨胀之后变成没有洞的，就直接把这个区域填充
    auto tmp_drs = drs;
    drs.clear();
    for (auto & dr : tmp_drs)
    {
        cv::Mat temp_image = dr.region;

        // 如果这个区域经过0.15m的膨胀以后变成无动的，那么将此区域填充后的所有区域均为此方向对应的区域
        double inflation_radius = 0.15;
        int inflation_pixels = inflation_radius/map.getResolution();
        cv::Mat inflation_element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(inflation_pixels * 2 + 1, inflation_pixels * 2 + 1));
        cv::dilate(temp_image, temp_image, inflation_element);
        std::vector<std::vector<cv::Point>> temp_contours;
        std::vector<cv::Vec4i> temp_hierarchy;
        cv::findContours(temp_image, temp_contours, temp_hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
        bool small_hole_flag = true;
        for (int i = 0; i < temp_contours.size(); i++)
        {
            if (temp_hierarchy[i][3] != -1)
            {
                small_hole_flag = false;
                break;
            }
        }
        // 不是small hole的都不用要，默认成full
        if (small_hole_flag)// 直接填充后作为区域
        {
            cv::Mat region = cv::Mat::zeros(check_Mat.size(), CV_8UC1);
            for (int i = 0; i < temp_contours.size(); i++)
            {
                cv::drawContours(region, temp_contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
            }
            DirectRegion temp_dr;
            temp_dr.min = dr.min;
            temp_dr.range = dr.range;
            temp_dr.region = region;
            drs.emplace_back(temp_dr);
        }
        else
        {
            cv::Mat hole_image = cv::Mat::zeros(check_Mat.size(), CV_8UC1);
            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::morphologyEx(hole_image, hole_image, cv::MORPH_OPEN, element);
            std::vector<std::vector<cv::Point>> hole_contours;
            std::vector<cv::Vec4i> hole_hierarchy;
            cv::findContours(hole_image, hole_contours, hole_hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
            for (size_t i = 0; i < hole_contours.size(); i++)
            {
                cv::Mat result = cv::Mat::zeros(check_Mat.size(), CV_8UC1);
                cv::drawContours(result, hole_contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
                cv::bitwise_and(hole_image, result, result);
                DirectRegion temp_dr;
                temp_dr.min = dr.min;
                temp_dr.range = dr.range;
                temp_dr.region = result;
                cv::imshow("region--**--", temp_dr.region);
                cv::waitKey(0);

                drs.emplace_back(temp_dr);
            }
        }
    }
    return true;
}

// 需要将可通行区域合并，一些离得比较近且方向大致相同的区域合并，例如台阶的多级都可以合并，合并区域内不能包含障碍区域。
// 合并后的区域可以使用膨胀后两区域的相交部分内没有障碍区域
// 障碍区域的定义：两个平面在某处的高度差达到某个阈值，所以，之前定义的障碍点信息不充分
void PathPlanning::clusterFeasibleRegions()
{
    Graph g;
    for(int i = 0; i < drs.size() - 1; i++)
    {
        for (int j = 0; j < drs.size(); j++)
        {
            // 如果两个区域膨胀后，有没有相交的区域
            cv::Mat image1 = drs.at(i).region;
            cv::Mat image2 = drs.at(j).region;
            int g_nStructBLlementSize = 0.2/map.getResolution();
            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * g_nStructBLlementSize + 1, 2 * g_nStructBLlementSize + 1 ));
            cv::dilate(image1, image1, element);
            cv::dilate(image2, image2, element);
            cv::Mat intersection;
            cv::bitwise_and(image1, image2, intersection);
            // 判断交集区域内是否有非零像素
            if (cv::countNonZero(intersection) > 0)
            {
                double angle1 = drs.at(i).min + ((double)(drs.at(i).range) - 1.0)/2.0;
                double angle2 = drs.at(j).min + ((double)(drs.at(j).range) - 1.0)/2.0;
                LOG(INFO)<<"angle1: "<<angle1<<", angle2: "<<angle2;
                if (abs(angle1 - angle2)*5 < 15)//如果小于15度
                {
                    g.addEdge(i, j);
                    g.addEdge(j, i);
                }
            }
        }
    }

    // 找到所有聚类
    vector<vector<int>> clusters = g.findClusters();
    
    for (auto & cluster : clusters)
    {
        for (auto & index : cluster)
        {
            cout<<index<<" ";
        }
        cout<<endl;
    }
    

    // 根据聚类构造
    // CompareDR compareDR(map);
    // for (auto & cluster : clusters)
    // {
    //     MergedDirectRegion msr;
    //     cv::Mat merged_region = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
    //     for (auto & ds : cluster)
    //     {
    //         merged_region.setTo(255, drs.at(ds).region);
    //         msr.merged_direct_regions.emplace_back(drs.at(ds));
    //     }
    //     msr.merged_region = merged_region;
    //     std::sort(msr.merged_direct_regions.begin(), msr.merged_direct_regions.end(), compareDR);
    // }
    
}

vector<int> PathPlanning::getPlanePath(int start_plane, int goal_plane)
{
    struct Node
    {
        int index, pre_index;
    }
    // 也不是每个平面只能一次

}

void PathPlanning::testisFeasible()
{
    grid_map::Index index(200, 92);
    double angle = 20/57.3;
    if (isFeasibleNew(index, angle))
    // if (isFeasible(index, angle))
    {
        LOG(INFO)<<"is feasible";
    }
    else
    {
        LOG(INFO)<<"NOT FEASIBLE";
    }
}



PathPlanning::~PathPlanning()
{
}