#include <path_planning/planning.h>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <Eigen/Dense>
#include <omp.h>
#include <chrono>
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
                }
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
                            if ((point - center).dot(normal) > 0.01)
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
        cv::Mat half_width_Element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(support_area.Button/map.getResolution() * 2 + 1, support_area.Button/map.getResolution() * 2 + 1));
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
        }
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        std::cout << "Elapsed time: " << duration << " milliseconds" << std::endl;
    }
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