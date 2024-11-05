#include <iostream>
#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/SubmapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <peac/PEAC_plane_detection.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <limits>
#include <glog/logging.h>

#define DEBUG
ros::Publisher pub;

// std::vector<std::pair<int, int>> findTrueBlocks(const std::vector<bool>& data) 
// {
//     int n = data.size();
//     std::vector<std::pair<int, int>> trueBlocks; // 存储连续true块的起点和长度
//     // Step 1: 扩展数组，处理首尾相接问题
//     std::vector<bool> extendedData(data.begin(), data.end());
//     extendedData.insert(extendedData.end(), data.begin(), data.end());
//     // Step 2: 扫描数组找出所有连续的true块
//     int start = -1;
//     for (int i = 0; i < extendedData.size(); ++i) {
//         if (extendedData[i]) {
//             if (start == -1) { // 这里在反复累积 为true的，
//                 start = i;
//             }
//         } else { // 当发现有一个不为true时
//             if (start != -1) { // 且不为-1， 表示已经累积了一些true
//                 int length = i - start; // 长度即为
//                 if (start < n) { // 表示没有进行到队列末尾
//                     if (start + length <= n) { // 如果没有超过队列
//                         trueBlocks.emplace_back(start, length);
//                     } else { // 如果超过
//                         trueBlocks.emplace_back(start, n - start);
//                         trueBlocks.emplace_back(0, length - (n - start));
//                     }
//                 }
//                 start = -1;
//             }
//         }
//     }
//     // 检查是否有末尾的true块没有被处理
//     if (start != -1) {
//         int length = extendedData.size() - start;
//         if (start < n) {
//             if (start + length <= n) {
//                 trueBlocks.emplace_back(start, length);
//             } else {
//                 trueBlocks.emplace_back(start, n - start);
//                 trueBlocks.emplace_back(0, length - (n - start));
//             }
//         }
//     }
//     return trueBlocks;
// }

// vector< vector< pair<int, int> > > parabolasSeg(vector<int> & histogram)
// {
//     int n = data.size();
//     vector< vector< pair<int, int> > > parabolas;
//     std::vector<bool> extendedData(data.begin(), data.end());
//     extendedData.insert(extendedData.end(), data.begin(), data.end());
//     while (extendedData.front() != 0)
//     {
//         extendedData.erase(extendedData.begin());
//     } 
//     for (int i = 0; i < n; ++i)
//     {
//         vector< pair<int, int> > parabola;
//         if (extendedData[i] != 0) 
//         {
//             parabola.emplace_back(std::make_pair(i, extendedData.at(i)));
//         } 
//         else 
//         {
//             if (parabola.size() >= 3) // 可通行区域太小不考虑
//             {
//                 parabolas.emplace_back(parabola);
//             }
//             parabola.clear();
//         }
//     }
//     if (parabola.size() >= 3) // 可通行区域太小不考虑
//     {
//         parabolas.emplace_back(parabola);
//     }
//     return parabolas;
// }

// 允许有一些nan的点，但是不允许有超过的点，
// bool isFeasible(grid_map::Index & index, double angle, grid_map::GridMap & map)
// {

//     struct FrequencyTracker 
//     {
//         std::unordered_map<int, int> frequencyMap;  // 哈希表，记录每个数的出现次数
//         int mostFrequentNumber;                     // 记录出现次数最多的数
//         int maxFrequency;                           // 记录最大出现次数

//         FrequencyTracker() : mostFrequentNumber(0), maxFrequency(0) {}

//         void addNumber(int number) {
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
//     };

//     Eigen::AngleAxisd ax(angle, Eigen::Vector3d::UnitZ());
//     grid_map::Position p2;
//     if (map.getPosition(index, p2))
//     {
//         grid_map::Position3 p3 = Eigen::Vector3d::Zero();
//         p3.head(2) = p2;
//         Eigen::Vector3d mid_top = ax.toRotationMatrix() * Eigen::Vector3d(0.15, 0, 0) + p3;
//         Eigen::Vector3d mid_down = ax.toRotationMatrix() * Eigen::Vector3d(- 0.1, 0, 0) + p3;
//         Eigen::Vector3d top_left = ax.toRotationMatrix() * Eigen::Vector3d(0, 0.165, 0) + mid_top;
//         Eigen::Vector3d top_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - 0.165, 0) + mid_top;
//         Eigen::Vector3d down_left = ax.toRotationMatrix() * Eigen::Vector3d(0, 0.65, 0) + mid_down;
//         Eigen::Vector3d down_right = ax.toRotationMatrix() * Eigen::Vector3d(0, - 0.65, 0) + mid_down;
//         grid_map::Position top_left_l(top_left.x(), top_left.y());
//         grid_map::Position top_right_l(top_right.x(), top_right.y());
//         grid_map::Position down_left_l(down_left.x(), down_left.y());
//         grid_map::Position down_right_l(down_right.x(), down_right.y());
//         if (label_localmap.isInside(top_left_l) && label_localmap.isInside(top_right_l) && label_localmap.isInside(down_left_l) && label_localmap.isInside(down_right_l))
//         {
//             FrequencyTracker ft;
//             int Nan_number = 0;
//             grid_map::LineIterator iterator_start(label_localmap, down_right_l, down_left_l);
//             grid_map::LineIterator iterator_end(label_localmap, top_right_l, top_left_l);
//             for (; !iterator_start.isPastEnd()&&!iterator_end.isPastEnd(); ++iterator_start, ++iterator_end)
//             {
//                 grid_map::Index start_index(*iterator_start);
//                 grid_map::Index end_index(*iterator_end);
//                 for (grid_map::LineIterator iterator_l(label_localmap, start_index, end_index); !iterator_l.isPastEnd(); ++iterator_l)
//                 {
//                     const grid_map::Index index_l(*iterator_l);
//                     int label_index = static_cast<int>(map["label"](y, x));
//                     if (!std::isnan(map["label"](y, x)))
//                     {
//                         ft.addNumber(label_index);
//                     }
//                     else
//                     {
//                         Nan_number++;
//                     }
//                 }
//             }
//             if (Nan_number < 20)
//             {
//                 if (ft.getMaxFrequency() > 100)
//                 {
//                     return true
//                 }
                
//             }
            
//             return false;
//         }
//         else
//         {
//             return false;
//         } 
//     }
//     else
//     {
//         return false;
//     }
// }


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
    pub = nh.advertise<grid_map_msgs::GridMap>("map", 1);
    // 创建一个10*10的地图

// 这个地图这样声明后面会在释放内存时出错，不影响使用。但是问题可能是在库本身的缺陷或者环境配置
    grid_map::GridMap map;
    grid_map::Length length(10, 10);
    grid_map::Position position(5, 0);
    map.setGeometry(length, 0.02, position);

    map.add("elevation", 0.0);

    grid_map::Position stair_s(8, 4.5);
    grid_map::Position stair_e(6, 3);
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

    grid_map::Position obstacle_s(5, 1);
    grid_map::Position obstacle_e(3, -1);
    grid_map::Index obstacle_s_index;
    if (map.getIndex(obstacle_s, obstacle_s_index))
    {
        grid_map::Size obstacle_size(2/map.getResolution(), 2/map.getResolution());
        for (grid_map::SubmapIterator iter(map, obstacle_s_index, obstacle_size); !iter.isPastEnd(); ++iter)
        {
            grid_map::Index iter_index(*iter);
            map["elevation"](iter_index.x(), iter_index.y()) = 1.0;
        }
    }
    
    // 转成有序点云
    pcl::PointCloud<pcl::PointXYZ> pc = gridMap2Pointcloud(map);

    plane_detection pd;
    pd.detect(pc);
    // // 检查法向量的方向，默认朝上。即z>0
    for (int i = 0; i < pd.planes_info.size(); i++)
    {
        if (pd.planes_info.at(i).normal.z() < 0)
        {
            pd.planes_info.at(i).normal = - pd.planes_info.at(i).normal;
        }
    }
    

    cv::imwrite("/home/lichao/catkin_pathplanning/src/path_planning/data/output_image.png", pd.result);
    for (int i = 0; i < pd.planes.size(); i++)
    {
        cv::imwrite("/home/lichao/catkin_pathplanning/src/path_planning/data/" + std::to_string(i) + ".png", pd.planes.at(i));
        LOG(INFO)<<pd.planes_info.at(i).normal.transpose()<<", "<<pd.planes_info.at(i).center.transpose();
    }
    
    // 根据平面检测结果给高程图赋值，使其成为plane-aware heightmap
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
    LOG(INFO)<<"label index";

    // double resolution = map.getResolution();
    // double inflation_radius = 0.5;
    // int inflation_pixel = inflation_radius/resolution;

    cv::Mat obstacle_layer = cv::Mat::zeros(pd.result.size(), CV_8UC1);
    // 求障碍点
    for (int i = 0; i < pd.planes.size(); i++)
    {
        cv::Mat image = pd.planes.at(i);
        // 存储轮廓的向量
        std::vector<std::vector<cv::Point>> contours;

        // 查找所有轮廓，并且保存轮廓上的所有点
        cv::findContours(image, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

        // 在原图上绘制轮廓（可选）
        // cv::Mat contourImage = cv::Mat::zeros(image.size(), CV_8UC3);
        // cv::Mat colorImage;
        // cv::cvtColor(image, colorImage, cv::COLOR_GRAY2BGR);

        // cv::drawContours(colorImage, contours, -1, cv::Scalar(0, 255, 0), 1);

        // 显示结果
        // cv::imshow("Contours", colorImage);
        // cv::waitKey(0);
        // 定义形态学操作的核
        // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));

        // // // 应用形态学操作：先膨胀再腐蚀（开操作），去除毛刺
        // // cv::Mat smoothedImage;
        // // cv::morphologyEx(image, smoothedImage, cv::MORPH_CLOSE, element);

        // // 进行形态学梯度操作获取边缘
        // cv::Mat edges;
        // cv::morphologyEx(image, edges, cv::MORPH_GRADIENT, element);

        // 找到边缘某个点的附近的点，根据这些点的高度差及所在平面法向量的夹角判断该点的可通行性，不可通行时为障碍点，同时根据高度差及夹角对障碍程度进行打分
        // 遍历边缘图像中的每个像素点
        // 这个核需要根据实际地图来确定，尤其是噪声
        int kernelSize = 4; // 定义核的大小，例如3x3
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
                // cv::circle(image, cv_point, 10, cv::Scalar(0, 255, 0), 2);
                // cv::imshow("point", image);
                // cv::waitKey(0);
                std::unordered_map<int, Eigen::Vector3d> normals;
                for (int ky = -kernelSize / 2; ky <= kernelSize / 2; ++ky) 
                {
                    for (int kx = -kernelSize / 2; kx <= kernelSize / 2; ++kx) 
                    {
                        int nx = cv_point.x + kx;
                        int ny = cv_point.y + ky;
                        // 检查邻域内的点是否在图像边界内
                        // if (nx >= 0 && ny >= 0 && nx < image.cols && ny < image.rows) 
                        if (nx >= 10 && ny >= 10 && nx < image.cols -10 && ny < image.rows - 10) 
                        {
                            // std::cout << "  Neighbor pixel at: (" << nx << ", " << ny << ")" << std::endl;
                            grid_map::Position3 p3;
                            grid_map::Position3 plane_label;
                            // LOG(INFO)<<"..";
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
                                // LOG(INFO)<<"..";
                                if (!std::isnan(map["label"](ny, nx)))
                                {
                                    int label_index = static_cast<int>(map["label"](ny, nx));
                                    normals[label_index] = pd.planes_info.at(label_index).normal;
                                }
                            }
                        }
                    }
                }
                //   LOG(INFO)<<"..";
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
    cv::imwrite("/home/lichao/catkin_pathplanning/src/path_planning/data/obstacle.png", obstacle_layer);
    // // cv::imshow("obstacle", obstacle_layer);
    // // cv::waitKey(0);
    
    // 求每个cell的可通过方向, 每5度一个间隔，那么360度有72个
    // 只对平面边缘区域进行检查，中间部分不用，默认全向通过
    // 边缘部分的定义为位于（脚宽/2～前脚掌长度区域内）
    double half_width_foot = 0.165;
    double fore_length_foot = 0.15;
    double hind_length_foot = 0.1;
    vector<cv::Mat> checks_Mat(72);
    for (int i = 0; i < pd.planes.size(); i++)
    {
        cv::Mat image = pd.planes.at(i);
        // 定义形态学操作的核
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        // 应用形态学操作：先膨胀再腐蚀（开操作），去除毛刺
        cv::Mat smoothedImage;
        cv::morphologyEx(image, smoothedImage, cv::MORPH_CLOSE, element);

        // 找出两个图像中不同的像素
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

        
        // 找到0-脚宽/2，这些区域的可通行型为NAN
        cv::Mat Nan_eroded;
        cv::Mat half_width_Element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(hind_length_foot/map.getResolution() * 2 + 1, hind_length_foot/map.getResolution() * 2 + 1));
        cv::erode(smoothedImage, Nan_eroded, half_width_Element);

        // cv::imshow("Nan_eroded"+ std::to_string(i), Nan_eroded);
        // cv::waitKey(0);

        cv::Mat NanFeasible = smoothedImage - Nan_eroded;

        // cv::imshow("NanFeasible" + std::to_string(i), NanFeasible);
        // cv::waitKey(0);

        // 找到>前脚掌长度的区域，这些区域的可通行性为full，全向通行
        cv::Mat Full_eroded;
        double full_length = std::sqrt(half_width_foot * half_width_foot + fore_length_foot * fore_length_foot);
        cv::Mat fore_foot_Element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(full_length/map.getResolution() * 2 + 1, full_length/map.getResolution() * 2 + 1));
        cv::erode(smoothedImage, Full_eroded, fore_foot_Element);
        // cv::imshow("Full_eroded"+ std::to_string(i), Full_eroded);
        // cv::waitKey(0);
        cv::Mat FullFeasible = smoothedImage - Full_eroded;
        // cv::imshow("FullFeasible"+ std::to_string(i), FullFeasible);
        // cv::waitKey(0);

        // 对脚宽/2～前脚掌长度，进行
        cv::Mat CheckImage = Nan_eroded - Full_eroded;

        cv::imshow("CheckImage"+ std::to_string(i), CheckImage);
        cv::waitKey(0);

        std::vector<cv::Point> white_points;
        cv::findNonZero(CheckImage, white_points);
        // for (auto & cv_p : white_points)
        // {
        //     for (int j = 0; j < 72; j++)
        //     {
        //         double angle = (j * 5)/57.3;
        //         if (isFeasible(grid_map::Index(cv_p.y, cv_p.x), angle, map))
        //         {
        //             checks_Mat.at(j).at<uchar>(cv_p) = 255;
        //         }
        //         else
        //         {
        //             checks_Mat.at(j).at<uchar>(cv_p) = 0;
        //         }
        //     }
        // }
    
    }


    // cv::Mat Check_Image = cv::Mat::zeros(pd.result.size(), CV_8UC1);
    // typedef std::vector<std::pair<int, int>> cellFeasibles;
    // vector<vector <cellFeasibles> > map_feasible;
    // map_feasible.resize(map.getSize().x());
    // for (int i = 0; i < map.getSize().x(); i++)
    // {
    //     // vector<cellFeasibles> rows_feasible;
    //     map_feasible.at(i).resize(map.getSize().y());
    //     for (int j = 0; j < map.getSize().y(); j++)
    //     {
    //         vector<bool> feasible_angles;
    //         bool yu_flag = true;
    //         bool huo_flag = false;
    //         for (auto & image : checks_Mat)
    //         {
    //             if (image.at<uchar>(i, j) == 255) // 注意行列顺序
    //             {
    //                 feasible_angles.emplace_back(true);
    //                 yu_flag  = yu_flag & true;
    //                 huo_flag = huo_flag | true;
    //             }
    //             else
    //             {
    //                 feasible_angles.emplace_back(false);
    //                 yu_flag  = yu_flag & false;
    //                 huo_flag = huo_flag | false;
    //             }
    //         }
    //         // 都可以通行和都不可以通行的情况对Check_Image不作处理
    //         if (yu_flag) // 都可以通行
    //         {
    //             /* code */
    //             continue;
    //         }
    //         if (!huo_flag) // 都不可以通行
    //         {
    //             /* code */
    //             continue;
    //         }
    //         Check_Image.at<uchar>(j, i) = 255;// 这些地方表示需要进行特殊处理的栅格
    //         cellFeasibles cell_feasibles = findTrueBlocks(feasible_angles);
    //         map_feasible.at(i).at(j) = cell_feasibles;
    //         // rows_feasible.emplace_back(cell_feasibles);
    //     }
    //     // map_feasible.emplace_back(rows_feasible);
    // }
    // // 这些存在不是全向通行的点，对这些进行聚类，聚类不仅仅是连续的块，而且还会针对可通行区间进行聚类
    // // 块聚类就比较简单，甚至可以舍弃一些类
    // // 可行区间的聚类会更麻烦
    // // 对同一区域内的可行区间进行聚类
    // // 用于存储连通组件信息
    // cv::Mat labels, stats, centroids;
    // // 获取连通组件数量及相关信息
    // int numLabels = cv::connectedComponentsWithStats(binaryImage, labels, stats, centroids, 8, CV_32S);
    // // 创建一个向量数组来存储每个白色区域的像素点
    // std::vector<std::vector<cv::Point>> whiteRegions(numLabels - 1);
    // vector<cv::Mat> regions(numLabels - 1);
    // for (int y = 0; y < labels.rows; y++) 
    // {
    //     for (int x = 0; x < labels.cols; x++) 
    //     {
    //         int label = labels.at<int>(y, x);
    //         if (label > 0) 
    //         { // label 0 是背景
    //             whiteRegions[label - 1].push_back(cv::Point(x, y));
    //             regions[label - 1].at<uchar>(y, x) = 255;
    //         }
    //     }
    // }
    // // 对每个区域内的理想可通行区域进行提取，采用投票法
    // for (auto & region : whiteRegions)
    // {
    //     vector<int> histogram(72);
    //     for (auto & cv_point : region)
    //     {
    //         // cv_point 索引到对应的可通行角度
    //         cellFeasibles cellFeasibles = map_feasible.at(cv_point.y).at(cv_point.x);
    //         for (auto fea : cellFeasibles)
    //         {
    //             for (int i = 0; i <= fea.second; i++)
    //             {
    //                 histogram[i + fea.first]++; 
    //             }
    //         }
    //     }
    //     // 获取每个区域的vector
    //     vector< vector<std::pair<int, int> >> parabolas = parabolasSeg(histogram);
    //     // 对每个抛物线，求其中的可行角度范围
    //     for (auto & parabola : parabolas)
    //     {
    //         struct bin 
    //         {
    //             int angle;
    //             int count;
    //             // 构造函数
    //             bin(int a, int c) : angle(a), count(c) {}
    //         };
    //         struct Compare {
    //             bool operator()(const bin& a, const bin& b) {
    //                 return a.count < b.count;  // second 数越大，优先级越高
    //             }
    //         };
    //         std::priority_queue<bin, std::vector<bin>, Compare> pq;
    //         for (auto & pai : parabola)
    //         {
    //             pq.push(bin(pai.first, pai.second));
    //         }
    //         // 有没有可能两次没有交集，必须有交集才行，假设满足
    //         int sum = 0;
    //         while (sum > 10) // 保证
    //         {
    //             sum += pq.top();
    //         }
    //         // 这个时候便获得了某个区域的最合理方向
    //     }
    // }
    


    // 根据这些聚类结果，确定地图内多个吸嘴，每个吸嘴对应一种情况，加上本身没有考虑吸嘴的一种情况


    // 搜索时根据当前方向和栅格允许方向的差值来调整搜索长度，固定长度*cos（夹角）

    // 不考虑负压吸力一个，每个负压吸力一个， 根据权重选择最合适的点进行生长


    grid_map_msgs::GridMap map_msg;
    grid_map::GridMapRosConverter::toMessage(map, map_msg);
    map_msg.info.header.frame_id = "map";
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pub.publish(map_msg);
        loop_rate.sleep();
    }
    // return 0;
}

