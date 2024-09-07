#pragma once
#include <grid_map_core/GridMap.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <peac/PEAC_plane_detection.hpp>
#include <vector>
#include <queue>
#include <list>
#include <unordered_map>
#include <unordered_set>

// 先判断终点所在的平面，再根据连通性进行规划

#define DEBUG

struct SupportArea
{
    double Up, Button, Left, Right;
    SupportArea(double up, double button, double left, double right)
    {
        CHECK(up > 0 && button > 0 && left > 0 && right > 0);
        Up = up; Button = button; Left = left; Right = right;
    }
    SupportArea()
    {

    }
};


struct bin_preq
{
    int angle; // 代表角度
    vector<cv::Point> points;
    bin_preq(int angle_, vector<cv::Point> & points_)
    {
        angle = angle_;
        points = points_;
    }
};



struct Compare 
{
    bool operator()(const bin_preq& a, const bin_preq& b) 
    {
        return a.points.size() < b.points.size();  // second 数越大，优先级越高
    }
};

struct DirectRegion
{
    int min, range;
    cv::Mat region;
};

struct CompareDR
{
    // 构造函数，接收一个外部变量
    CompareDR(const grid_map::GridMap & externalVar) : map(externalVar) {}
    bool operator()(const DirectRegion& a, const DirectRegion& b) 
    {
        // 获取两个质心
        std::vector<std::vector<cv::Point>> contours_a, contours_b;
        cv::findContours(a.region, contours_a, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::Moments mu_a = cv::moments(contours_a[0]);
        // 计算质心位置
        cv::Point centroid_a(mu_a.m10 / mu_a.m00, mu_a.m01 / mu_a.m00);


        cv::findContours(b.region, contours_b, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        cv::Moments mu_b = cv::moments(contours_b[0]);
        // 计算质心位置
        cv::Point centroid_b(mu_b.m10 / mu_b.m00, mu_b.m01 / mu_b.m00);

        grid_map::Position map_cor_a, map_cor_b;
        if (map.getPosition(grid_map::Index(centroid_a.y, centroid_a.x), map_cor_a) && map.getPosition(grid_map::Index(centroid_b.y, centroid_b.x), map_cor_b))
        {
            // 获取两者的方向平均值，
            double angle1 = a.min + ((double)(b.range) - 1.0)/2.0;
            double angle2 = a.min + ((double)(b.range) - 1.0)/2.0;
            double angle = (angle1 + angle2)/2.0;
            Eigen::AngleAxisd ad(angle, Eigen::Vector3d::UnitZ());
            Eigen::Vector3d v = ad.toRotationMatrix() * Eigen::Vector3d::UnitX();
            Eigen::Vector3d div = Eigen::Vector3d::Zero();
            div.head(2) = map_cor_b - map_cor_a;
            if (div.dot(v) > 0)
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
            LOG(ERROR)<<"error get point";
        }
    }
private:
    const grid_map::GridMap map;
};

// 图的邻接表表示，将可通行区域进行聚类
class Graph {
public:
    unordered_map<int, vector<int>> adjList;

    // 添加边
    void addEdge(int u, int v) 
    {
        adjList[u].push_back(v);
        adjList[v].push_back(u); // 无向图的边是双向的
    }

    // 深度优先搜索（DFS）
    void DFS(int node, unordered_set<int>& visited, vector<int>& cluster) 
    {
        visited.insert(node);
        cluster.push_back(node);
        for (int neighbor : adjList[node]) 
        {
            if (visited.find(neighbor) == visited.end()) 
            {
                DFS(neighbor, visited, cluster);
            }
        }
    }

    // 找到所有连通分量（聚类）
    vector<vector<int>> findClusters() 
    {
        unordered_set<int> visited;
        vector<vector<int>> clusters;

        for (auto& entry : adjList) 
        {
            int node = entry.first;
            if (visited.find(node) == visited.end()) 
            {
                vector<int> cluster;
                DFS(node, visited, cluster);
                clusters.push_back(cluster);
            }
        }
        return clusters;
    }
};

struct MergedDirectRegion
{
    cv::Mat merged_region;
    std::list<DirectRegion> merged_direct_regions;
};

class PathPlanning
{
private:
    ros::NodeHandle nh;
    grid_map::GridMap map;
    ros::Publisher pub;
    SupportArea support_area;
    plane_detection pd;
    Graph plane_graph;
    // 通过判断外轮廓有没有内轮廓，并根据内轮廓的面积大小，判断这块区域是不是都是障碍区域
    cv::Mat obstacle_layer;
    double obstacle_inflation_radius;
    int full_size; // 支撑区域内cell数
    
    // 只有这些区域的不是全向通行或者全向非通行
    std::vector<cv::Mat> checks_Mat;
    cv::Mat check_Mat; // 非全向通行的区域 这里还没有将障碍附近的可通行区域删除，注意是障碍附近
    cv::Mat Nan_feasible; // 所有不可通行的
    cv::Mat Full_feasible; // 所有全向通行的  可以将局部几个角度不能通行的点视为可全向通行

    vector<DirectRegion> drs;

    vector<MergedDirectRegion> mdrs;

    cv::Mat strictChecksMat;
    int strict_section;
    double obstacle_length;
    // 对于一个区域而言，如果里面有可通行方向较全向的像素，那么肯定会有可通行方向单一的像素。反之不成立
    // 为了方便筛选出这些区域，这一步很重要，但是由于check_Mat可能存在非常杂乱的情况，这可能不方便筛选
    // 为此，我们筛选出checks_Mat中通行性局限性更强的像素点，并对这些像素点进行聚类，分割出可通行区域和可通行方向。再将这些区域与checks_Mat内包含这些区域且相邻的像素点合并，得到真实的可通行区域块及可通行方向
public:
    PathPlanning(ros::NodeHandle & nodehand, grid_map::GridMap & map_, SupportArea support_area_);

    void constructPlaneAwareMap();

    void constructObstacleLayer(int chect_scope);

    bool isFeasible(grid_map::Index & index, double angle);
    bool isFeasibleNew(grid_map::Index & index, double angle);

    void checkFeasibleDirect();

    bool getPoints(grid_map::Position TL, grid_map::Position TR, grid_map::Position BL, grid_map::Position BR, vector<Eigen::Vector3d> & return_points);

    /**
     * @brief 对不能全向通行的区域进行聚类，获得可行区域内可通行方向
     * 
     * @return true 
     * @return false 
     */
    bool clustering();

    void clusterFeasibleRegions();

    // 根据起点平面和终点平面确定到达终点的平面轨迹。分层引导规划，靠近子终点时以全局终点引导为主，根据前后平面序列来确定子终点，并圆滑一下
    vector<int> getPlanePath(int start_plane, int goal_plane);

    inline cv::Mat getObstacleLayer()
    {
        return obstacle_layer;
    }

    inline cv::Mat getNanRegion()
    {
        return Nan_feasible;
    }

    inline cv::Mat getFullRegion()
    {
        return Full_feasible;
    }

    inline cv::Mat getCheckMat()
    {
        return check_Mat;
    }
    ~PathPlanning();


    // test
    
    void testisFeasible();
};



