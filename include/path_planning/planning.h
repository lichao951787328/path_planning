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
// #include <path_planning/nodes.h>
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

struct Node
{
    grid_map::Position position;
    double ori;
    double g_cost = 0;
    double h_cost = 0;
    double cost = 0;
    std::shared_ptr<Node> PreFootstepNode = nullptr;
};
typedef std::shared_ptr<Node> NodePtr;

struct CompareNode
{
    bool operator()(const NodePtr& a, const NodePtr& b) const
    {
        return a->cost > b->cost;
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
    // vector<Eigen::Vector2d> obstacles;
    double obstacle_inflation_radius;
    int full_size; // 支撑区域内cell数
    
    // 只有这些区域的不是全向通行或者全向非通行
    std::vector<cv::Mat> checks_Mat;
    cv::Mat check_Mat; // 非全向通行的区域 这里还没有将障碍附近的可通行区域删除，注意是障碍附近
    cv::Mat Nan_feasible; // 所有不可通行的
    cv::Mat Full_feasible; // 所有全向通行的  可以将局部几个角度不能通行的点视为可全向通行

    vector<DirectRegion> drs;

    vector<MergedDirectRegion> mdrs;

    cv::Mat debug_image;

    string package_path;
    cv::Mat check_image;
    vector<cv::Mat> obstacles;
    cv::Mat goal_obstacle;
    cv::Mat strictChecksMat;
    int strict_section;
    double obstacle_length;
    // 对于一个区域而言，如果里面有可通行方向较全向的像素，那么肯定会有可通行方向单一的像素。反之不成立
    // 为了方便筛选出这些区域，这一步很重要，但是由于check_Mat可能存在非常杂乱的情况，这可能不方便筛选
    // 为此，我们筛选出checks_Mat中通行性局限性更强的像素点，并对这些像素点进行聚类，分割出可通行区域和可通行方向。再将这些区域与checks_Mat内包含这些区域且相邻的像素点合并，得到真实的可通行区域块及可通行方向

    cv::Mat full_feasible_region;
    cv::Mat goal_vortex_region;
    cv::Point2f ray_Dir; // 终点像素方向
    double d_safe, d_vort, d_noinflu_offset, d_inf, d_noinflu;
    double d_g_att = 5;
    double d_g_att_th = 3;
    int d_safe_rad, d_vort_rad, d_inf_rad, d_noinflu_rad;
    string SAPF_X, SAPF_Y;
    int map_size_; 
    // Node start_, goal_;
    Node start;
    Eigen::Vector3d start_, goal_;
    grid_map::Index start_index_, goal_index_;
    std::unordered_map<int, Node> sample_list_;  // set of sample nodes
    int sample_num_;                             // max sample number
    double max_dist_;                            // max distance threshold
    double opti_sample_p_ = 0.05; 
    double step = 0.4; // 或许在找时需要变化长度，因为不一定正好是0.4
    // 不往后走
    vector<double> angles = {-90/57.3, -67.5/57.3, -45/57.3, -22.5/57.3, 0, 22.5/57.3, 45/57.3, 67.5/57.3, 90/57.3};

    double goal_obstacle_cof = 3;
    double gen_obstacle_cof = 2;
    double att_cof = 1;
public:
    PathPlanning(ros::NodeHandle & nodehand, grid_map::GridMap & map_, SupportArea support_area_, Eigen::Vector3d & start, Eigen::Vector3d & goal);

    // Node _generateRandomNode();
    // Node _findNearestPoint(std::unordered_map<int, Node>& list, const Node& node);
    // bool plan(const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);
    // bool _isAnyObstacleInPath(const Node& n1, const Node& n2);
    // bool _checkGoal(const Node& new_node);
    // Node APF_newpoint(Node & nearest_point, Node & rand_point);
    // std::vector<Node> _convertClosedListToPath(std::unordered_map<int, Node>& closed_list, const Node& start, const Node& goal);
    // int node2Index(Node & n);
    // Eigen::Vector2d repF();
    bool reachGoal(Node & node);
    void showAttPotential();
    void constructPlaneAwareMap();
    void constructGoalVortexRegion();
    // void computeObstacles();
    void constructObstacleLayer(int chect_scope);
    void constructFullFeasibleRegion(double safe_length);
    void computeObstacles();
    void computeRepObstacleGoal();
    void computeRepObstacle();
    void computeRep();
    void computeRadius(double d_safe, double d_vort, double d_noinflu_offset);
    void mergeAllObstacle();
    bool AttPotential(Node & node, Eigen::Vector2d & AttForce);
    bool getYaw(Node & node, double & yaw);
    bool ComputeNodeCost(NodePtr currentNodeP, NodePtr nextNodeP, double & cost);
    bool getSAPF(Node & node, Eigen::Vector2d & SAPF);
    void showSAPF();
    void checkFeasibleDirect();
    bool ComputeNodeCost(NodePtr currentNodeP, NodePtr nextNodeP);
    bool isTraversbility(Node & node);
    bool getAllPoints(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, vector<Eigen::Vector3d> & points);
    bool subRegionSupportPlane(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, int & plane_index);
    bool clustering();
    void showSAPFMatplotlib();
    std::pair<cv::Mat, cv::Mat> getSegMoreAndLess(cv::Mat region, cv::Mat goal_region, cv::Point2f Dir);
    void goalDirectInImage(double yaw);
    void showRepImage();
    void showSAPFImage();
    bool transitions(NodePtr currentNodeP, double yaw, std::vector<NodePtr> & nodes);
    bool CheckTraversability(Node & node);
    bool plan();
    grid_map::GridMap inline getMap()
    {
        return map;
    }
    // bool isFeasible(grid_map::Index & index, double angle);
    // bool isFeasibleNew(grid_map::Index & index, double angle);


    // void checkFeasibleDirect();

    // bool getPoints(grid_map::Position TL, grid_map::Position TR, grid_map::Position BL, grid_map::Position BR, vector<Eigen::Vector3d> & return_points);

    /**
     * @brief 对不能全向通行的区域进行聚类，获得可行区域内可通行方向
     * 
     * @return true 
     * @return false 
     */
    // bool clustering();

    // void clusterFeasibleRegions();

    // 根据起点平面和终点平面确定到达终点的平面轨迹。分层引导规划，靠近子终点时以全局终点引导为主，根据前后平面序列来确定子终点，并圆滑一下
    // vector<int> getPlanePath(int start_plane, int goal_plane);

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
    
    // void testisFeasible();
};



