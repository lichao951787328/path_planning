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
#include <nav_msgs/Path.h>
// #include <path_planning/nodes.h>
// 先判断终点所在的平面，再根据连通性进行规划

// #define DEBUG
// #define SHOW_PLANNING_PROCESS
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

    SupportArea & operator=(const SupportArea & s)
    {
        Up = s.Up; Button = s.Button; Left = s.Left; Right = s.Right;
        return *this;
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


struct planningParam
{
    SupportArea support_area;
    double resolution;
    // 障碍参数
    double obstacle_inflation_radius;
    // 如果障碍物的面积obstacle_length*obstacle_length/（分辨率*分辨率）大于这个值，那么就认为这个障碍物是不可通过的
    double obstacle_length;
    int obstacle_rad;
    double safe_region_radius;

    // 势场的参数
    // 终点障碍距离参数
    double d_safe_goal, d_vort_goal, d_noinflu_offset_goal, d_inf_goal, d_noinflu_goal;

    // 普通障碍距离参数
    double d_safe_gen, d_vort_gen, d_noinflu_offset_gen, d_inf_gen, d_noinflu_gen;

    int d_safe_rad_goal, d_vort_rad_goal, d_inf_rad_goal, d_noinflu_rad_goal;
    int d_safe_rad_gen, d_vort_rad_gen, d_inf_rad_gen, d_noinflu_rad_gen;
    // 势场力参数
    double goal_obstacle_cof = 8;
    double gen_obstacle_cof = 6;
    double d_g_att = 5;
    double d_g_att_cof = 0.4;

    // 规划参数
    double support_ratio;
    int support_num_th;
    double step;

    void computeRadius()
    {
        CHECK(d_safe_goal > 0 && d_safe_gen > 0);
        CHECK(d_vort_goal > d_safe_goal);
        CHECK(d_vort_gen > d_safe_gen);
        CHECK(d_noinflu_offset_goal > 0);
        CHECK(d_noinflu_offset_gen > 0);
        d_inf_goal = 2*d_vort_goal - d_safe_goal;
        d_noinflu_goal = d_inf_goal + d_noinflu_offset_goal;

        d_inf_gen = 2*d_vort_gen - d_safe_gen;
        d_noinflu_gen = d_inf_gen + d_noinflu_offset_gen;

        d_safe_rad_goal = d_safe_goal/resolution;
        d_vort_rad_goal = d_vort_goal/resolution;
        d_inf_rad_goal = d_inf_goal/resolution;
        d_noinflu_rad_goal = d_noinflu_goal/resolution;

        d_safe_rad_gen = d_safe_gen/resolution;
        d_vort_rad_gen = d_vort_gen/resolution;
        d_inf_rad_gen = d_inf_gen/resolution;
        d_noinflu_rad_gen = d_noinflu_gen/resolution;

        LOG(INFO)<<d_safe_goal<<" "<<d_vort_goal<<" "<<d_noinflu_offset_goal<<" "<<d_inf_goal<<" "<<d_noinflu_goal;
        LOG(INFO)<<d_safe_gen<<" "<<d_vort_gen<<" "<<d_noinflu_offset_gen<<" "<<d_inf_gen<<" "<<d_noinflu_gen;
        LOG(INFO)<<d_safe_rad_goal<<" "<<d_vort_rad_goal<<" "<<d_inf_rad_goal<<" "<<d_noinflu_rad_goal;
        LOG(INFO)<<d_safe_rad_gen<<" "<<d_vort_rad_gen<<" "<<d_inf_rad_gen<<" "<<d_noinflu_rad_gen;
    }

    void computeSupportNumTh()
    {
        support_num_th = (support_area.Button / resolution) * (support_area.Left / resolution) * support_ratio;
    }
    planningParam()
    {
        support_area = SupportArea(0.15, 0.1, 0.165, 0.165);
        resolution = 0.02;
         obstacle_length = 2;
        obstacle_rad = 6;
        safe_region_radius = 0.55;

        // 将这个距离参数分为终点障碍参数和普通障碍参数,通常终点障碍
        d_safe_goal = 0.6;
        d_vort_goal = 2;
        d_noinflu_offset_goal = 0.4;


        d_safe_gen = 0.6;
        d_vort_gen = 1.2;
        d_noinflu_offset_gen = 0.6;
        computeRadius();
        goal_obstacle_cof = 8;
        gen_obstacle_cof = 6;

        d_g_att = 5;
        d_g_att_cof = 0.4;

        support_ratio = 0.55;
        computeSupportNumTh();
        step = 0.4;
    }

    planningParam(const planningParam & param)
    {
        support_area = param.support_area;
        resolution = param.resolution;
        obstacle_inflation_radius = param.obstacle_inflation_radius;
        obstacle_length = param.obstacle_length;
        obstacle_rad = param.obstacle_rad;
        safe_region_radius = param.safe_region_radius;
        d_safe_goal = param.d_safe_goal;
        d_vort_goal = param.d_vort_goal;
        d_noinflu_offset_goal = param.d_noinflu_offset_goal;
        d_safe_gen = param.d_safe_gen;
        d_vort_gen = param.d_vort_gen;
        d_noinflu_offset_gen = param.d_noinflu_offset_gen;
        computeRadius();
        goal_obstacle_cof = param.goal_obstacle_cof;
        gen_obstacle_cof = param.gen_obstacle_cof;
        d_g_att = param.d_g_att;
        d_g_att_cof = param.d_g_att_cof;
        support_ratio = param.support_ratio;
        support_num_th = (support_area.Button / resolution) * (support_area.Left / resolution) * support_ratio;
        step = param.step;
    }
    planningParam & operator=(const planningParam & param)
    {
        if (this == &param)
        {
            return *this;
        }
        
        support_area = param.support_area;
        resolution = param.resolution;
        obstacle_inflation_radius = param.obstacle_inflation_radius;
        obstacle_length = param.obstacle_length;
        obstacle_rad = param.obstacle_rad;
        safe_region_radius = param.safe_region_radius;
        d_safe_goal = param.d_safe_goal;
        d_vort_goal = param.d_vort_goal;
        d_noinflu_offset_goal = param.d_noinflu_offset_goal;
        d_safe_gen = param.d_safe_gen;
        d_vort_gen = param.d_vort_gen;
        d_noinflu_offset_gen = param.d_noinflu_offset_gen;
        computeRadius();
        goal_obstacle_cof = param.goal_obstacle_cof;
        gen_obstacle_cof = param.gen_obstacle_cof;
        d_g_att = param.d_g_att;
        d_g_att_cof = param.d_g_att_cof;
        support_ratio = param.support_ratio;
        support_num_th = (support_area.Button / resolution) * (support_area.Left / resolution) * support_ratio;
        step = param.step;
        return *this;
    }
};
class PathPlanning
{
private:
    planningParam planning_param;
    grid_map::GridMap map;
    plane_detection pd;
    // 通过判断外轮廓有没有内轮廓，并根据内轮廓的面积大小，判断这块区域是不是都是障碍区域
    cv::Mat obstacle_layer;
    cv::Mat distanceMap;
    
    // 只有这些区域的不是全向通行或者全向非通行
    cv::Mat check_Mat; // 非全向通行的区域 这里还没有将障碍附近的可通行区域删除，注意是障碍附近


    cv::Mat debug_image;

    // 包地址，用于存储数据
    string package_path;

    // 这个区域内的势场不考虑
    cv::Mat check_image;

    // 障碍区域，用于计算障碍区域的势场
    vector<cv::Mat> obstacles;
    cv::Mat goal_obstacle;
    cv::Point seg_point; // 分割轮廓区域的点
    cv::Point2f seg_normal;

    // 确定终点障碍区域分割点的线点
    cv::Point2f seg_point1;
    cv::Point2f seg_point2;

    // 构建能够全向通行的区域
    cv::Mat full_feasible_region;
    cv::Point2f ray_Dir; // 终点像素方向

    string SAPF_X, SAPF_Y;    

    // 搜索参数
    nav_msgs::Path path;
    NodePtr startP = nullptr;
    Eigen::Vector3d start_, goal_;
    grid_map::Index goal_index_;
    std::unordered_set<std::string> close_set;
    // 不往后走
    vector<double> angles_safe = {-90/57.3, -67.5/57.3, -45/57.3, -22.5/57.3, 0, 22.5/57.3, 45/57.3, 67.5/57.3, 90/57.3};
    vector<double> angle_dang = {-67.5/57.3, -56.25/57.3, -45/57.3, -33.75/57.3, -22.5/57.3, -11.25/57.3, 0, 11.25/57.3, 22.5/57.3, 33.75/57.3, 45/57.3, 56.25/57.3, 67.5/57.3};
    vector<double> angles_trans_Astar = {337.5/57.3, 315/57.3, 292.5/57.3, 270/57.3, 247.5/57.3, 225/57.3, 202.5/57.3, 180/57.3, 157.5/57.3, 135/57.3, 112.5/57.3, 90/57.3, 67.5/57.3, 45/57.3, 22.5/57.3, 0};
public:
    // 构造函数
    PathPlanning(grid_map::GridMap & map_, Eigen::Vector3d & start, Eigen::Vector3d & goal, planningParam & param);

    // 构造平面高程图，确定障碍层
    void constructPlaneAwareMap();
    void constructObstacleLayer(int chect_scope);
    void constructFullFeasibleRegion(double safe_length);
    void computeObstacles();
    void goalDirectInImage(double yaw);

    // 构造势场
    void computeRepObstacleGoal();
    void computeRepObstacle();
    void computeRep();
    // void computeRadius(double d_safe, double d_vort, double d_noinflu_offset);
    void showAttPotential();
    // void constructGoalVortexRegion();
    void mergeAllObstacle();
    bool AttPotential(Node & node, Eigen::Vector2d & AttForce);
    bool getSAPF(Node & node, Eigen::Vector2d & SAPF);
    void showSAPF();
    void showSAPFMatplotlib();
    std::pair<cv::Mat, cv::Mat> getSegMoreAndLess(cv::Mat region, cv::Mat goal_region, cv::Point2f Dir);
    void showRepImage();
    void showSAPFImage();
    // void showJoinForceImage();
    bool getNearestPointInContour(vector<cv::Point> & contour, cv::Point & nearestPoint);
    bool getNearestPointInContourNew(vector<cv::Point> & contour, cv::Point & nearestPoint);
    // std::pair<cv::Mat, cv::Mat> regionSeg(vector<cv::Point> & contour, cv::Point segPoint, cv::Point2f Dir);
    vector<cv::Point> raySeg(cv::Point2f rayOrigin, vector<cv::Point> & contour);
    std::pair<vector<cv::Point>, vector<cv::Point> > raySegLeftRight(cv::Point2f rayOrigin, vector<cv::Point> & contour);
    cv::Point getNearestPoint(cv::Point2f rayOrigin, vector<cv::Point> & contour);
    std::pair<cv::Mat, cv::Mat> getSegMoreAndLess(cv::Point2f rayOrigin,  vector<cv::Point> & contour, cv::Point segPoint);

    


    // 规划
    bool reachGoal(Node & node);
    void drawCandidateSquare(NodePtr currentNodeP);
    bool getYaw(Node & node, double & yaw);
    bool computePath(NodePtr goal);
    bool getNodeString(NodePtr node, std::string & str);
    bool ComputeNodeCost(NodePtr currentNodeP, NodePtr nextNodeP, double apfForce);
    bool ComputeNodeCostAstar(NodePtr NodeP);
    bool isCorss(NodePtr currentNodeP);
    bool isTraversbility(Node & node);
    bool isTraversbilityTra(Node & node);
    bool SqurePoints(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, vector<Eigen::Vector3d> & points);
    bool getAllPoints(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, vector<Eigen::Vector3d> & points);
    bool subRegionSupportPlane(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, int & plane_index);
    bool transitions(NodePtr currentNodeP, double yaw, std::vector<NodePtr> & nodes);
    bool CheckTraversability(Node & node);
    bool plan();
    bool processing();

    nav_msgs::Path inline getPath()
    {
        return path;
    }
    grid_map::GridMap inline getMap()
    {
        return map;
    }

    inline cv::Mat getObstacleLayer()
    {
        return obstacle_layer;
    }

    inline cv::Mat getCheckMat()
    {
        return check_Mat;
    }

    // smooth
    // void constructDistanceMap();
    // double getDistance(cv::Point2f point);
    // bool getNearestWhitePoint(cv::Point & input_point, cv::Point & nearest_point, double & distance);

    ~PathPlanning();

};



