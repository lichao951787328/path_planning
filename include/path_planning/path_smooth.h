#include <nav_msgs/Path.h>
#include <grid_map_core/GridMap.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
using namespace std;

namespace path_smooth
{
    struct pathSmoothParams
    {
        // falloff rate
        double alpha = 0.1;
        double wObstacle = 0.1;
        double wCurvature = 0.2;
        double wSmoothness = 0.6;
        // 最大迭代次数
        int max_iterations = 20;
        // obsDMax的作用更显著，应该 vorObsDMax >= obsDMax。首先要不碰撞障碍物，在不碰的前提下，调整离障碍物的距离
        // maximum distance for obstacles to influence the voronoi field
        double vorObsDMax = 0.6; 
        // maximum distance to obstacles that is penalized
        double obsDMax = 1;
        // maximum possible curvature of the non-holonomic vehicle
        double kappaMax = 1.0 / 2.0; 

        pathSmoothParams()
        {

        }
        pathSmoothParams(const pathSmoothParams & param)
        {
            alpha = param.alpha;
            wObstacle = param.wObstacle;
            wCurvature = param.wCurvature;
            wSmoothness = param.wSmoothness;
            max_iterations = param.max_iterations;
            vorObsDMax = param.vorObsDMax;
            obsDMax = param.obsDMax;
            kappaMax = param.kappaMax;
        }
        pathSmoothParams & operator=(const pathSmoothParams & param)
        {
            if (this == &param)
            {
                return *this;
            }
            alpha = param.alpha;
            wObstacle = param.wObstacle;
            wCurvature = param.wCurvature;
            wSmoothness = param.wSmoothness;
            max_iterations = param.max_iterations;
            vorObsDMax = param.vorObsDMax;
            obsDMax = param.obsDMax;
            kappaMax = param.kappaMax;
            return *this;
        }
        void printParam()
        {
            ROS_INFO("alpha: %f", alpha);
            ROS_INFO("wObstacle: %f", wObstacle);
            ROS_INFO("wCurvature: %f", wCurvature);
            ROS_INFO("wSmoothness: %f", wSmoothness);
            ROS_INFO("max_iterations: %d", max_iterations);
            ROS_INFO("vorObsDMax: %f", vorObsDMax);
            ROS_INFO("obsDMax: %f", obsDMax);
            ROS_INFO("kappaMax: %f", kappaMax);
        }
    };

    struct BsplineParams
    {
        int degree = 8;
        int numPoints = 100;
        BsplineParams()
        {

        }
        BsplineParams(const BsplineParams & param)
        {
            degree = param.degree;
            numPoints = param.numPoints;
        }
        BsplineParams & operator=(const BsplineParams & param)
        {
            if (this == &param)
            {
                return *this;
            }
            degree = param.degree;
            numPoints = param.numPoints;
            return *this;
        }
        void printParam()
        {
            ROS_INFO("degree: %d", degree);
            ROS_INFO("numPoints: %d", numPoints);
        }
    };

    class pathSmoother
    {
    private:
        // pathSmoothParams params;
        BsplineParams Bspline_params;
        nav_msgs::Path path;
        // cv::Mat obstacle_layer;
        // cv::Mat check_image;
        double resolution;
        grid_map::GridMap map;
        vector<grid_map::Position> path_grid;
        vector<cv::Point2f> controlPoints;
        vector<grid_map::Position3> smoothed_path;
    public:
        pathSmoother(nav_msgs::Path path_, grid_map::GridMap & map_, BsplineParams & Bspline_params_);
        // pathSmoother(nav_msgs::Path path_, cv::Mat obstacle_layer_, cv::Mat check_image_, grid_map::GridMap & map_, pathSmoothParams & params_, BsplineParams & Bspline_params_);
        // void smoothPath();
        // void constructDistanceMap();
        // // bool getDistance(grid_map::Position position, double & distance);
        // // bool getDistance(cv::Point2f point, double & distance);
        // bool getNearestWhitePoint(cv::Point & input_point, cv::Point & nearest_point, double & distance);
        // bool getNearestWhitePoint(grid_map::Position & input_point, grid_map::Position & nearest_point, double & distance);
        // grid_map::Position obstacleTerm(grid_map::Position & xi);
        // grid_map::Position curvatureTerm(grid_map::Position xim1, grid_map::Position xi, grid_map::Position xip1);
        // grid_map::Position smoothnessTerm(grid_map::Position xim, grid_map::Position xi, grid_map::Position xip);
        nav_msgs::Path getSmoothedPath();
        nav_msgs::Path getRawPath();
        void smoothBSPline();
        ~pathSmoother();
    };
}




