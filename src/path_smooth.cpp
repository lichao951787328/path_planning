#include <path_planning/path_smooth.h>
#include <tf2/LinearMath/Quaternion.h>
#include <grid_map_core/iterators/LineIterator.hpp>
#include <glog/logging.h>

namespace path_smooth
{

// pathSmoother::pathSmoother(nav_msgs::Path path_, cv::Mat obstacle_layer_, cv::Mat check_image_, grid_map::GridMap & map_, pathSmoothParams & params_, BsplineParams & Bspline_params_):path(path_), map(map_), obstacle_layer(obstacle_layer_), check_image(check_image_), params(params_), Bspline_params(Bspline_params_)
pathSmoother::pathSmoother(nav_msgs::Path path_, grid_map::GridMap & map_, BsplineParams & Bspline_params_):path(path_), map(map_), Bspline_params(Bspline_params_)
{
    path_grid.clear();

    for (auto & pt : path.poses)
    {
        grid_map::Position xi(pt.pose.position.x, pt.pose.position.y);
        path_grid.emplace_back(xi);
        grid_map::Index index;
        map.getIndex(xi, index);
        controlPoints.emplace_back(cv::Point2f(index.y(), index.x()));
        smoothed_path.emplace_back(grid_map::Position3(xi.x(), xi.y(), 0));
    }
    resolution = map.getResolution();
}

// 计算B样条基函数
double bsplineBasis(int i, int k, double t, const std::vector<double>& knots) {
    if (k == 1) {
        return (t >= knots[i] && t < knots[i + 1]) ? 1.0 : 0.0;
    } else {
        double coef1 = 0.0, coef2 = 0.0;

        if (knots[i + k - 1] != knots[i]) {
            coef1 = (t - knots[i]) / (knots[i + k - 1] - knots[i]);
        }
        if (knots[i + k] != knots[i + 1]) {
            coef2 = (knots[i + k] - t) / (knots[i + k] - knots[i + 1]);
        }

        double term1 = coef1 * bsplineBasis(i, k - 1, t, knots);
        double term2 = coef2 * bsplineBasis(i + 1, k - 1, t, knots);
        
        return term1 + term2;
    }
}

// 生成B样条曲线点
std::vector<cv::Point2f> generateBSpline(const std::vector<cv::Point2f>& controlPoints, int degree, int numPoints) {
    int n = controlPoints.size();
    std::vector<cv::Point2f> splinePoints;
    
    // 创建节点向量
    std::vector<double> knots(n + degree + 1);
    for (int i = 0; i < knots.size(); ++i) {
        if (i < degree + 1)
            knots[i] = 0.0;
        else if (i > n - 1)
            knots[i] = 1.0;
        else
            knots[i] = (double)(i - degree) / (n - degree);
    }

    // 计算曲线上的点
    for (int j = 0; j < numPoints; ++j) {
        double t = (double)j / (numPoints - 1);
        cv::Point2f point(0.0, 0.0);
        
        for (int i = 0; i < n; ++i) {
            double basis = bsplineBasis(i, degree + 1, t, knots);
            // LOG(INFO)<<"basis: "<<basis;
            point += controlPoints[i] * basis;
        }
        
        splinePoints.push_back(point);
    }
    
    return splinePoints;
}

void pathSmoother::smoothBSPline()
{
    LOG(INFO)<<"start bspline smooth";
    // int degree = 8;   // B样条的阶数
    // int numPoints = 100;  // 曲线上的点数
    std::vector<cv::Point2f> splinePoints = generateBSpline(controlPoints, Bspline_params.degree, Bspline_params.numPoints);
    LOG(INFO)<<"OKK"<<splinePoints.size();
    splinePoints.erase(splinePoints.end() - 1);
    smoothed_path.clear();
    for (auto & point : splinePoints)
    {
        grid_map::Position xi;
        map.getPosition(grid_map::Index(point.y, point.x), xi);
        smoothed_path.emplace_back(grid_map::Position3(xi.x(), xi.y(), 0));
    }
}

nav_msgs::Path pathSmoother::getSmoothedPath()
{
    nav_msgs::Path smoothed_path_nav;
    for (int i = 0; i < smoothed_path.size() - 1; i++)
    {
        grid_map::Index index;
        map.getIndex(smoothed_path[i].head(2), index);
        grid_map::Position3 p3;
        map.getPosition3("elevation", index, p3);

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = p3.x();
        pose.pose.position.y = p3.y();
        pose.pose.position.z = p3.z();

        double yaw = std::atan2(smoothed_path[i+1].y() - smoothed_path[i].y(), smoothed_path[i+1].x() - smoothed_path[i].x());
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        smoothed_path_nav.poses.push_back(pose);
    }
    
    smoothed_path_nav.poses.emplace_back(path.poses.back());
    return smoothed_path_nav;
}

nav_msgs::Path pathSmoother::getRawPath()
{
    nav_msgs::Path raw_path_nav;
    for (auto & pt : path_grid)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pt.x();
        pose.pose.position.y = pt.y();
        pose.pose.position.z = 0;
        raw_path_nav.poses.push_back(pose);
    }
    return raw_path_nav;
}


pathSmoother::~pathSmoother()
{
}

}// end namespace path_planning