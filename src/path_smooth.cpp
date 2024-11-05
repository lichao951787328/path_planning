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
    
    // for (int i = 0; i < path.poses.size() - 1; i++)
    // {
    //     grid_map::Position xi(path.poses[i].pose.position.x, path.poses[i].pose.position.y);
    //     grid_map::Position xi_1(path.poses[i + 1].pose.position.x, path.poses[i + 1].pose.position.y);
    //     grid_map::Index index_xi;
    //     map.getIndex(xi, index_xi);
    //     grid_map::Index index_xi_1;
    //     map.getIndex(xi_1, index_xi_1);
    //     int index = 0;
    //     for (grid_map::LineIterator iter(map, index_xi, index_xi_1); !iter.isPastEnd(); ++iter)
    //     {
    //         if (index % 5 == 0)
    //         {
    //             grid_map::Index index(*iter);
    //             grid_map::Position xi_new;
    //             map.getPosition(index, xi_new);
    //             path_grid.emplace_back(xi_new);
    //         }
    //         index++;
    //     }
    //     if (index % 5 == 0)
    //     {
    //         path_grid.erase(path_grid.end() - 1);
    //     }
    // }
    // if (path_grid.back().x() != path.poses.back().pose.position.x || path_grid.back().y() != path.poses.back().pose.position.y)    
    // {
    //     path_grid.emplace_back(grid_map::Position(path.poses.back().pose.position.x, path.poses.back().pose.position.y));
    // }
    
    
    
    // for (auto & point : path_grid)
    // {
    //     grid_map::Position3 point3 = grid_map::Position3::Zero();
    //     point3.head(2) = point;
    //     smoothed_path.emplace_back(point3);
    // }
    resolution = map.getResolution();
    // params.printParam();
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

    // cv::Mat display = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC3);
    // for (const auto& pt : controlPoints) {
    //     cv::circle(display, pt, 3, cv::Scalar(0, 0, 255), -1);
    //     // LOG(INFO)<<"point: "<<pt;
    // }
    // for (size_t i = 1; i < splinePoints.size(); ++i) {
    //     cv::line(display, splinePoints[i - 1], splinePoints[i], cv::Scalar(255, 0, 0), 1);
    // }

    // cv::imshow("B-Spline Curve", display);
    // cv::waitKey(0);
}

// void pathSmoother::smoothPath()
// {
//     int iterations = 0;
//     float totalWeight = params.wSmoothness + params.wCurvature + params.wObstacle;
//     LOG(INFO)<<"total weight: "<<totalWeight;
//     // float totalWeight = params.wSmoothness + params.wCurvature + params.wObstacle;
//     while (iterations < params.max_iterations)
//     {
//         // LOG(INFO)<<"iteration: "<<iterations;
//         for (int i = 1; i < path_grid.size() - 1; i++)
//         {
//             grid_map::Position xim1 = path_grid.at(i - 1);
//             grid_map::Position xi = path_grid.at(i);
//             grid_map::Position xip1 = path_grid.at(i + 1);
//             grid_map::Position correction = Eigen::Vector2d::Zero();
//             correction = correction - obstacleTerm(xi);
//             // LOG(INFO)<<"correction: "<<correction.transpose();
//             if (!map.isInside(xi + correction))
//             {
//                 continue;
//             }
//             correction = correction - smoothnessTerm(xim1, xi, xip1);
//             // LOG(INFO)<<"correction: "<<correction.transpose();
//             if (!map.isInside(xi + correction))
//             {
//                 continue;
//             }
//             correction = correction - curvatureTerm(xim1, xi, xip1);
//             // LOG(INFO)<<"correction: "<<correction.transpose();
//             if (!map.isInside(xi + correction))
//             {
//                 continue;
//             }
//             xi = xi + params.alpha * correction/totalWeight;
//             smoothed_path.at(i).head(2) = xi;
//             grid_map::Position Dxi = xi - xim1;
//             smoothed_path.at(i - 1).z() = std::atan2(Dxi.y(), Dxi.x());
//         }
//         iterations++;
//     }
// }


// void pathSmoother::constructDistanceMap()
// {
//     if (!obstacle_layer.empty())
//     {
//         cv::Mat invertedBinary;
//         cv::bitwise_not(obstacle_layer, invertedBinary);
//         // 计算距离变换
//         cv::distanceTransform(invertedBinary, distanceMap, cv::DIST_L2, 3);
//     }
//     else
//     {
//         LOG(ERROR)<<"obstacle layer is empty";
//     }
// }

// 注意当
// double pathSmoother::getDistance(grid_map::Position position)
// {
//     grid_map::Index index;
//     map.getIndex(position, index);
//     return getDistance(cv::Point2f(index.y(), index.x()));
// }

// double pathSmoother::getDistance(cv::Point2f point, double & distance)
// {
//     return distanceMap.at<float>(point.y, point.x) * map.getResolution();
// }

// bool pathSmoother::getNearestWhitePoint(cv::Point & input_point, cv::Point & nearest_point, double & distance)
// {
//     if (input_point.x > 0 && input_point.y > 0 && input_point.x < obstacle_layer.cols && input_point.y < obstacle_layer.rows)
//     {
//         if (check_image.at<uchar>(input_point) == 255)
//         {
//             double min_distance = std::numeric_limits<double>::max();
//             cv::Point nearestWhitePixel(-1, -1);
//             std::vector<cv::Point> whitePixels;
//             cv::findNonZero(obstacle_layer, whitePixels);
//             if (!whitePixels.empty())
//             {
//                 for (auto & whitePixel : whitePixels)
//                 {
//                     double distance = cv::norm(whitePixel - input_point) * resolution;
//                     if (distance < min_distance)
//                     {
//                         min_distance = distance;
//                         nearestWhitePixel = whitePixel;
//                     }
//                 }
//                 nearest_point =  nearestWhitePixel;
//                 distance = min_distance;
//                 return true;
//             }
//             else
//             {
//                 return false;
//             }
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

// bool pathSmoother::getNearestWhitePoint(grid_map::Position & input_point, grid_map::Position & nearest_point, double & distance)
// {
//     grid_map::Index index;
//     map.getIndex(input_point, index);
//     cv::Point nearest_white_point;
//     cv::Point input_point_cv(index.y(), index.x());
//     if (getNearestWhitePoint(input_point_cv, nearest_white_point, distance))
//     {
//         grid_map::Index nearest_white_index(nearest_white_point.y, nearest_white_point.x);
//         map.getPosition(nearest_white_index, nearest_point);
//         return true;
//     }
//     else
//     {
//         return false;
//     }
// }

// grid_map::Position pathSmoother::obstacleTerm(grid_map::Position & xi) 
// {
//     grid_map::Position gradient = Eigen::Vector2d::Zero();
//     // the distance to the closest obstacle from the current node
//     // float obsDst = getDistance(xi);
//     // grid_map::Index index;
//     grid_map::Position nearest_point = Eigen::Vector2d::Zero();
//     double obsDst = std::numeric_limits<double>::max();
//     if (getNearestWhitePoint(xi, nearest_point, obsDst))
//     {
//         grid_map::Position obsVct = xi - nearest_point;
//         if (obsDst < params.obsDMax && obsDst > 1e-6) 
//         {
//             gradient = params.wObstacle * 2 * (obsDst - params.obsDMax) * obsVct / obsDst;
//             return gradient;
//         }
//         else
//         {
//             return gradient;
//         }
//     }
//     else
//     {
//         return gradient;
//     }
// }

// grid_map::Position pathSmoother::voronoiTerm(grid_map::Position & xi) 
// {
//     grid_map::Position gradient = Eigen::Vector2d::Zero();
//     float obsDst = getDistance(xi);
//     if (map.isInside(xi))
//     {
//         grid_map::Position nearest_point;
//         double nearest_distance = 0;
//         if (getNearestWhitePoint(xi, nearest_point, nearest_distance))
//         {
//             grid_map::Position obsVct = xi - nearest_point;
//             if (obsDst < obsDMax && obsDst > 1e-6) 
//             {
//                 if (nearest_distance > 1e-6)
//                 {
//                     grid_map::Position PobsDst_Pxi = obsVct/obsDst;
//                     grid_map::Position PedgDst_Pxi = 
//                 }
//                 else
//                 {
//                     return gradient;
//                 }           
//             }
//         }
//         else
//         {
//             return gradient;
//         }
//     }
//     else
//     {
//         return gradient;
//     }
//     Vec2d obsVct(xi.x() - voronoi_.GetClosetObstacleCoor(xi).x(),
//                 xi.y() - voronoi_.GetClosetObstacleCoor(xi).y());
//     double edgDst = 0.0;
//     Vec2i closest_edge_pt = voronoi_.GetClosestVoronoiEdgePoint(xi, edgDst);
//     Vec2d edgVct(xi.x() - closest_edge_pt.x(), xi.y() - closest_edge_pt.y());
//     if (obsDst < vorObsDMax_ && obsDst > 1e-6) 
//     {
//         if (edgDst > 0) 
//         {
//             Vec2d PobsDst_Pxi = obsVct / obsDst;
//             Vec2d PedgDst_Pxi = edgVct / edgDst;
//             float PvorPtn_PedgDst = (alpha_ / alpha_ + obsDst) * (pow(obsDst - vorObsDMax_, 2) / pow(vorObsDMax_, 2)) * (obsDst / pow(obsDst + edgDst, 2));
//             float PvorPtn_PobsDst = (alpha_ / (alpha_ + obsDst)) * (edgDst / (edgDst + obsDst)) * ((obsDst - vorObsDMax_) / pow(vorObsDMax_, 2)) * (-(obsDst - vorObsDMax_) / (alpha_ + obsDst) - (obsDst - vorObsDMax_) / (obsDst + edgDst) + 2);
//             gradient = wVoronoi_ * (PvorPtn_PobsDst * PobsDst_Pxi + PvorPtn_PedgDst * PedgDst_Pxi) * 100;
//             return gradient;
//         }
//         return gradient;
//     }
//     return gradient;
// }


// grid_map::Position pathSmoother::curvatureTerm(grid_map::Position xim1, grid_map::Position xi, grid_map::Position xip1) 
// {
//     grid_map::Position gradient = grid_map::Position(0, 0);
//     // the vectors between the nodes
//     grid_map::Position Dxi = xi - xim1;
//     grid_map::Position Dxip1 = xip1 - xi;
//     // orthogonal complements vector
//     grid_map::Position p1, p2;
//     float absDxi = Dxi.norm();
//     float absDxip1 = Dxip1.norm();
//     // ensure that the absolute values are not null
//     if (absDxi > 0 && absDxip1 > 0) 
//     {
//         double InnerProd = (Dxi.x() * Dxip1.x() + Dxi.y() * Dxip1.y())/(absDxi * absDxip1);
//         if (InnerProd < -1)
//         {
//             InnerProd = -1;
//         }
//         else if (InnerProd > 1)
//         {
//             InnerProd = 1;
//         }     
//         float Dphi = std::acos(InnerProd);
//         float kappa = Dphi / absDxi;
//         if (kappa <= params.kappaMax) 
//         {
//             return grid_map::Position(0, 0);
//         } 
//         else 
//         {
//             //代入原文公式(2)与(3)之间的公式. 参考：
//             // Dolgov D, Thrun S, Montemerlo M, et al. Practical search techniques in path planning for 
//             //  autonomous driving[J]. Ann Arbor, 2008, 1001(48105): 18-80.
//             float absDxi1Inv = 1 / absDxi;
//             float PDphi_PcosDphi = -1 / std::sqrt(1 - std::pow(std::cos(Dphi), 2));
//             float u = -absDxi1Inv * PDphi_PcosDphi;
//             double InnerProd =  xi.x() * (- xip1.x()) + xi.y() * (- xip1.y());
//             p1 = (xi + xip1 * InnerProd / (xip1.norm() * xip1.norm()))/(absDxi * absDxip1);
//             // p1 = xi.ort(-xip1) / (absDxi * absDxip1);//公式(4)
//             p2 = (-xip1 - xi * InnerProd / (xi.norm() * xi.norm())) / (absDxi * absDxip1);
//             // p2 = -xip1.ort(xi) / (absDxi * absDxip1);
//             float s = Dphi / (absDxi * absDxi);
//             Eigen::Vector2d ones(1, 1);
//             // Vec2d ones(1, 1);
//             Eigen::Vector2d ki = u * (-p1 - p2) - (s * ones);
//             // Vec2d ki = u * (-p1 - p2) - (s * ones);
//             Eigen::Vector2d kim1 = u * p2 - (s * ones);
//             // Vec2d kim1 = u * p2 - (s * ones);
//             Eigen::Vector2d kip1 = u * p1;
//             // Vec2d kip1 = u * p1;
//             gradient = params.wCurvature * (0.25 * kim1 + 0.5 * ki + 0.25 * kip1);
//             if (std::isnan(gradient.x()) || std::isnan(gradient.y())) 
//             {
//                 return Eigen::Vector2d::Zero();
//             }
//             else 
//             {
//                 return gradient;
//             }
//         }
//     } 
//     else 
//     {
//         std::cout << "abs values not larger than 0" << std::endl;
//         std::cout << absDxi << absDxip1 << std::endl;
//         // Vec2d zeros;
//         std::cout << "curvatureTerm is 0 because abs values not larger than 0" << std::endl;
//         return Eigen::Vector2d::Zero();
//     }
// }

// grid_map::Position pathSmoother::smoothnessTerm(grid_map::Position xim, grid_map::Position xi, grid_map::Position xip) 
// {
//     return params.wSmoothness * (-4) * (xip - 2*xi + xim);
// }

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