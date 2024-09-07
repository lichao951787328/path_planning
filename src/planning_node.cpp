#include <grid_map_core/GridMap.hpp>
#include <iostream>
#include <glog/logging.h>
#include <ros/ros.h>
#include <grid_map_core/iterators/SubmapIterator.hpp>
#include <path_planning/planning.h>
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
    grid_map::GridMap map;
    grid_map::Length length(10, 6);
    grid_map::Position position(5, 0);
    map.setGeometry(length, 0.02, position);

    map.add("elevation", 0.0);

    grid_map::Position stair_s(8, 2.8);
    grid_map::Position stair_e(6, 1);
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

    grid_map::Position obstacle_s(3, -0.5);
    grid_map::Index obstacle_s_index;
    if (map.getIndex(obstacle_s, obstacle_s_index))
    {
        grid_map::Size obstacle_size(1/map.getResolution(), 1/map.getResolution());
        for (grid_map::SubmapIterator iter(map, obstacle_s_index, obstacle_size); !iter.isPastEnd(); ++iter)
        {
            grid_map::Index iter_index(*iter);
            map["elevation"](iter_index.x(), iter_index.y()) = 1.0;
        }
    }
    SupportArea support_area(0.15, 0.1, 0.165, 0.165);
    PathPlanning path_planning(nh, map, support_area);
    path_planning.constructPlaneAwareMap();
    path_planning.constructObstacleLayer(6);
    cv::imshow("obstacle_layer", path_planning.getObstacleLayer());
    cv::waitKey(0);
    // path_planning.testisFeasible();
    path_planning.checkFeasibleDirect();

    cv::imshow("getNanRegion", path_planning.getNanRegion());
    cv::waitKey(0);

    cv::imshow("getFullRegion", path_planning.getFullRegion());
    cv::waitKey(0);

    cv::imshow("getCheckMat", path_planning.getCheckMat());
    cv::waitKey(0);

    path_planning.clustering();
    // path_planning.clusterFeasibleRegions();
    LOG(INFO)<<"OUT";
    return 0;
}