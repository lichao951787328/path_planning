#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/GridMap.hpp>
int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_bag");
    ros::NodeHandle nh;

    // Open the input bag file
    rosbag::Bag input_bag;
    input_bag.open("/home/lichao/catkin_pathplanning/src/path_planning/bag/sim_map2.bag", rosbag::bagmode::Read);

    // Open the output bag file
    rosbag::Bag output_bag;
    output_bag.open("/home/lichao/catkin_pathplanning/src/path_planning/bag/simulation_map.bag", rosbag::bagmode::Write);

    // Specify the topics to read from the input bag
    std::vector<std::string> topics;
    topics.push_back("path");

    rosbag::View view(input_bag, rosbag::TopicQuery(topics));
    

    BOOST_FOREACH(rosbag::MessageInstance const m, view)
    {
        grid_map_msgs::GridMap::ConstPtr msg = m.instantiate<grid_map_msgs::GridMap>();
        
        if (msg != NULL)
        {
            grid_map_msgs::GridMap grid_map = *msg;
            grid_map.info.header.frame_id = "map";
            output_bag.write("/global_map", ros::Time::now(), grid_map);
        }
    }

    input_bag.close();
    output_bag.close();

    return 0;
}