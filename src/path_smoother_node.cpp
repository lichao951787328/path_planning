#include <path_planning/path_smoother.h>
#include <ros/ros.h>

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "path_smoother_node");
    ros::NodeHandle nh;
    path_smooth::path_smoother path_smoother(nh);
    ros::AsyncSpinner spinner(2);  // Use n threads
    spinner.start();
    ros::waitForShutdown();
}

