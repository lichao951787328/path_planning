#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <glog/logging.h>

// 这是一个点云预处理程序，用于将全局地图中的点云进行滤波和分割，提取出平面部分，并将平面部分高度小于5cm的点删除。
// 后续再去cloudcompare里将点云中的一些噪点删除
int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/lichao/catkin_pathplanning/src/path_planning/data/globalmap.pcd", pointcloud) == -1)
    {
        PCL_ERROR("Couldn't read file \n");
        return (-1);
    }
    
    LOG(INFO)<<"Loaded "<<pointcloud.size()<<" data points";
    // 体素滤波
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(pointcloud.makeShared());
    voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f);
    pcl::PointCloud<pcl::PointXYZ> filtered_pointcloud;
    voxel_filter.filter(filtered_pointcloud);

    // 统计滤波
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered_pointcloud.makeShared());
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(filtered_pointcloud);
    LOG(INFO)<<"Filtered "<<filtered_pointcloud.size()<<" data points";
    

    // 使用ransac算法将点云中最大的平面提取出来，然后将位于平面高度5cm一下的点删除
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>(filtered_pointcloud));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);

    seg.setInputCloud(cloud_ptr);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    // Extract the inliers
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_ptr);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    pcl::io::savePCDFileASCII("/home/lichao/catkin_pathplanning/src/path_planning/data/globalmap_filte1.pcd", *cloud_filtered);
    LOG(INFO)<<"PointCloud representing the planar component: "<<cloud_filtered->points.size()<<" data points.";

    for (size_t i = 0; i < coefficients->values.size(); i++)
    {
        LOG(INFO)<<i<<" "<<coefficients->values.at(i);
    }
    
    // Ensure the normal vector is pointing upwards
    if (coefficients->values[2] < 0)
    {
        for (auto& value : coefficients->values)
        {
            value = -value;
        }
    }

    // Remove points below the plane height + 5cm
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_above_plane(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : cloud_filtered->points)
    {
        double distance_to_plane = coefficients->values[0] * point.x + coefficients->values[1] * point.y + coefficients->values[2] * point.z + coefficients->values[3];
        if (distance_to_plane > 0.05)
        {
            cloud_above_plane->points.push_back(point);
        }
    }
    std::cout<<"plane coefficients: "<<coefficients->values[0]<<" "<<coefficients->values[1]<<" "<<coefficients->values[2]<<" "<<coefficients->values[3]<<std::endl;
    LOG(INFO)<<"ABOVE SIZE: "<<cloud_above_plane->points.size();
    // Transform the points so that the distance to the plane is reflected in the z-coordinate
    for (auto& point : cloud_above_plane->points)
    {
        double distance_to_plane = coefficients->values[0] * point.x + coefficients->values[1] * point.y + coefficients->values[2] * point.z + coefficients->values[3];
        point.z = distance_to_plane;
    }
    filtered_pointcloud = *cloud_above_plane;
    filtered_pointcloud.height = 1;
    filtered_pointcloud.width = filtered_pointcloud.points.size();
    pcl::io::savePCDFileASCII("/home/lichao/catkin_pathplanning/src/path_planning/data/globalmap_filte.pcd", filtered_pointcloud);
    // 后续还需要在cloudcompare里将点云中的一些噪点删除
    return 0;
}

