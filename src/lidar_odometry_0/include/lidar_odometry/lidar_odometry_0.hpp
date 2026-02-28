
#ifndef LIDAR_ODOMETRY_H
#define LIDAR_ODOMETRY_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/registration/gicp.h>
#include "lidar_odometry/utils_0.hpp"

class LidarOdometry
{
    public:
        LidarOdometry(double max_correspondence_distance = 1.0, double transformation_epsilon = 0.001, double maximum_iterations = 1000);
        StatePtr get_state();
        void process_scan_data(const ScanDataPtr scan_data);
    private:

        ScanDataPtr last_scan_ptr;
        StatePtr state_ptr;
        Eigen::Matrix4d POSE_G_L; // pose in SE(3) (ground -> LiDAR)

        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>::Ptr gicp;

        //Eigen::Matrix4d get_transform_matrix(ScanDataPtr source, ScanDataPtr target);
        Eigen::Matrix4d get_transform_matrix(
            pcl::PointCloud<pcl::PointXYZ>::Ptr source, 
            pcl::PointCloud<pcl::PointXYZ>::Ptr target);
        pcl::PointCloud<pcl::PointXYZ>::Ptr deskew_pointcloud(
            const pcl::PointCloud<pcl::PointXYZ>& cloud, 
            double start_time, double end_time, 
            const Eigen::Vector3d& angular_vel);
};

#endif

