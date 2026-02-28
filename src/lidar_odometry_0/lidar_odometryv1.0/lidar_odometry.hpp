#ifndef LIDAR_ODOMETRY_H
#define LIDAR_ODOMETRY_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/registration/gicp.h>
#include "lidar_odometry/utils.hpp"

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

        Eigen::Matrix4d get_transform_matrix(ScanDataPtr source, ScanDataPtr target);
        Eigen::Quaterniond last_imu_quat; // 新增：上一帧IMU姿态
        bool has_last_imu = false;        // 新增：首帧标志

        // 新增：点云补偿函数
        pcl::PointCloud<pcl::PointXYZ>::Ptr compensate_pointcloud(
            const pcl::PointCloud<pcl::PointXYZ> &raw_cloud,
            const Eigen::Quaterniond &imu_quat);
};

#endif
