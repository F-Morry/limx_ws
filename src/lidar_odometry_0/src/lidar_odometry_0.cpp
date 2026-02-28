#include "lidar_odometry/lidar_odometry_0.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

LidarOdometry::LidarOdometry(double max_correspondence_distance, double transformation_epsilon, double maximum_iterations)
{
    gicp = std::make_shared<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>>();

    gicp->setMaxCorrespondenceDistance(max_correspondence_distance);
    gicp->setTransformationEpsilon(transformation_epsilon);
    gicp->setMaximumIterations(maximum_iterations);

    POSE_G_L = Eigen::Matrix4d::Zero(); 

    POSE_G_L.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    POSE_G_L.block<3, 1>(0, 3) = Eigen::Vector3d::Zero();
    POSE_G_L(3, 3) = 1.0;

    state_ptr = std::make_shared<State>();
    state_ptr->pose.linear() = POSE_G_L.block<3, 3>(0, 0);
    state_ptr->pose.translation() = POSE_G_L.block<3, 1>(0, 3);
    state_ptr->velocity = Eigen::Matrix<double, 6, 1>::Zero();
}

StatePtr LidarOdometry::get_state()
{
    return state_ptr;
}

// void LidarOdometry::process_scan_data(ScanDataPtr scan_ptr)
// {
//     if (last_scan_ptr) {
//         double dt = scan_ptr->timestamp - last_scan_ptr->timestamp; //时间
//         Eigen::Matrix4d transform_matrix = get_transform_matrix(last_scan_ptr, scan_ptr);
//         POSE_G_L = POSE_G_L * inverseSE3(transform_matrix);

//         Eigen::Matrix3d rotation_matrix = POSE_G_L.block<3, 3>(0, 0);
//         Eigen::AngleAxisd angleAxis(rotation_matrix);
//         Eigen::Vector3d axis = angleAxis.axis();
//         double angle = angleAxis.angle();

//         Eigen::Vector3d translation_velocity = transform_matrix.block<3, 1>(0, 3) / dt;
//         Eigen::Vector3d angular_velocity = (angle / dt) * axis;

//         state_ptr->pose.linear() = POSE_G_L.block<3, 3>(0, 0);
//         state_ptr->pose.translation() = POSE_G_L.block<3, 1>(0, 3);
//         state_ptr->velocity.block<3, 1>(0, 0) = translation_velocity;
//         state_ptr->velocity.block<3, 1>(3, 0) = angular_velocity;
//     }

//     last_scan_ptr = scan_ptr;
// }

void LidarOdometry::process_scan_data(ScanDataPtr scan_ptr)
{
    if (last_scan_ptr) {
        double dt = scan_ptr->timestamp - last_scan_ptr->timestamp;

        // ==================== 1. 运动畸变补偿（新增，最重要） ====================
        Eigen::Vector3d angular_vel = state_ptr->velocity.block<3,1>(3,0);  // 使用上一帧角速度估计
        auto source_deskew = deskew_pointcloud(last_scan_ptr->point_cloud, 
                                               last_scan_ptr->timestamp, scan_ptr->timestamp, angular_vel);
        auto target_deskew = deskew_pointcloud(scan_ptr->point_cloud, 
                                               last_scan_ptr->timestamp, scan_ptr->timestamp, angular_vel);

        // ==================== 2. 点云滤波 ====================
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_filtered(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_filtered(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setLeafSize(0.025f, 0.025f, 0.025f);   // 改小到 2.5cm，更适合 2D 雷达

        voxel.setInputCloud(source_deskew);
        voxel.filter(*source_filtered);

        voxel.setInputCloud(target_deskew);
        voxel.filter(*target_filtered);

        // 统计滤波（降低严格度）
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setMeanK(30);
        sor.setStddevMulThresh(1.5);   // 放宽一点

        sor.setInputCloud(source_filtered);
        sor.filter(*source_filtered);

        sor.setInputCloud(target_filtered);
        sor.filter(*target_filtered);

        // ==================== 3. GICP 配准 ====================
        Eigen::Matrix4d transform_matrix = get_transform_matrix(source_filtered, target_filtered);

        POSE_G_L = POSE_G_L * inverseSE3(transform_matrix);

        Eigen::Matrix3d rotation_matrix = POSE_G_L.block<3, 3>(0, 0);
        Eigen::AngleAxisd angleAxis(rotation_matrix);
        Eigen::Vector3d axis = angleAxis.axis();
        double angle = angleAxis.angle();

        Eigen::Vector3d translation_velocity = transform_matrix.block<3, 1>(0, 3) / dt;
        Eigen::Vector3d angular_velocity = (angle / dt) * axis;

        state_ptr->pose.linear() = POSE_G_L.block<3, 3>(0, 0);
        state_ptr->pose.translation() = POSE_G_L.block<3, 1>(0, 3);
        state_ptr->velocity.block<3, 1>(0, 0) = translation_velocity;
        state_ptr->velocity.block<3, 1>(3, 0) = angular_velocity;
    }

    last_scan_ptr = scan_ptr;
}

// Eigen::Matrix4d LidarOdometry::get_transform_matrix(ScanDataPtr source, ScanDataPtr target)
// {
//     // 获取原始点云（假设为 pcl::PointCloud<pcl::PointXYZ>::Ptr 类型）
//     pcl::PointCloud<pcl::PointXYZ>::Ptr source_raw(new pcl::PointCloud<pcl::PointXYZ>(source->point_cloud));
//     pcl::PointCloud<pcl::PointXYZ>::Ptr target_raw(new pcl::PointCloud<pcl::PointXYZ>(target->point_cloud));

//     // ---- 滤波源点云 ----
//     pcl::PointCloud<pcl::PointXYZ>::Ptr source_filtered(new pcl::PointCloud<pcl::PointXYZ>);

//     // 体素滤波（降采样）
//     pcl::VoxelGrid<pcl::PointXYZ> voxel;
//     voxel.setInputCloud(source_raw);
//     voxel.setLeafSize(0.1f, 0.1f, 0.1f);   // 体素大小可调
//     voxel.filter(*source_filtered);

//     // 统计滤波（去除离群点）
//     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//     sor.setInputCloud(source_filtered);
//     sor.setMeanK(50);                     // 邻近点数量
//     sor.setStddevMulThresh(1.0);          // 标准差倍数
//     sor.filter(*source_filtered);

//     // ---- 滤波目标点云（相同参数） ----
//     pcl::PointCloud<pcl::PointXYZ>::Ptr target_filtered(new pcl::PointCloud<pcl::PointXYZ>);
//     voxel.setInputCloud(target_raw);
//     voxel.filter(*target_filtered);
//     sor.setInputCloud(target_filtered);
//     sor.filter(*target_filtered);

//     pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);

//     gicp->setInputSource(source_filtered);
//     gicp->setInputTarget(target_filtered);
//     gicp->align(*align);

//     Eigen::Matrix4f src2tgt = gicp->getFinalTransformation();

//     return src2tgt.cast<double>();    
// }
// 新增：接受滤波后点云的 GICP 函数
Eigen::Matrix4d  LidarOdometry::get_transform_matrix(
    pcl::PointCloud<pcl::PointXYZ>::Ptr source, 
    pcl::PointCloud<pcl::PointXYZ>::Ptr target)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>());

    gicp->setInputSource(source);
    gicp->setInputTarget(target);
    gicp->align(*align);

    return gicp->getFinalTransformation().cast<double>();
}
// 新增：简单运动畸变补偿函数
pcl::PointCloud<pcl::PointXYZ>::Ptr LidarOdometry::deskew_pointcloud(
    const pcl::PointCloud<pcl::PointXYZ>& cloud, 
    double start_time, double end_time, 
    const Eigen::Vector3d& angular_vel)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr deskewed(new pcl::PointCloud<pcl::PointXYZ>());
    double scan_duration = end_time - start_time;

    for (size_t i = 0; i < cloud.size(); ++i) {
        double point_time = start_time + (i * scan_duration / cloud.size());  // 线性插值时间
        double delta_t = point_time - start_time;

        // 简单旋转补偿（假设角速度恒定）
        Eigen::AngleAxisd rot(delta_t * angular_vel.norm(), angular_vel.normalized());
        Eigen::Vector3d p(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
        Eigen::Vector3d p_deskew = rot * p;

        deskewed->push_back(pcl::PointXYZ(p_deskew.x(), p_deskew.y(), p_deskew.z()));
    }
    return deskewed;
}



