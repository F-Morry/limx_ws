
#include "lidar_odometry/lidar_odometry.hpp"

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

void LidarOdometry::process_scan_data(ScanDataPtr scan_ptr)
{
    if (last_scan_ptr) {
        double dt = scan_ptr->timestamp - last_scan_ptr->timestamp; //时间

        // 获取当前和上一帧IMU姿态
        Eigen::Quaterniond current_imu_quat = scan_ptr->imu_orientation;
        Eigen::Quaterniond last_imu_quat_this = has_last_imu ? last_imu_quat : current_imu_quat;

        // 补偿点云（逆旋转到水平）
        auto source_comp = compensate_pointcloud(last_scan_ptr->point_cloud, last_imu_quat_this);
        auto target_comp = compensate_pointcloud(scan_ptr->point_cloud, current_imu_quat);

        pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);
        gicp->setInputSource(source_comp);
        gicp->setInputTarget(target_comp);
        gicp->align(*align);
        Eigen::Matrix4d transform_matrix = gicp->getFinalTransformation().cast<double>();

        // Eigen::Matrix4d transform_matrix = get_transform_matrix(last_scan_ptr, scan_ptr);
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

    // 更新上一帧
    last_imu_quat = scan_ptr->imu_orientation;
    has_last_imu = true;
}

//调用gicp算法进行历程计运算
Eigen::Matrix4d LidarOdometry::get_transform_matrix(ScanDataPtr source, ScanDataPtr target)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);

    gicp->setInputSource(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(source->point_cloud));
    gicp->setInputTarget(std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(target->point_cloud));
    gicp->align(*align);

    Eigen::Matrix4f src2tgt = gicp->getFinalTransformation();

    return src2tgt.cast<double>();    
}

//利用imu数据直接补偿点云数据（主要使用四元数）
pcl::PointCloud<pcl::PointXYZ>::Ptr LidarOdometry::compensate_pointcloud(
    const pcl::PointCloud<pcl::PointXYZ> &raw_cloud,
    const Eigen::Quaterniond &imu_quat)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr compensated(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Matrix3d R = imu_quat.toRotationMatrix();
    Eigen::Matrix3d R_inv = R.inverse(); // 逆旋转补偿

    // 如需仅补偿pitch，可切换以下代码：
    // double pitch = std::asin(2.0 * (imu_quat.w() * imu_quat.y() - imu_quat.z() * imu_quat.x()));
    // Eigen::AngleAxisd pitch_comp(-pitch, Eigen::Vector3d::UnitY());
    // Eigen::Matrix3d R_inv = pitch_comp.toRotationMatrix();

    for (const auto &pt : raw_cloud.points)
    {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        Eigen::Vector3d p_comp = R_inv * p;
        compensated->push_back(pcl::PointXYZ(p_comp.x(), p_comp.y(), p_comp.z()));
    }
    return compensated;
}
