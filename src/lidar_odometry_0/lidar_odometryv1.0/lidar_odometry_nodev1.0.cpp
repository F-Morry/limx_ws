#include <cstdio>
#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp" // 新增
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "lidar_odometry/lidar_odometry.hpp"
#include "tf2_ros/transform_broadcaster.hpp"

using LaserScanMsg = sensor_msgs::msg::LaserScan;
using ImuMsg = sensor_msgs::msg::Imu;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<LaserScanMsg, ImuMsg>;

class LidarOdometryNode : public rclcpp::Node
{
  public:
    LidarOdometryNode() : Node("lidar_odometry_node")
    {
      RCLCPP_INFO(this->get_logger(), "lidar_odometry_node");

      parameter_initilization();

      double max_correspondence_distance;
      double transformation_epsilon;
      double maximum_iterations;
      std::string scan_topic_name;
      std::string odom_topic_name;
      std::string imu_topic_name;

      this->get_parameter("max_correspondence_distance", max_correspondence_distance);
      this->get_parameter("transformation_epsilon", transformation_epsilon);
      this->get_parameter("maximum_iterations", maximum_iterations);
      this->get_parameter("scan_topic_name", scan_topic_name);
      this->get_parameter("odom_topic_name", odom_topic_name);
      this->get_parameter("imu_topic_name", imu_topic_name); //新增imu订阅话题

      RCLCPP_INFO(this->get_logger(), "===== Configuration =====");

      RCLCPP_INFO(this->get_logger(), "max_correspondence_distance: %.4f", max_correspondence_distance);
      RCLCPP_INFO(this->get_logger(), "transformation_epsilon: %.4f", transformation_epsilon);
      RCLCPP_INFO(this->get_logger(), "maximum_iterations %.4f", maximum_iterations);
      RCLCPP_INFO(this->get_logger(), "scan_topic_name: %s", scan_topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "odom_topic_name: %s", odom_topic_name.c_str());
      RCLCPP_INFO(this->get_logger(), "imu_topic_name: %s", imu_topic_name.c_str());

      //实例化一个GICP算法对象
      lidar_odometry_ptr = std::make_shared<LidarOdometry>(max_correspondence_distance, transformation_epsilon, maximum_iterations);

      odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 100);

      // 同步订阅 LaserScan 和 IMU
      // 同步订阅（使用 std::make_shared 初始化）
      scan_sub = std::make_shared<message_filters::Subscriber<LaserScanMsg>>(this, scan_topic_name,rclcpp::SensorDataQoS().get_rmw_qos_profile());
      imu_sub = std::make_shared<message_filters::Subscriber<ImuMsg>>(this, imu_topic_name);
      sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
          SyncPolicy(10), *scan_sub, *imu_sub); // 使用完整命名空间
      sync->registerCallback(std::bind(&LidarOdometryNode::sync_callback, this,
                                       std::placeholders::_1, std::placeholders::_2));

      // scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
      //   scan_topic_name, 1000, std::bind(&LidarOdometryNode::scan_callback, this, std::placeholders::_1)
      // );
      // 初始化 TF broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

    private:
      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; //add tf broadcaster
      rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
      std::shared_ptr<LidarOdometry> lidar_odometry_ptr;

      //定义imu订阅
      std::shared_ptr<message_filters::Subscriber<ImuMsg>> imu_sub;
      std::shared_ptr<message_filters::Subscriber<LaserScanMsg>> scan_sub;
      std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync;


      void parameter_initilization() {
        this->declare_parameter<double>("max_correspondence_distance", 1.0);
        this->declare_parameter<double>("transformation_epsilon", 0.005);
        this->declare_parameter<double>("maximum_iterations", 30);
        this->declare_parameter<std::string>("scan_topic_name", "scan");
        this->declare_parameter<std::string>("odom_topic_name", "scan_odom");
        this->declare_parameter<std::string>("imu_topic_name", "imu_plugin/out");
      }
      
      // 新的回调函数：同步回调LaserScan + IMU
      void sync_callback(const LaserScanMsg::ConstSharedPtr &scan_msg,
                         const ImuMsg::ConstSharedPtr &imu_msg)
      {
        // 激光转点云（保持您原有的laser2cloudmsg和cloudmsg2cloud函数）
        auto point_cloud_msg = laser2cloudmsg(scan_msg);
        auto pcl_point_cloud = cloudmsg2cloud(point_cloud_msg);

        // 创建ScanData并填充IMU姿态
        auto scan_data = std::make_shared<ScanData>();
        scan_data->timestamp = scan_msg->header.stamp.sec + scan_msg->header.stamp.nanosec / 1e9;
        scan_data->point_cloud = pcl_point_cloud;
        scan_data->imu_orientation = Eigen::Quaterniond(
            imu_msg->orientation.w,
            imu_msg->orientation.x,
            imu_msg->orientation.y,
            imu_msg->orientation.z);

        // 处理并发布
        lidar_odometry_ptr->process_scan_data(scan_data);
        publish_odometry();
      }

      // void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
      // {
      //   auto point_cloud_msg = laser2cloudmsg(scan_msg);
      //   auto pcl_point_cloud = cloudmsg2cloud(point_cloud_msg);

      //   auto scan_data = std::make_shared<ScanData>();
      //   scan_data->timestamp = scan_msg->header.stamp.sec + scan_msg->header.stamp.nanosec / 1e9;
      //   scan_data->point_cloud = pcl_point_cloud;

      //   lidar_odometry_ptr->process_scan_data(scan_data); //处理数据
      //   publish_odometry();
      // }

      void publish_odometry() {
        auto state = lidar_odometry_ptr->get_state();
        rclcpp::Time now = this->now();
        std::string fixed_id = "odom";

        nav_msgs::msg::Odometry odom_msg;

        odom_msg.header.frame_id = fixed_id;
        odom_msg.child_frame_id = "scan";
        odom_msg.header.stamp = this->get_clock()->now();

        odom_msg.pose.pose = Eigen::toMsg(state->pose);
        odom_msg.twist.twist = Eigen::toMsg(state->velocity);

        odom_publisher->publish(odom_msg);

        // ==================== 发布 TF 变换 ====================
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_Link";

        // 从 state 中取出位姿
        transform.transform.translation.x = state->pose.translation().x();
        transform.transform.translation.y = state->pose.translation().y();
        transform.transform.translation.z = state->pose.translation().z();

        Eigen::Quaterniond q(state->pose.rotation());
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
      }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
