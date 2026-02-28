// #include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <deque>   // 用于低通滤波

using LaserScanMsg = sensor_msgs::msg::LaserScan;
using ImuMsg = sensor_msgs::msg::Imu;
using SyncPolicy = message_filters::sync_policies::ApproximateTime<LaserScanMsg, ImuMsg>;

class ScanPitchCorrector : public rclcpp::Node
{
public:
  ScanPitchCorrector() : Node("scan_corrector")
  {
    scan_sub_.subscribe(this, "/scan");
    imu_sub_.subscribe(this, "imu_plugin/out"); //仿真imu话题 
    //imu_sub_.subscribe(this, "ImuData");  //实机imu话题

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(50), scan_sub_, imu_sub_);

    sync_->registerCallback(std::bind(&ScanPitchCorrector::sync_callback, this,
                                      std::placeholders::_1, std::placeholders::_2));

    corrected_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
        "/scan_corrected", rclcpp::SensorDataQoS().best_effort());

    RCLCPP_INFO(this->get_logger(), "Scan Pitch Corrector 启动成功！使用 IMU 直接补偿 → 发布 /scan_corrected");
  }

private:
  void sync_callback(const LaserScanMsg::ConstSharedPtr &scan_msg,
                     const ImuMsg::ConstSharedPtr &imu_msg)
  {
    // 提取 roll, pitch, yaw
    tf2::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y,
                      imu_msg->orientation.z, imu_msg->orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // ==================== 优化1：低通滤波平滑角度（强烈推荐） ====================
    static std::deque<double> pitch_filter, roll_filter;
    const size_t filter_size = 8;   // 滤波窗口大小，可调

    pitch_filter.push_back(pitch);
    roll_filter.push_back(roll);
    if (pitch_filter.size() > filter_size) {
      pitch_filter.pop_front();
      roll_filter.pop_front();
    }

    double pitch_filtered = 0.0, roll_filtered = 0.0;
    for (auto p : pitch_filter) pitch_filtered += p;
    for (auto r : roll_filter) roll_filtered += r;
    pitch_filtered /= pitch_filter.size();
    roll_filtered /= roll_filter.size();

    // 限制最大补偿角度（防止极端姿态导致补偿错误）
    pitch_filtered = std::clamp(pitch_filtered, -0.4, 0.4);  // ±23度左右
    roll_filtered = std::clamp(roll_filtered, -0.4, 0.4);

    // 创建补偿后的 LaserScan
    auto corrected = std::make_shared<sensor_msgs::msg::LaserScan>(*scan_msg);
    corrected->ranges.clear();

    double angle = scan_msg->angle_min;
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
    {
      double r = scan_msg->ranges[i];
      if (r < scan_msg->range_min || r > scan_msg->range_max || std::isnan(r))
      {
        corrected->ranges.push_back(0.0);
        angle += scan_msg->angle_increment;
        continue;
      }

      double x = r * std::cos(angle);
      double y = r * std::sin(angle);
      double z = 0.0;

      // 使用滤波后的角度进行补偿
      tf2::Matrix3x3 R;
      R.setRPY(-roll_filtered, -pitch_filtered, 0.0);
      tf2::Vector3 p(x, y, z);
      tf2::Vector3 p_corrected = R * p;

      double r_corrected = std::sqrt(p_corrected.x() * p_corrected.x() + p_corrected.y() * p_corrected.y());
      corrected->ranges.push_back(r_corrected);

      angle += scan_msg->angle_increment;
    }

    corrected_pub_->publish(*corrected);
  }

  message_filters::Subscriber<LaserScanMsg> scan_sub_;
  message_filters::Subscriber<ImuMsg> imu_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr corrected_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanPitchCorrector>());
  rclcpp::shutdown();
  return 0;
}