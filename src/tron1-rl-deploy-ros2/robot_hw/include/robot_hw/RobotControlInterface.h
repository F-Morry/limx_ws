#ifndef ROBOT_CONTROL_INTERFACE_H
#define ROBOT_CONTROL_INTERFACE_H

#include <rclcpp/rclcpp.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <atomic>
#include <thread>
#include <memory>
#include <string>
#include <queue>
#include <mutex>
#include <condition_variable>

#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <nlohmann/json.hpp>
using json = nlohmann::json;

// ROS 消息类型
#include <robot_msgs/msg/robot_command.hpp>
#include <robot_msgs/msg/robot_info.hpp>
#include <robot_msgs/msg/robot_response.hpp>
#include <std_msgs/msg/int32.hpp>
// ADDED: 包含 IMU 消息头
#include <sensor_msgs/msg/imu.hpp>

typedef websocketpp::client<websocketpp::config::asio_client> client;
typedef websocketpp::lib::shared_ptr<websocketpp::lib::asio::ssl::context> context_ptr;
typedef client::connection_ptr connection_ptr;
typedef websocketpp::connection_hdl connection_hdl;

class RobotControlInterface {
public:
    RobotControlInterface(const std::string& robot_ip, int port, const std::string& accid);
    ~RobotControlInterface();

    // 控制接口
    void stand();
    void walk();
    void sitdown();
    void adjustHeight(int direction);
    void emergencyStop();
    void twist(double x, double y, double z);
    void setStairMode(bool enable);

private:
    // WebSocket 相关
    std::string ws_url;
    std::string robot_ip;
    int port;
    std::string accid;
    client ws_client_;
    connection_hdl current_hdl_;
    std::atomic<bool> is_connected_{false};
    std::thread ws_thread_;

    // ROS 相关
    rclcpp::Publisher<robot_msgs::msg::RobotCommand>::SharedPtr command_publisher_;
    rclcpp::Publisher<robot_msgs::msg::RobotInfo>::SharedPtr robot_info_publisher_;
    rclcpp::Publisher<robot_msgs::msg::RobotResponse>::SharedPtr response_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr battery_publisher_;
    // ADDED: IMU 发布者
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

    // 计数器
    std::atomic<uint64_t> command_count_{0};
    std::atomic<uint64_t> info_count_{0};
    std::atomic<uint64_t> response_count_{0};

    // 辅助函数
    void connect();
    std::string generate_message(const std::string& title, const json& data = json::object());
    void send_message(const std::string& message);
    void publish_command(const std::string& json_message);
    void handle_message(client::message_ptr msg);
    void publish_robot_info(const json& message);
    void publish_response(const json& response);
    // ADDED: 新增 IMU 处理函数
    void publish_imu(const json& message);
    bool should_publish_notification(const std::string& title);
};

#endif