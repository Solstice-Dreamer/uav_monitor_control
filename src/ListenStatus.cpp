#include "uav_monitor_control/ListenStatus.hpp"
#include "uav_monitor_control/send_info.hpp"
#include <fstream>
#include <chrono>
#include <filesystem>

using namespace uav_monitor_control;

ListenStatus::ListenStatus(const std::string &name,
                           const BT::NodeConfiguration &config,
                           rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config),
      node_(node)
{
    // 与 MAVROS 保持一致的 QoS
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);

    // 订阅位置+姿态
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/mavros/local_position/pose", qos,
        std::bind(&ListenStatus::poseCb, this, std::placeholders::_1));

    // 订阅电池电量
    battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(
        "/mavros/battery", qos,
        std::bind(&ListenStatus::batteryCb, this, std::placeholders::_1));
}

void ListenStatus::poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    last_pose_msg_ = *msg;
}

void ListenStatus::batteryCb(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{
    battery_ = msg->percentage * 100.0; // 转为百分比
}

BT::NodeStatus ListenStatus::tick()
{
    // 处理回调，更新 last_pose_msg_ 和 battery_
    rclcpp::spin_some(node_);

    // 拆出三个部分，写入黑板
    const auto &p = last_pose_msg_.pose.position;
    const auto &q = last_pose_msg_.pose.orientation;

    geometry_msgs::msg::Point pos_msg;
    geometry_msgs::msg::Quaternion ori_msg;

    pos_msg.x = p.x;
    pos_msg.y = p.y;
    pos_msg.z = p.z;
    
    // 更新本地文件夹中的当前位置

    // 准备文件路径
    std::string home_dir = std::getenv("HOME");
    std::filesystem::path dir = home_dir + "/communication/path";
    std::filesystem::create_directories(dir);
    std::string current_point_file = (dir / ("current_point.txt")).string();

    // 写入start_point点坐标
    std::ofstream ofs(current_point_file);
    if (!ofs)
    {
        RCLCPP_ERROR(node_->get_logger(), "current_point: failed to open points file");
    }
    else
    {
        ofs << p.x << " " << p.y << " " << p.z << "\n";
        ofs.close();
    }

    ori_msg.w = q.w;
    ori_msg.x = q.x;
    ori_msg.y = q.y;
    ori_msg.z = q.z;

    setOutput("position_port", pos_msg);
    setOutput("orientation_port", ori_msg);
    setOutput("battery_port", battery_);

    // 日志
    RCLCPP_INFO(node_->get_logger(),
                "[ListenStatus] pos=(%.2f,%.2f,%.2f) ori=[w=%.4f,x=%.4f,y=%.4f,z=%.4f] battery=%.2f%%",
                pos_msg.x, pos_msg.y, pos_msg.z,
                ori_msg.w, ori_msg.x, ori_msg.y, ori_msg.z,
                battery_);

    // 可选：继续广播给 send_info
    UAV_info uav{
        {static_cast<float>(pos_msg.x),
         static_cast<float>(pos_msg.y),
         static_cast<float>(pos_msg.z)},
        {static_cast<float>(ori_msg.w),
         static_cast<float>(ori_msg.x),
         static_cast<float>(ori_msg.y),
         static_cast<float>(ori_msg.z)},
        battery_};
    send_info(uav, 10017);

    return BT::NodeStatus::SUCCESS;
}
