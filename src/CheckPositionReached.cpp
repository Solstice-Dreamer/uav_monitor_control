#include "uav_monitor_control/CheckPositionReached.hpp"
#include <cmath>
#include <filesystem>
#include <fstream>

using namespace uav_monitor_control;

CheckPositionReached::CheckPositionReached(const std::string &name,
                                           const BT::NodeConfiguration &config,
                                           rclcpp::Node::SharedPtr node)
    : BT::ConditionNode(name, config),
      node_(node)
{
}

BT::PortsList CheckPositionReached::providedPorts()
{
    return {
        BT::InputPort<geometry_msgs::msg::Point>(
            "current_position_port", "当前 ENU 位姿(Point)"),
        BT::InputPort<geometry_msgs::msg::Point>(
            "target_position_port", "目标 ENU 位姿(Point)"),
        BT::InputPort<double>(
            "threshold", 0.05, "距离阈值 (m)")};
}

BT::NodeStatus CheckPositionReached::tick()
{
    geometry_msgs::msg::Point current;
    geometry_msgs::msg::Point target;
    double threshold = 0.05;

    // 读当前位姿
    if (!getInput("current_position_port", current))
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[CheckPositionReached] 无法获取 current_position_port");
        return BT::NodeStatus::FAILURE;
    }
    // 读目标位姿
    if (!getInput("target_position_port", target))
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[CheckPositionReached] 未初始化目标位置 → 视为已到达");
        // 在这里也写标志文件
    }
    // 读阈值（如果有的话）
    getInput("threshold", threshold);

    // 计算 ENU 空间欧氏距离
    double dx = current.x - target.x;
    double dy = current.y - target.y;
    double dz = current.z - target.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist <= threshold)
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[CheckPositionReached] 已到达 (dist=%.2f ≤ thr=%.2f)",
                    dist, threshold);

        // 只要到达，就把 reached.sign 写为 “1”
        std::string home_dir = std::getenv("HOME");
        std::filesystem::path dir = home_dir + "/communication/path";
        std::filesystem::create_directories(dir);
        std::string reached_file = (dir / "reached.sign").string();

        std::ofstream ofs(reached_file, std::ios::trunc);
        if (!ofs.is_open())
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[CheckPositionReached] 无法打开 reached.sign");
        }
        else
        {
            ofs << "1";
            ofs.close();
        }

        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(),
                    "[CheckPositionReached] 未到达 (dist=%.2f > thr=%.2f)",
                    dist, threshold);
        return BT::NodeStatus::FAILURE;
    }
}
