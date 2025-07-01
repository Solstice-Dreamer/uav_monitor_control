#include "uav_monitor_control/SetTakeoffTarget.hpp"
#include <fstream>
#include <chrono>
#include <filesystem>

using namespace uav_monitor_control;

SetTakeoffTarget::SetTakeoffTarget(const std::string &name,
                                   const BT::NodeConfiguration &config,
                                   rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config),
      node_(node)
{
}

BT::NodeStatus SetTakeoffTarget::tick()
{
    // 1. 获取并记录当前 ENU 位姿
    geometry_msgs::msg::Point current;
    if (!getInput("current_position_port", current))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[SetTakeoffTarget] missing current_position_port");
        return BT::NodeStatus::FAILURE;
    }

    // 准备文件路径
    std::string home_dir = std::getenv("HOME");
    std::filesystem::path dir = home_dir + "/communication/path";
    std::filesystem::create_directories(dir);
    std::string start_point_file = (dir / ("start_point.txt")).string();

    // 写入start_point点坐标
    std::ofstream ofs(start_point_file);
    if (!ofs)
    {
        RCLCPP_ERROR(node_->get_logger(), "start_point: failed to open points file");
    }
    else
    {
        ofs << current.x << " " << current.y << " " << current.z << "\n";
        ofs.close();
    }

    // 2. 获取命令字典
    std::map<std::string, std::string> cmd_map;
    if (!getInput("cmd_map_port", cmd_map))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[SetTakeoffTarget] missing cmd_map_port");
        return BT::NodeStatus::FAILURE;
    }

    // 3. 解析 alt 参数
    double alt = 0.0;
    auto it = cmd_map.find("alt");
    if (it == cmd_map.end())
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[SetTakeoffTarget] cmd_map 中未找到 \"alt\" 字段");
        return BT::NodeStatus::FAILURE;
    }
    try
    {
        alt = std::stod(it->second);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[SetTakeoffTarget] 无效的 alt 值：'%s'", it->second.c_str());
        return BT::NodeStatus::FAILURE;
    }

    // 4. 计算目标位姿（ENU 坐标系：正 z 向上，起飞则 z 增加 alt）
    geometry_msgs::msg::Point target;
    target.x = current.x;
    target.y = current.y;
    target.z = current.z + static_cast<float>(alt);

    // 5. 输出到黑板
    setOutput("target_position_port", target);

    // 6. 日志（ENU）
    RCLCPP_INFO(node_->get_logger(),
                "[SetTakeoffTarget] 计算目标 ENU 位姿: x=%.2f, y=%.2f, z=%.2f (alt=%.2f)",
                target.x,
                target.y,
                target.z,
                alt);

    return BT::NodeStatus::SUCCESS;
}
