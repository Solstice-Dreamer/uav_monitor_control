#include "uav_monitor_control/StopFollow.hpp"
#include <fstream>

using namespace uav_monitor_control;

StopFollow::StopFollow(const std::string &name,
                       const BT::NodeConfiguration &config,
                       rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config),
      node_(node)
{
}

BT::NodeStatus StopFollow::tick()
{
    try
    {
        writeZeroToFile();
        RCLCPP_INFO(node_->get_logger(), "[StopFollow] 已将 follow.sign 写入 0");
        return BT::NodeStatus::SUCCESS;
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(), "[StopFollow] 写文件失败: %s", e.what());
        return BT::NodeStatus::FAILURE;
    }
}

void StopFollow::writeZeroToFile()
{
    // 文件路径
    const std::string home = std::string(std::getenv("HOME"));
    const std::string filepath = home + "/communication/path/follow.sign";

    std::ofstream ofs(filepath, std::ios::trunc);
    if (!ofs.is_open())
    {
        throw std::runtime_error("无法打开文件: " + filepath);
    }

    ofs << '0';
    ofs.close();
}