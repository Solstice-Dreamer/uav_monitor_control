#include "uav_monitor_control/SetPointTarget.hpp"

using namespace uav_monitor_control;

SetPointTarget::SetPointTarget(const std::string &name,
                               const BT::NodeConfiguration &config,
                               rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config),
      node_(node)
{
}

BT::PortsList SetPointTarget::providedPorts()
{
    return {
        BT::InputPort<std::map<std::string, std::string>>(
            "cmd_map_port", "命令字典，包含绝对坐标 x,y,z"),
        BT::OutputPort<geometry_msgs::msg::Point>(
            "target_position_port", "解析后的目标位姿（ENU 坐标系）")};
}

BT::NodeStatus SetPointTarget::tick()
{
    // 1. 读取命令字典
    std::map<std::string, std::string> cmd_map;
    if (!getInput("cmd_map_port", cmd_map))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[SetPointTarget] 未获取到 cmd_map_port");
        return BT::NodeStatus::FAILURE;
    }

    // 2. 解析 x, y, z
    double x, y, z;
    try
    {
        x = std::stod(cmd_map.at("x"));
        y = std::stod(cmd_map.at("y"));
        z = std::stod(cmd_map.at("z"));
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[SetPointTarget] 解析 x/y/z 失败: %s",
                     e.what());
        return BT::NodeStatus::FAILURE;
    }

    // 3. 构造 ENU Point
    geometry_msgs::msg::Point target;

    target.x = static_cast<float>(x);
    target.y = static_cast<float>(y);
    target.z = static_cast<float>(z);
    // 姿态保持默认（0,0,0,1），如需可添加 orientation 输入端口

    // 4. 输出到黑板
    setOutput("target_position_port", target);

    // 5. 日志
    RCLCPP_INFO(node_->get_logger(),
                "[SetPointTarget] 解析目标 ENU 位姿: x=%.2f, y=%.2f, z=%.2f",
                x, y, z);
    return BT::NodeStatus::SUCCESS;
}
