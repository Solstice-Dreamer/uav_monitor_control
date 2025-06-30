#include "uav_monitor_control/CheckCommandName.hpp"

using namespace uav_monitor_control;

CheckCommandName::CheckCommandName(const std::string &name,
                                   const BT::NodeConfiguration &config)
    : BT::ConditionNode(name, config)
{
}

BT::PortsList CheckCommandName::providedPorts()
{
    return {
        BT::InputPort<std::map<std::string, std::string>>(
            "cmd_map_port", "命令字典，包含 \"Name\" 字段"),
        BT::InputPort<std::string>(
            "expected_port", "预期的命令名称")};
}

BT::NodeStatus CheckCommandName::tick()
{
    std::map<std::string, std::string> cmd_map;
    std::string expected;

    // 尝试从黑板读取，但即便缺失也直接当条件不满足
    getInput("cmd_map_port", cmd_map);
    getInput("expected_port", expected);

    auto it = cmd_map.find("Name");
    if (it != cmd_map.end() && it->second == expected)
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}
