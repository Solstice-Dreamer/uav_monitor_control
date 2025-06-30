#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include <string>
#include <map>

namespace uav_monitor_control
{

    /**
     * @brief 条件节点：检查命令字典中 "Name" 字段是否匹配预期
     *
     * 输入端口：
     *   cmd_map_port  — std::map<std::string,std::string>，命令字典，包含 "Name" 键
     *   expected_port — std::string，预期的命令名称
     *
     * 如果 cmd_map["Name"] == expected 返回 SUCCESS，否则 FAILURE。
     */
    class CheckCommandName : public BT::ConditionNode
    {
    public:
        CheckCommandName(const std::string &name,
                         const BT::NodeConfiguration &config);

        /// 声明要使用的端口
        static BT::PortsList providedPorts();

        /// tick 函数实现条件检查
        BT::NodeStatus tick() override;
    };

} // namespace uav_monitor_control
