#pragma once
// 防止重复包含

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace uav_monitor_control
{

    class StopFollow : public BT::SyncActionNode
    {
    public:
        StopFollow(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node);

        // 所有的 SyncActionNode 都必须实现这个静态方法，用来定义输入端口
        static BT::PortsList providedPorts()
        {
            return {/* 无需输入端口 */};
        }

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        void writeZeroToFile();
    };

} // namespace uav_monitor_control
