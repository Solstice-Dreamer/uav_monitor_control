#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_long.hpp>

namespace uav_monitor_control
{

    /**
     * @brief 同步动作节点：进入自动降落 (AUTO.LAND)
     *
     * 通过调用 mavros/cmd/command 服务（CommandLong）发起请求，
     * 不等待响应。
     */
    class LandAction : public BT::SyncActionNode
    {
    public:
        LandAction(const std::string &name,
                   const BT::NodeConfiguration &config,
                   rclcpp::Node::SharedPtr node);

        static BT::PortsList providedPorts() { return {}; }

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr cmd_client_;
    };

} // namespace uav_monitor_control
