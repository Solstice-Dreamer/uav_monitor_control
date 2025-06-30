#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

namespace uav_monitor_control
{

    /**
     * @brief 同步动作节点：切换飞控到 OFFBOARD 模式
     *
     * 通过调用 mavros/set_mode 服务发起请求，不等待响应。
     */
    class SwitchToOffboardMode : public BT::SyncActionNode
    {
    public:
        SwitchToOffboardMode(const std::string &name,
                             const BT::NodeConfiguration &config,
                             rclcpp::Node::SharedPtr node);

        static BT::PortsList providedPorts() { return {}; }

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr client_;
    };

} // namespace uav_monitor_control
