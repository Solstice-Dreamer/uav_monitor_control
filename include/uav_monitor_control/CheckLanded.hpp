#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/extended_state.hpp>

namespace uav_monitor_control
{

    /**
     * @brief 条件节点：检查无人机是否已降落
     *
     * 构造时注入 ROS2 Node，用于订阅 /mavros/extended_state，
     * QoS 保持与 PX4 电量主题一致：BestEffort + Volatile，深度 10。
     */
    class CheckLanded : public BT::ConditionNode
    {
    public:
        /**
         * @param name   节点名称
         * @param config 行为树节点配置
         * @param node   注入的 ROS2 Node，用于创建 Subscription
         */
        CheckLanded(const std::string &name,
                    const BT::NodeConfiguration &config,
                    rclcpp::Node::SharedPtr node);

        /// 不需要任何端口
        static BT::PortsList providedPorts();

        /// tick 时检查最新的 landed_state
        BT::NodeStatus tick() override;

    private:
        /// ExtendedState 回调，更新 landed_state_
        void extendedStateCb(const mavros_msgs::msg::ExtendedState::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr ext_state_sub_;
        uint8_t landed_state_;
    };

} // namespace uav_monitor_control
