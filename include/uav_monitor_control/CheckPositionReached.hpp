#pragma once

#include <behaviortree_cpp_v3/condition_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace uav_monitor_control
{

    /**
     * @brief 条件节点：判断 current 和 target 是否接近
     *
     * 输入端口：
     *   current_position_port — geometry_msgs::msg::Point，当前 ENU 位姿
     *   target_position_port  — geometry_msgs::msg::Point，可选；未提供则视为“已到达”
     *   threshold             — double，距离阈值（米），默认为 1.0
     */
    class CheckPositionReached : public BT::ConditionNode
    {
    public:
        CheckPositionReached(const std::string &name,
                             const BT::NodeConfiguration &config,
                             rclcpp::Node::SharedPtr node);

        /// 声明使用的端口
        static BT::PortsList providedPorts();

        /// tick 时执行距离判断
        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
    };

} // namespace uav_monitor_control
