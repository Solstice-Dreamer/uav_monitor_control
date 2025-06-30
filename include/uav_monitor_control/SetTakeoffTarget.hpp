#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <string>

namespace uav_monitor_control
{

    /**
     * @brief BT 同步动作节点：根据当前 ENU 位姿和指令字典中的 "alt"，
     *        计算起飞目标 ENU 位姿。
     *
     * 输入：
     *   - current_position_port (geometry_msgs::msg::Point)：当前 ENU 位姿
     *   - cmd_map_port     (std::map<std::string,std::string>)：命令字典，包含 "alt"
     * 输出：
     *   - target_pose_port (geometry_msgs::msg::Point)：计算后的起飞目标 ENU 位姿
     */
    class SetTakeoffTarget : public BT::SyncActionNode
    {
    public:
        SetTakeoffTarget(const std::string &name,
                         const BT::NodeConfiguration &config,
                         rclcpp::Node::SharedPtr node);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<geometry_msgs::msg::Point>(
                    "current_position_port", "当前 ENU 位姿(PoseStamped)"),
                BT::InputPort<std::map<std::string, std::string>>(
                    "cmd_map_port", "命令字典，包含 key=\"alt\""),
                BT::OutputPort<geometry_msgs::msg::Point>(
                    "target_position_port", "计算后的起飞目标位姿(PoseStamped)"),
            };
        }

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
    };

} // namespace uav_monitor_control
