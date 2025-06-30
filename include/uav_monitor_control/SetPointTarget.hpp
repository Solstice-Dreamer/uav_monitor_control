#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <map>
#include <string>

namespace uav_monitor_control
{

    /**
     * @brief BT 同步动作节点：从命令字典中解析绝对目标位置
     *
     * 输入端口：
     *   cmd_map_port — std::map<std::string,std::string>，包含 "x","y","z" 字段
     * 输出端口：
     *   target_pose_port — geometry_msgs::msg::Point，绝对目标位姿（ENU）
     */
    class SetPointTarget : public BT::SyncActionNode
    {
    public:
        SetPointTarget(const std::string &name,
                       const BT::NodeConfiguration &config,
                       rclcpp::Node::SharedPtr node);

        static BT::PortsList providedPorts();

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
    };

} // namespace uav_monitor_control
