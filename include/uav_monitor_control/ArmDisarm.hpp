#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

namespace uav_monitor_control
{

    /**
     * @brief 同步动作节点：解锁/上锁 (Arm/Disarm)
     *
     * 通过调用 mavros/cmd/arming 服务（CommandBool）发起请求，
     * 不等待响应，tick() 立即返回 SUCCESS。
     */
    class ArmDisarm : public BT::SyncActionNode
    {
    public:
        ArmDisarm(const std::string &name,
                  const BT::NodeConfiguration &config,
                  rclcpp::Node::SharedPtr node);

        /// 输入端口：arm=true 解锁；arm=false 上锁
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<bool>("arm", true, "true=Arm, false=Disarm")};
        }

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr client_;
    };

} // namespace uav_monitor_control
