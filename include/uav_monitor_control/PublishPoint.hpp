#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

namespace uav_monitor_control
{

    class PublishPoint : public BT::SyncActionNode
    {
    public:
        PublishPoint(const std::string &name,
                     const BT::NodeConfiguration &config,
                     rclcpp::Node::SharedPtr node);

        BT::NodeStatus tick() override;

        static BT::PortsList providedPorts()
        {
            return {BT::InputPort<geometry_msgs::msg::Point>("target_position_port")};
        }

    private:
        void publishOnce(const geometry_msgs::msg::Point &target);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
    };

} // namespace uav_monitor_control