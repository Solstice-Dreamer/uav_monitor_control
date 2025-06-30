#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <map>
#include <string>

namespace uav_monitor_control
{

    class ReturnToHome : public BT::SyncActionNode
    {
    public:
        ReturnToHome(const std::string &name,
                     const BT::NodeConfiguration &config,
                     rclcpp::Node::SharedPtr node);

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<geometry_msgs::msg::Point>("home_position_port"),
                BT::InputPort<std::map<std::string, std::string>>("cmd_map_port"),
                BT::InputPort<geometry_msgs::msg::Point>("current_position_port")};
        }

        BT::NodeStatus tick() override;

    private:
        rclcpp::Node::SharedPtr node_;
    };

} // namespace uav_monitor_control