#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

namespace uav_monitor_control
{

    class ListenStatus : public BT::SyncActionNode
    {
    public:
        ListenStatus(const std::string &name,
                     const BT::NodeConfiguration &config,
                     rclcpp::Node::SharedPtr node);

        // 三个输出端口：位置、姿态、电量
        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<geometry_msgs::msg::Point>("position_port"),
                BT::OutputPort<geometry_msgs::msg::Quaternion>("orientation_port"),
                BT::OutputPort<double>("battery_port")};
        }

        BT::NodeStatus tick() override;

    private:
        // 回调
        void poseCb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void batteryCb(const sensor_msgs::msg::BatteryState::SharedPtr msg);

        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;

        geometry_msgs::msg::PoseStamped last_pose_msg_;
        double battery_{0.0};
    };

} // namespace uav_monitor_control
