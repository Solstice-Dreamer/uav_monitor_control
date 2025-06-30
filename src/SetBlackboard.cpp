#include "uav_monitor_control/SetBlackboard.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

using namespace uav_monitor_control;

template <typename T>
SetBlackboard<T>::SetBlackboard(const std::string &name,
                                const BT::NodeConfiguration &config,
                                rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config),
      node_(node)
{
}

template <typename T>
BT::PortsList SetBlackboard<T>::providedPorts()
{
    return {
        BT::InputPort<std::string>("key", "要写入的黑板键"),
        BT::InputPort<T>("value", "要写入的值")};
}

template <typename T>
BT::NodeStatus SetBlackboard<T>::tick()
{
    std::string key;
    T val;

    if (!getInput("key", key) || !getInput("value", val))
    {
        RCLCPP_ERROR(node_->get_logger(),
                     "[SetBlackboard] missing key or value");
        return BT::NodeStatus::FAILURE;
    }

    // 写入黑板
    config().blackboard->set<T>(key, val);
    RCLCPP_INFO(node_->get_logger(),
                "[SetBlackboard] 写入 blackboard: %s", key.c_str());
    return BT::NodeStatus::SUCCESS;
}
template class SetBlackboard<geometry_msgs::msg::Point>;