#include "uav_monitor_control/CheckLanded.hpp"

using namespace uav_monitor_control;

CheckLanded::CheckLanded(const std::string &name,
                         const BT::NodeConfiguration &config,
                         rclcpp::Node::SharedPtr node)
    : BT::ConditionNode(name, config),
      node_(node),
      // 默认设为 undefined，直到收到第一条消息
      landed_state_(mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED)
{
    // 与 PX4 电量主题保持一致的 QoS：depth=10, BestEffort, Volatile
    rclcpp::QoS qos(10);
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);

    // 创建带 QoS 的订阅
    ext_state_sub_ = node_->create_subscription<mavros_msgs::msg::ExtendedState>(
        "/mavros/extended_state",
        qos,
        std::bind(&CheckLanded::extendedStateCb, this, std::placeholders::_1));
}

BT::PortsList CheckLanded::providedPorts()
{
    return {}; // 无需输入或输出端口
}

void CheckLanded::extendedStateCb(const mavros_msgs::msg::ExtendedState::SharedPtr msg)
{
    landed_state_ = msg->landed_state;
}

BT::NodeStatus CheckLanded::tick()
{
    using Extended = mavros_msgs::msg::ExtendedState;
    if (landed_state_ == Extended::LANDED_STATE_ON_GROUND)
    {
        RCLCPP_INFO(node_->get_logger(), "[CheckLanded] 已检测到落地");
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        RCLCPP_DEBUG(node_->get_logger(),
                     "[CheckLanded] 未落地 (landed_state=%u)", landed_state_);
        return BT::NodeStatus::FAILURE;
    }
}