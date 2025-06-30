#include "uav_monitor_control/PublishPoint.hpp"

using namespace uav_monitor_control;

PublishPoint::PublishPoint(const std::string &name,
                           const BT::NodeConfiguration &config,
                           rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node)
{
    // 在下面是否也要用这个qos
    // rclcpp::QoS qos(10);
    // qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    // qos.durability(rclcpp::DurabilityPolicy::Volatile);
    // 创建 Publisher
    setpoint_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/mavros/setpoint_position/local", 10);
}

BT::NodeStatus PublishPoint::tick()
{
    geometry_msgs::msg::Point target;
    if (!getInput("target_position_port", target))
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[PublishPoint] 缺少输入端口 target_position_port");
        return BT::NodeStatus::SUCCESS;
    }

    publishOnce(target);
    return BT::NodeStatus::SUCCESS;
}

void PublishPoint::publishOnce(const geometry_msgs::msg::Point &target)
{
    geometry_msgs::msg::PoseStamped msg;

    // 填 header
    msg.header.stamp = node_->now();
    msg.header.frame_id = "map";

    // 直接把 ENU 点填进去
    msg.pose.position.x = target.x;
    msg.pose.position.y = target.y;
    msg.pose.position.z = target.z;
    

    // 姿态保持水平，四元数设为 (0,0,0,1)
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;

    setpoint_pub_->publish(msg);

    RCLCPP_INFO(node_->get_logger(),
                "[PublishPoint] 发布 ENU 目标到 /mavros/setpoint_position/local: "
                "x=%.2f, y=%.2f, z=%.2f",
                target.x, target.y, target.z);
}
