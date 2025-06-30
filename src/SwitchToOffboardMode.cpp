#include "uav_monitor_control/SwitchToOffboardMode.hpp"

using namespace uav_monitor_control;

SwitchToOffboardMode::SwitchToOffboardMode(const std::string &name,
                                           const BT::NodeConfiguration &config,
                                           rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config),
      node_(node)
{
    // 创建 set_mode 服务客户端
    client_ = node_->create_client<mavros_msgs::srv::SetMode>(
        "mavros/set_mode");
}

BT::NodeStatus SwitchToOffboardMode::tick()
{
    // 构造请求
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->base_mode = 0;            // 只用 custom_mode
    req->custom_mode = "OFFBOARD"; // 切换到 OFFBOARD

    client_->async_send_request(req);

    RCLCPP_INFO(node_->get_logger(),
                "[SwitchToOffboardMode] 异步发送 set_mode → OFFBOARD");
    return BT::NodeStatus::SUCCESS;
}
