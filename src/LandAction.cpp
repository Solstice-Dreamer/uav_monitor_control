#include "uav_monitor_control/LandAction.hpp"

using namespace uav_monitor_control;

LandAction::LandAction(const std::string &name,
                       const BT::NodeConfiguration &config,
                       rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config),
      node_(node)
{
    // 创建 CommandLong 服务客户端
    cmd_client_ = node_->create_client<mavros_msgs::srv::CommandLong>(
        "/mavros/cmd/command");
}

BT::NodeStatus LandAction::tick()
{
    // 构造请求：MAV_CMD_NAV_LAND = 21
    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    req->broadcast = false;
    req->command = 21;
    req->confirmation = 0;
    // param1..param7 默认 → 在当前位置自动降落

    // 异步发送
    cmd_client_->async_send_request(req);

    RCLCPP_INFO(node_->get_logger(),
                "[LandAction] 发送 NAV_LAND 请求");
    return BT::NodeStatus::SUCCESS;
}
