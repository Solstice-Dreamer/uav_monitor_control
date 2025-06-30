#include "uav_monitor_control/ArmDisarm.hpp"

using namespace uav_monitor_control;

ArmDisarm::ArmDisarm(const std::string &name,
                     const BT::NodeConfiguration &config,
                     rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config),
      node_(node)
{
    // 创建 arming 服务客户端
    client_ = node_->create_client<mavros_msgs::srv::CommandBool>(
        "mavros/cmd/arming");
}

BT::NodeStatus ArmDisarm::tick()
{
    bool arm_flag = true;
    getInput("arm", arm_flag);

    // 构造请求
    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = arm_flag;

    // 异步发起，不等待
    client_->async_send_request(req);

    RCLCPP_INFO(node_->get_logger(),
                "[ArmDisarm] 异步%s 请求",
                arm_flag ? "Arm" : "Disarm");
    return BT::NodeStatus::SUCCESS;
}
