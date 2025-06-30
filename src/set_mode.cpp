#include "uav_monitor_control/set_mode.hpp"
#include <unordered_map>

using namespace uav_monitor_control;

// 将常用模式名映射到 MAV_CMD_DO_SET_MODE 的 param2 值
static const std::unordered_map<std::string, float> MODE_MAP = {
  {"MANUAL",    1.0f},
  {"ALTCTL",    2.0f},
  {"POSCTL",    3.0f},
  {"OFFBOARD",  6.0f},
  {"AUTO.RTL",  5.0f},
  // 如果需要，还可以增加其它模式
};

SetMode::SetMode(const std::string &name,
                 const BT::NodeConfiguration &config,
                 rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config)
  , node_(node)
{
  // 在 "/fmu/in/vehicle_command" 话题上创建 publisher，队列深度 10
  cmd_pub_ = node_->create_publisher<px4_msgs::msg::VehicleCommand>(
    "/fmu/in/vehicle_command", 10);
}

BT::NodeStatus SetMode::tick()
{
  // 1. 从 XML 端口读取 mode 字符串
  std::string mode;
  if (!getInput<std::string>("mode", mode)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[SetMode] 未传入模式参数");
    return BT::NodeStatus::FAILURE;
  }

  // 2. 查表获取对应的数值
  auto it = MODE_MAP.find(mode);
  if (it == MODE_MAP.end()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "[SetMode] 不支持的模式：%s", mode.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 3. 构造 VehicleCommand 消息
  px4_msgs::msg::VehicleCommand cmd;
  cmd.timestamp        = node_->get_clock()->now().nanoseconds() / 1000;
  cmd.command          = 176;            // MAV_CMD_DO_SET_MODE
  cmd.param1           = 1.0f;           // 保留字段，一般设 1
  cmd.param2           = it->second;     // 模式编号
  cmd.target_system    = 1;              // 默认系统 ID
  cmd.target_component = 1;              // 默认组件 ID
  cmd.source_system    = 1;
  cmd.source_component = 1;
  cmd.from_external    = true;           // 标记这是外部指令

  // 4. 发布命令
  cmd_pub_->publish(cmd);
  RCLCPP_INFO(node_->get_logger(),
              "[SetMode] 已发布切换模式：%s", mode.c_str());

  // 5. 返回 SUCCESS，表示动作完成
  return BT::NodeStatus::SUCCESS;
}
