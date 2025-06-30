#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace uav_monitor_control
{

/**
 * @brief 通用“写入黑板”同步节点
 *
 * 模板参数 T 决定 value 的类型。  
 * 构造时注入 ROS2 Node，用于日志输出。
 *
 * 输入端口：
 *   - key   (std::string)：要写入的黑板键
 *   - value (T)：         要写入的值
 */
template <typename T>
class SetBlackboard : public BT::SyncActionNode
{
public:
  SetBlackboard(const std::string &name,
                const BT::NodeConfiguration &config,
                rclcpp::Node::SharedPtr node);

  /// 声明端口
  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

} // namespace uav_monitor_control
