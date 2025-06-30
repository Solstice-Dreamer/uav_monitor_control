#pragma once
// 防止重复包含

#include <behaviortree_cpp_v3/action_node.h>     // BehaviorTree.CPP 同步动作基类
#include <rclcpp/rclcpp.hpp>                     // ROS2 C++ 客户端库
#include <px4_msgs/msg/vehicle_command.hpp>      // PX4 飞控命令消息

namespace uav_monitor_control
{

/**
 * @brief SetMode：切换飞控模式的同步动作节点
 *
 * 在 tick() 被调用时，它会向 /fmu/in/vehicle_command 发布一条
 * MAV_CMD_DO_SET_MODE（176）指令，将飞控切换到指定模式。
 */
class SetMode : public BT::SyncActionNode
{
public:
  /**
   * @param name   节点在行为树中的实例名称
   * @param config 行为树端口等配置
   * @param node   共享的 ROS2 节点指针，用于创建 publisher
   */
  SetMode(const std::string &name,
          const BT::NodeConfiguration &config,
          rclcpp::Node::SharedPtr node);

  // 声明一个 string 类型的输入端口 "mode"
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("mode", "要切换到的飞行模式名称，例如 OFFBOARD")
    };
  }

  // tick() 一调用就发布一次切模式命令，然后返回 SUCCESS
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr                                    node_;    // 共享ROS节点
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_; // 发布器
};

}  // namespace uav_monitor_control
