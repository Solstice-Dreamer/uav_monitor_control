#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <tinyxml2.h>
#include <filesystem>
#include <map>
#include <string>

namespace uav_monitor_control
{

    namespace fs = std::filesystem;
    using namespace tinyxml2;

    class ListenCommand : public BT::SyncActionNode
    {
    public:
        ListenCommand(const std::string &name,
                      const BT::NodeConfiguration &config,
                      const std::string &folder_path,
                      rclcpp::Node::SharedPtr node);

        static BT::PortsList providedPorts()
        {
            return {
                BT::OutputPort<std::map<std::string, std::string>>(
                    "cmd_map_port", "命令及其参数字典")};
        }

        BT::NodeStatus tick() override;

    private:
        void parseXmlFile(const fs::path &file);

        rclcpp::Node::SharedPtr node_;
        std::string folder_path_;
        std::map<std::string, std::string> cmd_map_;
    };

} // namespace uav_monitor_control
