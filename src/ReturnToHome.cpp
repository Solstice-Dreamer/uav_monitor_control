#include "uav_monitor_control/ReturnToHome.hpp"
#include <fstream>
#include <chrono>
#include <filesystem>

namespace uav_monitor_control
{

    ReturnToHome::ReturnToHome(const std::string &name,
                               const BT::NodeConfiguration &config,
                               rclcpp::Node::SharedPtr node)
        : BT::SyncActionNode(name, config), node_(node)
    {
    }

    BT::NodeStatus ReturnToHome::tick()
    {
        geometry_msgs::msg::Point home_point;
        std::map<std::string, std::string> cmd_map;
        geometry_msgs::msg::Point current_point;

        if (!getInput("home_position_port", home_point))
        {
            RCLCPP_WARN(node_->get_logger(), "ReturnToHome: home_position_port not available");
        }
        if (!getInput("cmd_map_port", cmd_map))
        {
            RCLCPP_WARN(node_->get_logger(), "ReturnToHome: cmd_map_port not available");
        }
        if (!getInput("current_position_port", current_point))
        {
            RCLCPP_WARN(node_->get_logger(), "ReturnToHome: current_position_port not available");
        }

        // 解析高度参数
        double alt = 0.0;
        if (cmd_map.count("alt"))
        {
            try
            {
                alt = std::stod(cmd_map.at("alt"));
            }
            catch (const std::exception &)
            {
                RCLCPP_WARN(node_->get_logger(), "ReturnToHome: invalid altitude parameter, using default 0");
            }
        }
        else
        {
            RCLCPP_WARN(node_->get_logger(), "ReturnToHome: altitude parameter missing or cmd_map unavailable, using default 0");
        }

        // 准备文件路径
        auto now = std::chrono::system_clock::now();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        std::string home_dir = std::getenv("HOME");
        std::filesystem::path dir = home_dir + "/communication/path";
        std::filesystem::create_directories(dir);
        std::string points_file = (dir / ("points" + std::to_string(millis) + ".txt")).string();

        // 写入点坐标
        std::ofstream ofs(points_file);
        if (!ofs)
        {
            RCLCPP_ERROR(node_->get_logger(), "ReturnToHome: failed to open points file");
        }
        else
        {
            ofs << current_point.x << " " << current_point.y << " " << alt << "\n"
                << home_point.x << " " << home_point.y << " " << alt << "\n"
                << home_point.x << " " << home_point.y << " " << home_point.z << "\n";
            ofs.close();
        }

        // 更新 reached.sign
        std::string reached_file = (dir / "reached.sign").string();
        std::ofstream ofs2(reached_file, std::ios::trunc);
        if (!ofs2)
        {
            RCLCPP_ERROR(node_->get_logger(), "ReturnToHome: failed to open reached.sign");
        }
        else
        {
            ofs2 << "1";
            ofs2.close();
        }

        RCLCPP_INFO(node_->get_logger(), "ReturnToHome: path process completed");
        return BT::NodeStatus::SUCCESS;
    }

} // namespace uav_monitor_control
