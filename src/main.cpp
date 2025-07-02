#include <behaviortree_cpp_v3/bt_factory.h>                // BehaviorTree.CPP的核心工厂类，用于注册自定义节点类型并加载XML
#include <rclcpp/rclcpp.hpp>                               // ROS 2 C++ 客户端库，提供Node、init、spin等API
#include <ament_index_cpp/get_package_share_directory.hpp> // ament_index用于在运行时查找已安装包的share目录
#include "uav_monitor_control/ListenStatus.hpp"
#include "uav_monitor_control/ListenCommand.hpp"
#include "uav_monitor_control/PublishPoint.hpp"
#include "uav_monitor_control/CheckCommandName.hpp"
#include "uav_monitor_control/SetTakeoffTarget.hpp"
#include "uav_monitor_control/SwitchToOffboardMode.hpp"
#include "uav_monitor_control/ArmDisarm.hpp"
#include "uav_monitor_control/SetPointTarget.hpp"
#include "uav_monitor_control/LandAction.hpp"
#include "uav_monitor_control/CheckLanded.hpp"
#include "uav_monitor_control/SetBlackboard.hpp"
#include "uav_monitor_control/CheckPositionReached.hpp"
#include "uav_monitor_control/ReturnToHome.hpp"
#include "uav_monitor_control/StopFollow.hpp"

int main(int argc, char **argv)
{
    // 1. 初始化ROS 2，必须在任何rclcpp函数调用前执行
    rclcpp::init(argc, argv);

    // 2. 创建一个唯一的ROS节点实例
    //    该节点将被整个行为树的所有叶子节点复用，用于订阅、发布和参数服务
    auto node = std::make_shared<rclcpp::Node>("main_tree");

    // 3. 从参数服务器声明并读取名为"tree_file"的参数
    //    该参数由launch文件通过parameters传入，内容是行为树XML文件的绝对路径
    node->declare_parameter<std::string>("tree_file", ""); // 声明tree_file参数，默认空串
    std::string xml_path;                                  // 用于存放读取到的路径
    node->get_parameter("tree_file", xml_path);            // 获取参数值
    RCLCPP_INFO(node->get_logger(),                        // 在日志中打印加载的XML路径，方便调试
                "加载行为树 XML：%s", xml_path.c_str());

    // 4. 创建BehaviorTreeFactory，用于注册自定义节点类型和构建行为树
    BT::BehaviorTreeFactory factory;

    // 5. 注册CheckBattery节点类型
    //    - 模板参数<uav_monitor_control::CheckBattery>指定节点类
    //    - 第一个参数"CheckBattery"必须与XML中<CheckBattery>标签一致
    //    - 第二个参数是一个Lambda工厂函数，用来创建节点实例
    //      这里将同一个ROS节点指针(node)传入，以便CheckBattery内部使用它进行订阅
    factory.registerBuilder<uav_monitor_control::ListenStatus>(
        "ListenStatus",
        [node](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<uav_monitor_control::ListenStatus>(name, config, node);
        });
    factory.registerBuilder<uav_monitor_control::ListenCommand>(
        "ListenCommand",
        [node](const std::string &name, const BT::NodeConfiguration &config)
        {
            const char *home = std::getenv("HOME");
            std::string folder = std::string(home) + "/communication/commands_xml";
            return std::make_unique<uav_monitor_control::ListenCommand>(
                name, config, folder, node);
        });
    factory.registerBuilder<uav_monitor_control::PublishPoint>(
        "PublishPoint",
        [node](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<uav_monitor_control::PublishPoint>(
                name, config, node);
        });
    factory.registerBuilder<uav_monitor_control::ArmDisarm>(
        "ArmDisarm",
        [node](const std::string &name,
               const BT::NodeConfiguration &config)
        {
            return std::make_unique<uav_monitor_control::ArmDisarm>(
                name, config, node);
        });
    factory.registerBuilder<uav_monitor_control::CheckLanded>(
        "CheckLanded",
        [node](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<uav_monitor_control::CheckLanded>(name, config, node);
        });

    factory.registerBuilder<uav_monitor_control::SetPointTarget>(
        "SetPointTarget",
        [node](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<uav_monitor_control::SetPointTarget>(name, config, node);
        });
    factory.registerNodeType<uav_monitor_control::CheckCommandName>("CheckCommandName");

    factory.registerBuilder<uav_monitor_control::SetTakeoffTarget>(
        "SetTakeoffTarget",
        [node](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<uav_monitor_control::SetTakeoffTarget>(name, config, node);
        });
    factory.registerBuilder<uav_monitor_control::SwitchToOffboardMode>(
        "SwitchToOffboardMode",
        [node](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<uav_monitor_control::SwitchToOffboardMode>(name, config, node);
        });
    factory.registerBuilder<uav_monitor_control::LandAction>(
        "LandAction",
        [node](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<uav_monitor_control::LandAction>(name, config, node);
        });
    factory.registerBuilder<uav_monitor_control::SetBlackboard<geometry_msgs::msg::Point>>(
        "SetBlackboardPose",
        [node](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<uav_monitor_control::SetBlackboard<geometry_msgs::msg::Point>>(name, config, node);
        });

    factory.registerBuilder<uav_monitor_control::CheckPositionReached>(
        "CheckPositionReached",
        [node](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<uav_monitor_control::CheckPositionReached>(name, config, node);
        });
    factory.registerBuilder<uav_monitor_control::ReturnToHome>(
        "ReturnToHome",
        [node](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<uav_monitor_control::ReturnToHome>(name, config, node);
        });
    factory.registerBuilder<uav_monitor_control::StopFollow>(
        "StopFollow",
        [node](const std::string &name, const BT::NodeConfiguration &config)
        {
            return std::make_unique<uav_monitor_control::StopFollow>(name, config, node);
        });

    // 6. 从XML文件创建行为树实例
    //    工厂会根据XML中定义的结构，构造整棵树
    auto tree = factory.createTreeFromFile(xml_path);

    // 7. 主循环：以固定频率依次执行以下操作
    //    a) tree.tickRoot() 驱动整个行为树“打点”，执行各叶子节点的tick()
    //    b) rclcpp::spin_some(node) 让该ROS节点处理一次订阅/服务/计时器回调
    //    c) rate.sleep() 控制循环频率为1Hz
    rclcpp::Rate rate(2); // 设置循环频率为10Hz
    while (rclcpp::ok())
    {
        tree.tickRoot();         // 整棵行为树打点一次
        rclcpp::spin_some(node); // 处理ROS回调（例如电量订阅、服务应答等）
        rate.sleep();            // 等待到下一次循环时刻
    }

    // 清理并退出ROS 2
    rclcpp::shutdown();
    return 0;
}
