#include "uav_monitor_control/ListenCommand.hpp"
using namespace uav_monitor_control;

ListenCommand::ListenCommand(const std::string &name,
                             const BT::NodeConfiguration &config,
                             const std::string &folder_path,
                             rclcpp::Node::SharedPtr node)
    : BT::SyncActionNode(name, config), node_(node), folder_path_(folder_path)
{
}

BT::NodeStatus ListenCommand::tick()
{
    // 清空上一次的命令
    cmd_map_.clear();

    // 扫描文件夹并处理第一个 XML 文件
    for (const auto &entry : fs::directory_iterator(folder_path_))
    {
        if (!entry.is_regular_file())
            continue;
        auto path = entry.path();
        if (path.extension() == ".xml")
        {
            parseXmlFile(path);
            fs::rename(path, path.string() + ".finish");
            break; // 每次只处理一个新命令
        }
    }

    // 输出整张命令字典到黑板
    setOutput("cmd_map_port", cmd_map_);

    // 日志打印：展示所有键值对
    std::string log;
    for (auto &kv : cmd_map_)
    {
        log += kv.first + "=" + kv.second + " ";
    }
    RCLCPP_INFO(node_->get_logger(),
                "[ListenCommand] cmd_map: %s", log.c_str());

    return BT::NodeStatus::SUCCESS;
}

void ListenCommand::parseXmlFile(const fs::path &file)
{
    XMLDocument doc;
    if (doc.LoadFile(file.string().c_str()) != XML_SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load XML: %s",
                     file.c_str());
        return;
    }

    XMLElement *root = doc.FirstChildElement("Command");
    if (!root)
        return;

    // 名称
    if (auto *eName = root->FirstChildElement("Name"))
    {
        if (auto txt = eName->GetText())
        {
            cmd_map_["Name"] = txt;
        }
    }

    // 参数列表
    if (auto *params = root->FirstChildElement("Parameters"))
    {
        for (XMLElement *child = params->FirstChildElement();
             child; child = child->NextSiblingElement())
        {
            const char *key = child->Name();
            const char *text = child->GetText();
            if (key && text)
            {
                cmd_map_[key] = text;
            }
        }
    }
}
