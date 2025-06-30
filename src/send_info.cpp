#include "uav_monitor_control/send_info.hpp"

// 发送信息（组装字符串并广播）
void send_info(const UAV_info& info, int port) {
    std::string head = "state";
    std::string message = head + " " +
                          std::to_string(info.pos[0]) + " " + std::to_string(info.pos[1]) + " " + std::to_string(info.pos[2]) + " " +
                          std::to_string(info.q[0]) + " " + std::to_string(info.q[1]) + " " + std::to_string(info.q[2]) + " " +
                          std::to_string(info.q[3]) + " " +
                          std::to_string(info.battery);

    // 发送广播
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        std::cerr << "创建UDP socket失败！\n";
        return;
    }

    // 设置 socket 为可广播
    int broadcastEnable = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char *)&broadcastEnable, sizeof(broadcastEnable)) < 0) {
        std::cerr << "设置广播选项失败！\n";
        close(sock);
        return;
    }

    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    inet_pton(AF_INET, "192.168.1.255", &(addr.sin_addr));
    // addr.sin_addr.s_addr = inet_addr("10.101.121.255"); // 通信地址地址

    ssize_t sent = sendto(sock, message.c_str(), message.size(), 0,
                          reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    if (sent < 0) {
        std::cerr << "广播发送失败！\n";
    } else {
        std::cout << "已广播指令 \"" << message << "\" 到端口 " << port << "\n";
    }

    close(sock);
}