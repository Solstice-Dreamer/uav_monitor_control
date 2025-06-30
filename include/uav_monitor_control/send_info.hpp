#pragma once
// 防止重复包含

#include <iostream>
#include <vector>
#include <string>
#include <cstring>      // for memset
#include <arpa/inet.h>  // for inet_addr, sockaddr_in (on Linux)
#include <sys/socket.h> // for socket functions
#include <unistd.h>     // for close()

struct UAV_info
{
    float pos[3] = {0, 0, 0};
    float q[4] = {1, 0, 0, 0};
    float battery = 0;
};

// 发送信息（组装字符串并广播）
void send_info(const UAV_info &info, int port);

// int main()
// {
//     UAV_info uav{{0, 0, 1}, {1, 0, 0, 0}, 0.5f};
//     int port = 10016;
//     send_info(uav, port);
//     return 0;
// }