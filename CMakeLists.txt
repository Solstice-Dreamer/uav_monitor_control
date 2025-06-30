# -------------------------------------------------
# 1. 指定 CMake 的最低版本要求
#    ROS 2 推荐至少使用 3.5 版本
# -------------------------------------------------
cmake_minimum_required(VERSION 3.5)

# -------------------------------------------------
# 2. 定义工程名称
#    与 package.xml 中的 <name> 保持一致
# -------------------------------------------------
project(uav_monitor_control)

# -------------------------------------------------
# 3. 查找依赖包
# -------------------------------------------------
# ament 和 ROS2 C++ 库
find_package(ament_cmake          REQUIRED)
find_package(rclcpp              REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# MAVROS 的消息／服务定义包
find_package(mavros_msgs   REQUIRED)

# MAVROS 里会用到的标准消息类型
# find_package(std_msgs      REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs   REQUIRED)

# -------------------------------------------------
# 4. 包含头文件目录
#    告诉编译器在 include/ 目录下查找 .hpp 文件
# -------------------------------------------------
include_directories(
  include
)

# -------------------------------------------------
# 5. 编译自定义的 BT 节点库 monitor_control_nodes
#    将 src/check_battery.cpp 编译成一个库，以供 bt_runner 链接使用
# -------------------------------------------------
add_library(monitor_control_nodes
  src/ListenStatus.cpp
  src/send_info.cpp
  src/ListenCommand.cpp
  src/PublishPoint.cpp
  src/CheckCommandName.cpp
  src/SetTakeoffTarget.cpp
  src/SwitchToOffboardMode.cpp
  src/ArmDisarm.cpp
  src/SetPointTarget.cpp
  src/LandAction.cpp
  src/CheckLanded.cpp
  src/CheckPositionReached.cpp
  src/SetBlackboard.cpp
  src/ReturnToHome.cpp
)
# 为该库添加依赖，自动设置 include 路径和链接库
ament_target_dependencies(monitor_control_nodes
  rclcpp                   # 提供 ROS 2 Node、Subscription 等接口
  mavros_msgs                 # 提供 BatteryStatus 消息类型
  behaviortree_cpp_v3      # 提供 BehaviorTree 演算法框架
  std_msgs
  geometry_msgs
  sensor_msgs
)

# -------------------------------------------------
# 6. 编译可执行文件 bt_runner
#    该可执行负责加载 XML，创建行为树，执行打点循环
# -------------------------------------------------
add_executable(bt_runner
  src/main.cpp
)
# 为 bt_runner 添加依赖，自动链接 rclcpp 和 behaviortree_cpp_v3
ament_target_dependencies(bt_runner
  rclcpp
  behaviortree_cpp_v3
)
# 将我们刚才编译的 BT 节点库链接到 bt_runner 上
target_link_libraries(bt_runner
  monitor_control_nodes
)

# -------------------------------------------------
# 7. 安装规则（install）
#    构建完成后，文件将被安装到 install/ 目录下，
#    以便 `ros2 launch` 等工具能够找到它们
# -------------------------------------------------
# 安装库和可执行文件到 lib/<project_name> 下
install(TARGETS
  monitor_control_nodes  # 自定义库
  bt_runner               # 可执行程序
  DESTINATION lib/${PROJECT_NAME}
)

# 安装资源目录（XML、launch 脚本）到 share/<project_name> 下
install(
  DIRECTORY
    trees   # 行为树 XML 定义
    launch  # ROS2 launch 脚本
  DESTINATION share/${PROJECT_NAME}
)

# -------------------------------------------------
# 8. ament_package()
#    ROS 2 下使用 ament_cmake 构建系统时必须调用，
#    用于生成包的元数据和导出依赖信息
# -------------------------------------------------
ament_package()
