import os
# ROS 2 提供的 ament_index 接口，用于在已安装的 ROS 包中查找 share 目录
from ament_index_python.packages import get_package_share_directory

# launch 核心接口，用于构造启动描述
from launch import LaunchDescription
# launch_ros 提供的 Node 动作，用于启动一个 ROS 2 节点
from launch_ros.actions import Node

def generate_launch_description():
    # """
    # generate_launch_description() 是 ROS 2 launch 文件的约定函数，
    # 返回一个 LaunchDescription 对象，描述了这个启动脚本中要执行哪些动作。
    # """

    # -------------------------------------------------------------------
    # 1. 获取 uav_monitor_control 包的安装路径
    # -------------------------------------------------------------------
    # get_package_share_directory 会根据包名查到它在 install/share/<pkg> 下的路径，
    # 这样无论你的 workspace 或者系统如何安装，都能正确定位到 share 目录。
    pkg_dir = get_package_share_directory('uav_monitor_control')

    # -------------------------------------------------------------------
    # 2. 拼接行为树 XML 的绝对路径
    # -------------------------------------------------------------------
    # os.path.join 跨平台地拼接路径，最终会得到类似：
    # "/home/jetson/ros2_ws/install/uav_monitor_control/share/uav_monitor_control/trees/battery_monitor.xml"
    xml_file = os.path.join(pkg_dir, 'trees', 'main_tree.xml')

    # -------------------------------------------------------------------
    # 3. 用 LaunchDescription 列表注册所有要执行的动作
    # -------------------------------------------------------------------
    # 目前这里只有一个 Node 动作，用来启动 bt_runner 可执行文件（你的行为树 Runner）。
    return LaunchDescription([

        # ---------------------------------------------------------------
        # Node 动作：启动 bt_runner 节点
        # ---------------------------------------------------------------
        Node(
            # ROS 包名：告诉 launch 去哪个包里找可执行文件
            package='uav_monitor_control',

            # 可执行文件名：和 CMakeLists.txt 里 add_executable(bt_runner) 一致
            executable='bt_runner',

            # 节点名：在 ROS graph 中显示的名字，方便在 rqt_graph 等工具中识别
            name='main_tree',

            # 日志输出到屏幕（screen）而不是日志文件
            output='screen',

            # 给节点传入参数：把 behavior tree XML 的路径传到节点的 parameter server
            # 这样在节点里可以通过 declare_parameter / get_parameter 读取到 tree_file
            parameters=[{'tree_file': xml_file}],
        )
    ])
