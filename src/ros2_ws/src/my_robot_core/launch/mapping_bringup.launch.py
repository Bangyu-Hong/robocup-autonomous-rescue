import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 获取各个功能包的绝对路径
    my_robot_core_dir = get_package_share_directory('my_robot_core')
    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')

    # 2. 定位咱们刚才偷出来的 SLAM 配置文件
    slam_params_file = os.path.join(my_robot_core_dir, 'config', 'my_slam_params.yaml')

    # ==========================================
    # 3. 定义要启动的四大金刚 (节点与 Launch)
    # ==========================================

    # 3.1 底盘驱动节点 (双腿)
    base_driver_node = Node(
        package='my_robot_core',
        executable='base_driver',  # 对应你在 setup.py 中设置的执行文件名
        name='base_driver',
        output='screen'
    )

    # 3.2 思岚激光雷达 (眼睛) - 直接调用官方 Launch 并传入绑定的 Udev 端口
    sllidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_ros2_dir, 'launch', 'sllidar_c1_launch.py')
        ),
        launch_arguments={'serial_port': '/dev/rplidar'}.items()
    )

    # 3.3 静态 TF 发布器 (骨架) 
    # 安装位置：X向前0.1m, Y居中0m, Z离地0.15m, Yaw反转180度(3.14159弧度)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--x', '0.075', 
            '--y', '0.0', 
            '--z', '0.15', 
            '--yaw', '3.14159', 
            '--pitch', '0.0', 
            '--roll', '0.0', 
            '--frame-id', 'base_footprint', 
            '--child-frame-id', 'laser'
        ],
        output='screen'
    )

    # 3.4 SLAM Toolbox 节点 (大脑) - 强行加载我们的自定义 yaml 参数
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    )

    # 4. 把所有东西打包交给 ROS 2 一次性点火启动
    return LaunchDescription([
        base_driver_node,
        sllidar_launch,
        static_tf_node,
        slam_toolbox_node
    ])
