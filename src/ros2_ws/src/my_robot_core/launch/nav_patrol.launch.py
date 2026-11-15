import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 找到各个包的绝对路径
    my_robot_core_dir = get_package_share_directory('my_robot_core')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup') # 系统大佬的包

    # 2. 指定你的配置文件和【已经建好的地图】路径
    mppi_params_file = os.path.join(my_robot_core_dir, 'config', 'nav2_mppi_params.yaml')
    
    # ⚠️ 注意：这里写死了你保存的地图路径。确保该目录下有这两个文件：my_room.yaml 和 my_room.pgm
    map_file = '/userdata/ros2_ws/maps/my_room.yaml'

    # ========================================================
    # 3. 开始召唤 (纯硬件 + 完整版 Nav2)
    # ========================================================

    # [终端 1]: 启动躯体与感觉 (调用 hardware_bringup.launch.py，绝对不能带 SLAM！)
    hardware_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_core_dir, 'launch', 'hardware_bringup.launch.py')
        )
    )

    # [终端 2]: 启动完整版领航员 Nav2 (调用 bringup_launch.py，自带 AMCL 定位和地图服务器)
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'autostart': 'True',
            'map': map_file,               # 👈 把地图喂给它
            'params_file': mppi_params_file # 👈 把 MPPI 避障参数喂给它
        }.items()
    )

    # 4. 打包发送给系统，点火！
    return LaunchDescription([
        hardware_cmd,
        nav2_cmd
    ])
