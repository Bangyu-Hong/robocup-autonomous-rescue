import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 找到各个包的绝对路径
    my_robot_core_dir = get_package_share_directory('my_robot_core')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup') # 系统大佬的包
    # explore_lite_dir = get_package_share_directory('explore_lite') # 探索插件的包

    # 2. 找到我们自己写的 MPPI 配置文件
    mppi_params_file = os.path.join(my_robot_core_dir, 'config', 'nav2_mppi_params.yaml')

    # ========================================================
    # 3. 开始召唤四大金刚 (相当于把你敲的 4 个终端命令写成代码)
    # ========================================================

    # [终端 1]: 启动躯体与大脑 (调用我们自己写的 mapping_bringup.launch.py)
    mapping_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(my_robot_core_dir, 'launch', 'mapping_bringup.launch.py')
        )
    )

    # [终端 2]: 启动领航员 Nav2 (调用系统的 navigation_launch.py，坚决不带 AMCL)
    # 并强制它吃下我们写的 MPPI 参数！
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': mppi_params_file
        }.items()
    )

    # [终端 4]: 启动自动探索插件
    # explore_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(explore_lite_dir, 'launch', 'explore.launch.py')
    #     )
    # )

    # 4. 打包发送给系统，同时点火！
    return LaunchDescription([
        mapping_cmd,
        nav2_cmd,
        # explore_cmd
    ])
