import os
from glob import glob
from setuptools import find_packages, setup  # 导入 Python 官方的打包工具

package_name = 'my_robot_core'  # 定义你的 ROS 2 功能包名字

setup(
    name=package_name,          # 包名
    version='0.0.0',            # 版本号
    
    # 【核心1：寻找 Python 代码】
    # 自动在 my_robot_core 文件夹下寻找所有的 .py 源码文件 (排除了 test 测试文件夹)
    packages=find_packages(exclude=['test']), 
    
    # 【核心2：搬运非 Python 文件】 (这里极其容易踩坑！)
    # data_files 负责把不需要编译的文件（比如配置文件、launch文件）原封不动地搬运到 install 目录。
    data_files=[
        # 这句话是 ROS 2 的底层暗号，告诉系统“我是一个合法的 ROS 2 功能包”，必须留着。
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # 把 package.xml 文件搬运到 share/my_robot_core/ 目录下。
        ('share/' + package_name, ['package.xml']),
        
        # ⚠️ 新增下面这两行，把 launch 和 config 文件夹安装到系统里
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    
    # 下面这些是基础的软件登记信息，随便填，不影响运行
    install_requires=['setuptools'], # 依赖的打包工具
    zip_safe=True,                   # 是否允许以 zip 形式运行（ROS 2 通常为 True）
    maintainer='elf',                # 维护者名字
    maintainer_email='elf@todo.todo',# 维护者邮箱
    description='TODO: Package description', # 描述
    license='TODO: License declaration',     # 开原协议
    
    # 测试相关的依赖库
    extras_require={
        'test': [
            'pytest',
        ],
    },
    
    # 【核心3：定义可执行文件指令 (相当于 ROS 1 的 CMake 配置)】
    # 当你在终端敲 `ros2 run my_robot_core base_driver` 时，系统就是来这里找对应关系的。
    entry_points={
        'console_scripts': [
            # 格式：'终端命令名 = 文件夹名.脚本文件名:脚本里的入口函数名'
            'main_control = my_robot_core.main_control:main',
            'base_driver = my_robot_core.base_driver:main', # 新增这一行：将 base_driver.py 里的 main() 函数映射为 base_driver 命令
        ],
    },
)