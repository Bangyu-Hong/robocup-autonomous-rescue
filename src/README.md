# 源码与模块导航

返回[项目主页](../README.md)。

| 目录 | 内容 | 入口 |
|---|---|---|
| [ros1_ws](./ros1_ws/) | ROS 1 激光建图工作空间 | [mapping.launch](./ros1_ws/src/shensi_driver/launch/mapping.launch) |
| [shensi_driver](./ros1_ws/src/shensi_driver/) | 底盘串口、里程计、TF、硬件启动与 Gmapping 参数 | [base_driver.py](./ros1_ws/src/shensi_driver/scripts/base_driver.py) |
| [rplidar_ros](./ros1_ws/src/rplidar_ros/) | 第三方 RPLIDAR 雷达驱动 | [C1 启动配置](./ros1_ws/src/rplidar_ros/launch/rplidar_c1.launch) |
| [stm32f407-chassis](./stm32f407-chassis/) | RoboCup 目录中保存的 STM32F407 底盘工程 | [main.c](./stm32f407-chassis/Core/Src/main.c) |
| [docs](./docs/) | 环境、启动与材料说明 | [SETUP.md](./docs/SETUP.md) |
| [archive](./archive/) | 原底盘代码的历史差异 | [版本说明](./archive/README.md) |
