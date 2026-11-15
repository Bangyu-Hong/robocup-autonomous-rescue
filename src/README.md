# 源码与模块导航

返回[项目主页](../README.md)。

## 主要展示版本：RA6M5 移动操作平台

| 目录 | 内容 | 入口 |
|---|---|---|
| [ra6m5_base](./ra6m5_base/) | RA6M5 FSP / Keil 底盘源码、外设配置与厂商支持文件 | [hal_entry.c](./ra6m5_base/src/hal_entry.c) |
| [my_robot_core](./ros2_ws/src/my_robot_core/) | ROS 2 串口、里程计、TF、任务框架和导航配置 | [base_driver.py](./ros2_ws/src/my_robot_core/my_robot_core/base_driver.py) |
| [robot_vision](./ros2_ws/src/robot_vision/) | RKNN 球体识别与 RGB-D 三维定位 | [npu_3d_tracker.py](./ros2_ws/src/robot_vision/robot_vision/npu_3d_tracker.py) |
| [jibot1_driver](./ros2_ws/src/jibot1_driver/) | 机械臂 ROS 2 驱动、运动学和拾取接口 | [pick_place_node.py](./ros2_ws/src/jibot1_driver/jibot1_driver/pick_place_node.py) |
| [esp32_arm](./esp32_arm/) | J1 厂商 MicroPython 参考程序 | [main.py](./esp32_arm/main.py) |
| [OrbbecSDK_ROS2-master](./ros2_ws/src/OrbbecSDK_ROS2-master/) | 第三方相机驱动 | [厂商说明](./ros2_ws/src/OrbbecSDK_ROS2-master/README_CN.MD) |
| [sllidar_ros2](./ros2_ws/src/sllidar_ros2/) | 第三方激光雷达驱动 | [厂商说明](./ros2_ws/src/sllidar_ros2/README.md) |

上述工程来自“电赛邀请赛”归档中的 RA6M5 自动捡球平台。底盘五个核心业务模块与原 `瑞萨代码/Rasc_Core` 文件逐字节核对一致；上位机与机械臂源码保持归档内容。救援技术报告、平台功能和 RoboCup 实机记录分别说明，详见[环境与版本说明](./docs/SETUP.md)。

## 历史 STM32F407 版本

完整工程保留在 [archive/stm32f407-chassis](./archive/stm32f407-chassis/)，来自 `RobotCup/代码/下位机/control（8M）`；轮位映射与历史版本差异见[归档说明](./archive/README.md)。

## 整理范围

保留业务源码、配置、已有许可证和厂商支持文件。未收录构建输出、个人 IDE 状态、自动工作流模板和重复动态库。Orbbec 相机动态库需按架构与版本恢复，详见运行说明。现有模型权重未找到，任务主程序仍为框架；本次不改写业务逻辑。
