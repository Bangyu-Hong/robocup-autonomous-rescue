# RoboCup ROS 1 环境建图机器人

**ROS 1 · Gmapping · STM32F407 · RPLIDAR · RViz**

团队移动机器人项目，主要内容是室内二维环境建图。机器人移动时采集激光扫描与底盘运动信息，由 Gmapping 生成占据栅格地图，并通过 RViz 查看地图与传感器数据。

[场地运行视频](./演示视频.mp4) · [建图技术文档](./技术文档.pdf) · [源码导航](./src/README.md) · [代码压缩包](./重点代码压缩包.zip)

## 1. 系统组成

| 模块 | 工作内容 |
|---|---|
| ROS 1 上位机 | 连接雷达与底盘，组织里程计、TF 和建图节点 |
| RPLIDAR | 发布 `/scan` 二维激光扫描数据，工程采用 C1 驱动入口 |
| STM32F407 | 麦轮控制、编码器测速、IMU 数据处理与里程计计算 |
| Gmapping | 结合激光扫描与机器人运动信息生成二维栅格地图 |
| RViz / map_server | 查看地图和传感器数据，导出地图文件 |

![ROS 1 建图与底盘接口](./images/system_architecture.svg)

```text
RPLIDAR ── /scan ───────────────┐
                               ▼
编码器 / IMU → STM32F407 → ROS 1 底盘驱动
                         /odom + TF
                               │
                               ▼
                           Gmapping
                           /map + TF
                               │
                         ┌─────┴─────┐
                         ▼           ▼
                       RViz       地图保存
```

## 2. 工程内容

**雷达与建图接入。** 配置 RPLIDAR、Gmapping 和底盘启动项，连接 `/scan`、里程计及 TF，形成建图数据链路。

**底盘与运动反馈。** STM32F407 处理麦轮运动学、编码器反馈与 IMU 航向信息，通过串口与 ROS 1 交换速度指令和里程计数据。

**地图查看与保存。** 使用 RViz 检查 `/map`、`/scan` 和坐标关系，通过 `map_server` 导出 `.pgm` 与 `.yaml` 地图文件。

ROS 1 工作空间复用自团队「神思智飞」工程，底盘工程为 STM32F407 版本。

## 3. 关键代码

| 内容 | 入口 |
|---|---|
| 统一建图启动 | [mapping.launch](./src/ros1_ws/src/shensi_driver/launch/mapping.launch) |
| Gmapping 参数 | [my_gmapping.launch](./src/ros1_ws/src/shensi_driver/launch/my_gmapping.launch) |
| ROS 底盘串口、里程计与 TF | [base_driver.py](./src/ros1_ws/src/shensi_driver/scripts/base_driver.py) |
| 雷达接入与静态 TF | [robot_bringup.launch](./src/ros1_ws/src/shensi_driver/launch/robot_bringup.launch) |
| RPLIDAR 驱动 | [rplidar_ros](./src/ros1_ws/src/rplidar_ros/) |
| MCU 主程序与运动学 | [main.c](./src/stm32f407-chassis/Core/Src/main.c) |
| 编码器反馈与电机速度闭环 | [stm32f4xx_it.c](./src/stm32f407-chassis/Core/Src/stm32f4xx_it.c) |
| MCU 工程 | [Keil 工程](./src/stm32f407-chassis/MDK-ARM/test1.uvprojx) · [CubeMX 配置](./src/stm32f407-chassis/test1.ioc) |

## 4. 项目展示与参数

### 机器人与场地运行

![复用平台整机照片](./images/shensi_robot.jpg)

整机照片来自「神思智飞」项目报告，展示复用平台的麦轮底盘、分层结构与传感器布置。

| 机器人近景 | 场地运行 |
|---|---|
| ![视频开始时的机器人](./images/robot_overview.jpg) | ![视频中的场地运行](./images/exploration_scene.jpg) |

场地视频记录机器人在箱体和挡板之间的移动过程。

### RViz 地图界面

!{RViz 地图界面](./images/shensi_mapping.png)

图源为复用平台的「神思智飞」项目报告，展示 RViz 中的地图与传感器界面。

### Gmapping 配置

| 参数 | 工程值 | 含义 |
|---|---|---|
| `delta` | `0.02 m` | 地图栅格尺寸 |
| `particles` | `30` | 粒子数 |
| `map_update_interval` | `1.0 s` | 地图发布更新间隔 |
| `linearUpdate` | `0.1 m` | 平移更新阈值 |
| `angularUpdate` | `0.1 rad` | 转动更新阈值 |

参数来自 [my_gmapping.launch](./src/ros1_ws/src/shensi_driver/launch/my_gmapping.launch)，用于说明建图设置。

## 5. 环境与运行

工程使用 Ubuntu 20.04 / ROS Noetic，主入口为 `mapping.launch`。构建、设备配置、启动命令和地图保存方法见[环境与运行说明](./src/docs/SETUP.md)。

目录：`src/ros1_ws` 为 ROS 1 工作空间，`src/stm32f407-chassis` 为底盘工程，`src/docs` 为运行说明，`src/archive` 为底盘历史版本对比。

## 6. 第三方组件

工程使用 ROS 1、Gmapping、RViz、`rplidar_ros`、STM32 HAL 与 CMSIS。
