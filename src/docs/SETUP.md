# ROS 1 环境建图：环境与运行

返回[项目主页](../../README.md)。

## 项目范围与来源

项目用途为二维环境建图，使用 ROS 1 和 STM32F407 底盘。项目本人已确认复用「神思智飞」目录中的 ROS 1 工作空间，本仓库仅整理建图链路。

原工程资料使用 Ubuntu 20.04 / ROS Noetic。实际主机环境、设备映射和安装外参以所用硬件为准；本次未在 Linux / ROS 环境中执行构建。

## 环境与构建

主要依赖为 `rospy`、`tf`、`geometry_msgs`、`nav_msgs`、`diagnostic_msgs`、pyserial、Gmapping、RViz、map_server 及雷达驱动依赖。按照所用 ROS 1 环境安装各包声明的依赖。

在仓库根目录：

```bash
source /opt/ros/noetic/setup.bash
cd src/ros1_ws
chmod +x src/shensi_driver/scripts/base_driver.py
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## 启动建图

```bash
roslaunch shensi_driver mapping.launch \
  base_port:=/dev/stm32_base \
  lidar_port:=/dev/rplidar \
  use_rviz:=true
```

新增入口只汇总原底盘与雷达启动项、Gmapping 和可选 RViz：

- [mapping.launch](../ros1_ws/src/shensi_driver/launch/mapping.launch)
- [robot_bringup.launch](../ros1_ws/src/shensi_driver/launch/robot_bringup.launch)
- [my_gmapping.launch](../ros1_ws/src/shensi_driver/launch/my_gmapping.launch)

机器人移动过程中，由原有控制方式提供底盘移动指令，建图节点订阅激光扫描并通过 TF 获取运动信息。当前入口不替代实际硬件接线、传感器标定或移动控制配置。

## 数据与坐标关系

| 数据 | 来源 | 用途 |
|---|---|---|
| `/scan` | `rplidar_ros` | 二维激光扫描 |
| `/odom` | `base_driver.py` | 底盘位姿和速度 |
| `odom → base_footprint` | 底盘驱动 TF | 里程计坐标关系 |
| `base_footprint → base_link → laser` | 启动文件静态 TF | 底盘与雷达安装关系 |
| `map → odom`、`/map` | Gmapping | 地图坐标关系与栅格地图 |

保存配置中的底盘波特率为 `115200`，雷达 C1 配置为 `460800`。默认设备为 `/dev/stm32_base` 和 `/dev/rplidar`，必须与实际设备权限和串口映射匹配。

静态 TF 中的雷达安装位置是保存的工程参数，运行前按实物测量与校准。地图配置 `delta=0.02` 表示栅格尺寸，不表示实测误差为 2 cm。

## 保存地图

在已加载工作空间环境的另一个终端：

```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/robocup
```

记录对应的 `.pgm`、`.yaml`、运行参数及场地条件，便于后续复现和评价地图质量。

## STM32 工程

打开[Keil 工程](../stm32f407-chassis/MDK-ARM/test1.uvprojx)，外设配置见 [test1.ioc](../stm32f407-chassis/test1.ioc)。原[系统说明](../stm32f407-chassis/SYSTEM_GUIDE.md)与[通信协议](../stm32f407-chassis/PROTOCOL_GUIDE.md)随源码保存。

原底盘通信指令为 `$vx,vy,wz`，反馈为 `$x,y,th,vx,vy,wz`，以换行结束。按实际晶振、四轮通道、编码器参数、IMU 和调试输出模式核对。主工程 A/B/C/D 对应左前、右前、左后、右后；历史文件的 B/C 映射和角速度计算不同，详见[对比说明](../archive/README.md)。

## 材料与验证

根目录技术 PDF 已重写为当前 ROS 1 建图说明，视频保持原始内容。此前混入的其他平台资料移出当前发布目录，原文件保留在本地备份及 Git 历史中。

本次核对源码复制一致性、Python 语法、启动文件 XML、节点及文件引用、文档链接和压缩包内容。新增的建图入口尚未进行 ROS 实际启动、Keil 构建或实机复测。
