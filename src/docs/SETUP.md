# 环境、版本与验证说明

返回[项目主页](../../README.md)。

## 版本与材料对应

| 材料 | 来源及用途 |
|---|---|
| `技术文档.pdf` | RoboCup 原技术认证报告，描述 RK3588 / RA6M5 / ESP32 救援方案 |
| `演示视频.mp4` | RoboCup 原场地视频；项目本人已确认实机底盘为 STM32F407 |
| `src/ra6m5_base` | 电赛邀请赛归档中的 RA6M5 底盘工程，核心业务文件与原 Rasc_Core 核对一致 |
| `src/ros2_ws` | 同归档中的自动捡球平台 ROS 2 工程 |
| `src/esp32_arm` | J1 厂商机械臂 MicroPython 参考程序 |
| `平台参考答辩.pptx` | 原 `瑞萨代码/演示ppt.zip` 为完整 OOXML 演示文稿，保留字节内容并使用 `.pptx` 扩展名；19 页，主题为体育馆服务与捡球 |
| `RA6M5平台技术报告.pdf`、`RA6M5平台作品简介.pdf` | “极视之眼”自动捡球平台原材料 |
| `src/archive/stm32f407-chassis` | RoboCup 目录中原 STM32F407 完整底盘工程 |

按用户选择，以材料更完整的 RA6M5 平台版本作为主页主要代码入口。该选择不更改历史 RoboCup 实机使用 STM32F407 的事实；也不把捡球平台的模型与接口直接认定为已完成救援任务适配。

## RA6M5 环境与构建入口

- FSP 配置：[configuration.xml](../ra6m5_base/configuration.xml)。
- Keil 工程：[Led.uvprojx](../ra6m5_base/Led.uvprojx)。
- 原资料记载 Renesas FSP 6.4.0；实际工具链、器件包、内存布局和引脚以工程为准。
- [项目配置手册](../ra6m5_base/PROJECT_CONFIG.md)、[UART 配置](../ra6m5_base/FSP_ROS_UART_CONFIG.md)和[参数说明](../ra6m5_base/PARAMS_README.md)保留原内容，接线须与生成配置及实物核对。

代码设定 `CONTROL_PERIOD=0.01`，每两个控制节拍回传一次里程计；超时计数达到 50 后清零目标并停止电机输出。这些是源码中的调度设定，不是本次实际计时测试结果。

## ROS 2 环境与运行

保存工作空间使用 `rclpy`、`ament_python`，涉及 Nav2、SLAM Toolbox、pyserial、OpenCV、RKNNLite、相机与雷达驱动。ROS 2 发行版和依赖版本需按实际 RK3588 运行环境核对；本次未在 Linux 环境中执行构建。

在仓库根目录，加载匹配的 ROS 2 环境后：

```bash
cd src/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

入口分别保留为：

```bash
ros2 run my_robot_core base_driver
ros2 run robot_vision yolo_3d_node
ros2 launch my_robot_core hardware_bringup.launch.py
```

以上是工程声明的入口，不表示当前所有依赖、设备和模型已配置完毕。

### 串口、模型与相机

- 底盘节点默认串口 `/dev/ttyS9`、波特率 `115200`；按实际映射、权限和 MCU 配置核对。
- 速度命令为 `$vx,vy,wz`；里程计反馈为 `$x,y,yaw,vx,vy,wz`，帧以换行结束。
- 视觉节点引用 `/userdata/yolo_tracking/2.rknn`，当前材料未提供该权重；需补齐兼容模型及 RKNN 运行环境。
- Orbbec 多平台 `.so` 运行库未纳入本次 Git 提交。原库位于源归档的 `ros2_ws/src/OrbbecSDK_ROS2-master/orbbec_camera/SDK/lib/`，包含 1.5.7 版本文件；按实际架构恢复匹配库、头文件及必要链接关系。
- 相机参数、TF 外参、雷达串口与型号均需按实物核对。

## 当前代码限制

1. `main_control.py` 只有节点日志与周期心跳，未实现完整任务状态机。
2. `hardware_bringup.launch.py` 定义了 SLAM 节点，但该节点未加入实际返回的启动列表；其他建图和导航启动文件单独保留。
3. 视觉节点针对球体感知；受困人员、急救包与危险标识识别需要对应模型和场景验证。
4. ESP32 文件为 J1 厂商出厂参考，不能据此认定救援动作已经完成集成验收。
5. 报告训练图标记为 `goal`，缺少数据集、权重及原始评估明细，不将曲线扩展为救援目标精度或板端耗时。

## 历史 STM32 版本

[历史 Keil 工程](../archive/stm32f407-chassis/MDK-ARM/test1.uvprojx)、[CubeMX 配置](../archive/stm32f407-chassis/test1.ioc)及[通信说明](../archive/stm32f407-chassis/PROTOCOL_GUIDE.md)保留原样。主目录轮位为 A=左前、B=右前、C=左后、D=右后；历史 compare 文件的 B/C 映射及角速度计算不同，详见[归档说明](../archive/README.md)。

## 本次验证范围

验证材料复制一致性、源码哈希、本地链接、PPTX 结构、Python 语法和 Keil 源文件引用。未执行 ROS 2 / Keil 构建、模型推理或实机复测；完赛率、建图误差和救援任务成功率尚无完整原始统计。
