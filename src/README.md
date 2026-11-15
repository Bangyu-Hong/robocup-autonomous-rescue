# 源码与模块导航

返回[项目主页](../README.md)。

## 已保存的底盘工程

`stm32f407-chassis/` 来自 RoboCup 本地资料 `代码/下位机/control（8M）`，包含 STM32F407 业务源码、HAL / CMSIS、STM32CubeMX 配置、Keil 工程与原有技术说明。

| 入口 | 内容 |
|---|---|
| [main.c](./stm32f407-chassis/Core/Src/main.c) | 初始化、麦轮运动学、IMU 帧解析、里程计、串口命令与超时处理 |
| [stm32f4xx_it.c](./stm32f407-chassis/Core/Src/stm32f4xx_it.c) | 中断处理、轮速反馈与闭环执行 |
| [tim.c](./stm32f407-chassis/Core/Src/tim.c) | 编码器、PWM 和定时器配置 |
| [usart.c](./stm32f407-chassis/Core/Src/usart.c) | 串口与 DMA 配置 |
| [Keil 工程](./stm32f407-chassis/MDK-ARM/test1.uvprojx) | MDK 工程入口 |
| [CubeMX 配置](./stm32f407-chassis/test1.ioc) | 硬件外设配置 |
| [原理图](./stm32f407-chassis/STM32F407VE_核心板_原理图_V5.5.pdf) | 保存的核心板原理图 |
| [通信协议](./stm32f407-chassis/PROTOCOL_GUIDE.md) | 速度指令、里程计与通信示例 |
| [IMU 说明](./stm32f407-chassis/IMU_GUIDE.md) | 姿态数据解析及标定说明 |

## 整理范围

- 保持收录源码、厂商文件和工程配置的原始内容。
- 本地 `project` 副本中的业务源码与主目录一致，不重复发布。
- `compare/control（8M）` 的 `main.c` 存在轮位映射及角速度计算差异，单独归档在 [archive](../archive/README.md)。
- 不收录编译输出、IDE 个人状态、自动工作流模板或无关报名材料。
- 不将「神思智飞」目录中的 ROS / 视觉代码混入本仓库。

## 与报告的对应关系

[技术报告](../技术文档.pdf)描述 RA6M5 底盘及 RK3588 / ESP32 系统，其中 RA6M5 主程序片段保存在报告附录。实际底盘已由项目本人确认为 STM32F407，与当前完整工程一致。报告中的 RA6M5 描述作为原始资料保留，不替代实际底盘说明；对应 ROS 上位机及 ESP32 工程尚未收录。

详见[环境与版本说明](../docs/SETUP.md)。
