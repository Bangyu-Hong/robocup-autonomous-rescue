# 历史 STM32F407 版本与对比

主页主要代码入口已调整为材料更完整的 RA6M5 平台，原 STM32F407 工程仍保留在 [stm32f407-chassis](./stm32f407-chassis/)，对应 RoboCup 本地 `代码/下位机/control（8M）`。原架构说明图保留为 [stm32f407-architecture.svg](./stm32f407-architecture.svg)。

本地 `project` 副本中的业务源码与 `control（8M）` 一致，不重复发布。`compare/control（8M）` 的差异文件保留为 [compare-main.c](./compare-main.c)，原文本对比见 [main-variants.diff](./main-variants.diff)。

| 项目 | 保存主工程 | compare 历史文件 |
|---|---|---|
| 轮位映射 | A=LF、B=RF、C=LR、D=RR | A=LF、B=LR、C=RF、D=RR |
| 轮速读取和目标分配 | 按主目录通道映射 | 按历史通道映射 |
| 里程计角速度分母 | `2 × (WHEEL_BASE + WHEEL_TRACK)` | `4 × (WHEEL_BASE + WHEEL_TRACK)` |

此归档用于追溯已有版本。两个 MCU 平台的工程独立保留，不能直接替换固件、引脚或控制参数。
