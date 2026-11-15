# STM32 底盘历史版本对比

当前主工程为 [stm32f407-chassis](../stm32f407-chassis/)，来自 RoboCup 原 `control（8M）` 目录。原 `project` 副本的业务文件与其一致，未重复收录。

`compare/control（8M）` 中的差异文件保存为 [compare-main.c](./compare-main.c)，文本对比见 [main-variants.diff](./main-variants.diff)。

| 项目 | 主工程 | compare 历史文件 |
|---|---|---|
| 轮位映射 | A=LF、B=RF、C=LR、D=RR | A=LF、B=LR、C=RF、D=RR |
| 角速度分母 | `2 × (WHEEL_BASE + WHEEL_TRACK)` | `4 × (WHEEL_BASE + WHEEL_TRACK)` |

版本选择须与实际接线和轮位对应，本次未调整原控制算法。
