# 历史版本对比

本地 `project` 与 `control（8M）` 中的业务源码核对一致，当前以 `control（8M）` 为主要收录目录。

`compare/control（8M）` 的主要源码差异位于 `Core/Src/main.c`，保留为 [compare-main.c](./compare-main.c)，完整文本差异见 [main-variants.diff](./main-variants.diff)。

主要差异：

| 项目 | 主目录源码 | compare 历史文件 |
|---|---|---|
| 轮位映射 | A=LF、B=RF、C=LR、D=RR | A=LF、B=LR、C=RF、D=RR |
| 轮速读取和目标分配 | 按主目录通道映射 | 按历史通道映射 |
| 里程计角速度 | 分母为 `2 × (WHEEL_BASE + WHEEL_TRACK)` | 分母为 `4 × (WHEEL_BASE + WHEEL_TRACK)` |

此归档用于追溯已有版本，不替换主工程，也未据此判断哪一套映射与具体实物接线一致。
