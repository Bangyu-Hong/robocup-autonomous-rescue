# FSP 配置 ROS 上位机串口指南

本指南将帮助您在 Renesas FSP 中配置用于 ROS 通信的串口。

---

## 当前项目配置信息

根据您的项目，已配置的外设：
- **IMU 串口**: SCI4 通道 (g_uart_imu)，波特率 115200
- **GPT0-3**: PWM 输出（电机驱动）
- **GPT4-7**: 编码器计数
- **GPT0**: 10ms 定时器

---

## 步骤一：打开 FSP 配置器

1. 在 e² studio 中，双击项目中的 `configuration.xml` 文件
2. 或者右键项目 → `RA Smart Configurator`

---

## 步骤二：添加 UART 模块

### 2.1 添加新的 UART Stack

1. 点击 **Stacks** 选项卡
2. 点击 **New Stack** → **Connectivity** → **UART (r_sci_uart)**
3. 会自动添加一个新的 UART 实例

### 2.2 配置 UART 参数

点击新添加的 UART 模块，在 **Properties** 面板中设置：

| 属性 | 设置值 | 说明 |
|------|--------|------|
| **General** | | |
| Name | `g_uart_ros` | 实例名称 |
| Channel | `9` | 选择一个未使用的 SCI 通道 (推荐 SCI9) |
| **Baud** | | |
| Baud Rate | `115200` | 波特率 |
| **Interrupts** | | |
| Callback | `ros_uart_callback` | 回调函数名 (代码中已定义) |
| Receive Interrupt Priority | `3` | 接收中断优先级 |
| Transmit Data Empty Interrupt Priority | `3` | 发送中断优先级 |
| Transmit End Interrupt Priority | `3` | 发送完成中断优先级 |
| Error Interrupt Priority | `3` | 错误中断优先级 |

### 2.3 详细参数设置

```
┌─────────────────────────────────────────────────────────┐
│  Module: UART (r_sci_uart)                              │
├─────────────────────────────────────────────────────────┤
│  General                                                │
│    Name                    : g_uart_ros                 │
│    Channel                 : 9                          │
│    Data Bits               : 8bits                      │
│    Parity                  : None                       │
│    Stop Bits               : 1bit                       │
│                                                         │
│  Baud                                                   │
│    Baud Rate               : 115200                     │
│    Baud Rate Modulation    : Disabled                   │
│                                                         │
│  Flow Control                                           │
│    CTS/RTS Selection       : Hardware RTS              │
│    Software RTS Port       : Disabled                   │
│                                                         │
│  Extra                                                  │
│    Clock Source            : Internal Clock             │
│    Start bit detection     : Falling Edge               │
│    Noise Filter            : Disable                    │
│    Receive FIFO Trigger    : Max                        │
│                                                         │
│  Interrupts                                             │
│    Callback                : ros_uart_callback          │
│    Receive Interrupt Priority       : Priority 3        │
│    Transmit Data Empty Int Priority : Priority 3        │
│    Transmit End Interrupt Priority  : Priority 3        │
│    Error Interrupt Priority         : Priority 3        │
└─────────────────────────────────────────────────────────┘
```

---

## 步骤三：配置引脚

### 3.1 打开 Pins 选项卡

1. 点击 **Pins** 选项卡
2. 展开 **Peripherals** → **Connectivity:SCI** → **SCI9**

### 3.2 选择引脚

根据您使用的 SCI 通道，选择对应的 TXD 和 RXD 引脚：

**SCI9 可用引脚选项：**

| 引脚功能 | 可选引脚 | 建议 |
|----------|----------|------|
| TXD9 | P109, P203, P602 | 选择未使用的 |
| RXD9 | P110, P202, P601 | 选择未使用的 |

**配置示例（使用 P109/P110）：**
```
SCI9:
  Operation Mode : Asynchronous UART
  TXD9          : P109
  RXD9          : P110
```

### 3.3 其他 SCI 通道可用引脚参考

如果 SCI9 的引脚不可用，可以选择其他通道：

| 通道 | TXD | RXD | 备注 |
|------|-----|-----|------|
| SCI0 | P411 | P410 | |
| SCI1 | P401, P709 | P402, P708 | |
| SCI2 | P112, P302 | P113, P301 | P113 已用于 PWM |
| SCI3 | P310, P409 | P309, P408 | |
| SCI5 | P501 | P502 | |
| SCI6 | P506 | P505 | |
| SCI7 | P614 | P615 | |
| SCI9 | P109, P203, P602 | P110, P202, P601 | 推荐 |

**注意：** 请检查您的硬件原理图，确认选择的引脚没有被其他功能占用！

---

## 步骤四：生成代码

1. 点击 **Generate Project Content** 按钮（齿轮图标）
2. 等待代码生成完成
3. 检查 `ra_gen/hal_data.h` 中是否出现 `g_uart_ros` 相关声明

---

## 步骤五：修改代码启用 ROS 串口

### 5.1 修改 hal_entry.c

取消注释并修改初始化代码（约第 134-137 行）：

```c
// 初始化上位机串口
R_SCI_UART_Open(&g_uart_ros_ctrl, &g_uart_ros_cfg);
uint8_t dummy_rx;
R_SCI_UART_Read(&g_uart_ros_ctrl, &dummy_rx, 1);
```

### 5.2 添加串口接收重触发

在 `ros_uart_callback` 函数末尾添加（参考 `imu_uart_callback` 的修改）：

```c
void ros_uart_callback(uart_callback_args_t *p_args)
{
    if (p_args->event == UART_EVENT_RX_CHAR)
    {
        char c = (char)p_args->data;

        if (c == '\n')
        {
            ros_rx_buf[ros_rx_index] = '\0';
            ros_cmd_ready = 1;
            ros_rx_index = 0;
        }
        else if (ros_rx_index < 60 && c != '\r')
        {
            ros_rx_buf[ros_rx_index] = c;
            ros_rx_index++;
        }

        // 【重要】重新启动下一次接收
        R_SCI_UART_Read(&g_uart_ros_ctrl, (uint8_t*)&c, 1);
    }
}
```

---

## 步骤六：硬件连接

### 6.1 接线说明

```
┌──────────────────┐         ┌──────────────────┐
│   RA6M5 MCU      │         │  USB转TTL模块    │
│                  │         │  (连接电脑)       │
│   TXD9 (P109) ───┼────────►│  RXD             │
│   RXD9 (P110) ◄──┼─────────┤  TXD             │
│   GND ───────────┼─────────┤  GND             │
└──────────────────┘         └──────────────────┘
```

### 6.2 注意事项

1. **电平匹配**: RA6M5 是 3.3V 电平，如果使用 5V USB转TTL，需要电平转换
2. **交叉连接**: MCU 的 TXD 接模块的 RXD，MCU 的 RXD 接模块的 TXD
3. **共地**: 必须连接 GND

---

## 步骤七：测试验证

### 7.1 电脑端测试

1. 打开串口调试助手（如 SSCOM、PuTTY）
2. 设置波特率 115200，8N1
3. 发送测试命令：`$0.1,0,0\n`（前进 0.1m/s）

### 7.2 预期结果

- 小车应该开始前进
- 每 50ms 会收到里程计数据：`$x,y,yaw,vx,vy,wz\r\n`

---

## 常见问题

### Q1: 找不到空闲的 SCI 通道
检查各通道使用情况：
- SCI4: 已用于 IMU
- SCI0-3, 5-9: 检查引脚是否与 PWM/编码器冲突

### Q2: 串口无数据输出
1. 检查 `printf` 重定向是否正确配置
2. 确认回调函数名与 FSP 配置一致
3. 确认已调用 `R_SCI_UART_Open()`

### Q3: 只能收一次数据
确保在回调函数末尾调用了 `R_SCI_UART_Read()` 重新启动接收

---

## 附录：SCI 通道与引脚对照表 (RA6M5)

| SCI | TXD 引脚选项 | RXD 引脚选项 |
|-----|-------------|-------------|
| 0 | P411 | P410 |
| 1 | P401, P709 | P402, P708 |
| 2 | P112, P302 | P113, P301 |
| 3 | P310, P409 | P309, P408 |
| 4 | P205, P512 | P206, P511 |
| 5 | P501 | P502 |
| 6 | P506 | P505 |
| 7 | P614 | P615 |
| 8 | P105 | P104 |
| 9 | P109, P203, P602 | P110, P202, P601 |
