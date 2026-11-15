# WT901BCTTL IMU 使用指南

> 本文档详细介绍维特智能 WT901BCTTL 九轴姿态传感器在本项目中的使用方法

---

## 目录

1. [硬件概述](#1-硬件概述)
2. [接线说明](#2-接线说明)
3. [数据协议](#3-数据协议)
4. [软件实现](#4-软件实现)
5. [角度归零功能](#5-角度归零功能)
6. [配置与校准](#6-配置与校准)
7. [常见问题](#7-常见问题)

---

## 1. 硬件概述

### 1.1 WT901BCTTL 简介

WT901BCTTL 是维特智能推出的一款高精度九轴姿态传感器模块，集成了：

| 传感器 | 型号 | 功能 |
|--------|------|------|
| 三轴加速度计 | - | 测量线性加速度 |
| 三轴陀螺仪 | - | 测量角速度 |
| 三轴磁力计 | - | 测量磁场强度 |

### 1.2 主要参数

| 参数 | 规格 |
|------|------|
| 工作电压 | 3.3V ~ 5V |
| 通信接口 | TTL串口 / I2C |
| 默认波特率 | 115200 bps |
| 数据输出频率 | 10Hz ~ 200Hz (可配置) |
| 加速度量程 | ±16g |
| 陀螺仪量程 | ±2000°/s |
| 角度精度 | 静态 0.05°, 动态 0.1° |

### 1.3 模块引脚定义

```
┌─────────────────────────────┐
│       WT901BCTTL            │
│                             │
│  VCC ──── 电源正极 (3.3~5V) │
│  GND ──── 电源负极          │
│  TX  ──── 串口发送          │
│  RX  ──── 串口接收          │
│  SDA ──── I2C数据 (可选)    │
│  SCL ──── I2C时钟 (可选)    │
│                             │
└─────────────────────────────┘
```

---

## 2. 接线说明

### 2.1 与STM32F407连接

| WT901引脚 | STM32F407引脚 | 说明 |
|-----------|---------------|------|
| VCC | 5V 或 3.3V | 电源供电 |
| GND | GND | 共地 |
| TX | PA3 (USART2_RX) | IMU发送 → STM32接收 |
| RX | PA2 (USART2_TX) | STM32发送 → IMU接收 (用于配置) |

### 2.2 接线示意图

```
┌─────────────────┐              ┌─────────────────┐
│   STM32F407     │              │  WT901BCTTL     │
│                 │              │                 │
│  5V ────────────┼──────────────┼── VCC           │
│  GND ───────────┼──────────────┼── GND           │
│  PA2 (TX) ──────┼──────────────┼── RX            │
│  PA3 (RX) ◄─────┼──────────────┼── TX            │
│                 │              │                 │
└─────────────────┘              └─────────────────┘
```

### 2.3 安装位置建议

- **位置**：安装在机器人重心附近，尽量靠近旋转中心
- **方向**：X轴指向前方，Y轴指向左侧，Z轴指向上方
- **固定**：使用减震垫固定，避免直接刚性连接
- **远离**：远离电机、电源线等电磁干扰源

```
        X+ (前)
         ↑
         │    ┌───────┐
    Y+ ←─┼────│ IMU   │
   (左)  │    │ 模块  │
         │    └───────┘
        Z+ (上，垂直纸面向外)
```

---

## 3. 数据协议

### 3.1 数据帧格式

WT901BCTTL 输出的每个数据包为 **11字节**：

```
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
│ 0x55 │ Type │ DL   │ DH   │ DL   │ DH   │ DL   │ DH   │ DL   │ DH   │ Sum  │
├──────┼──────┼──────┴──────┼──────┴──────┼──────┴──────┼──────┴──────┼──────┤
│ 包头 │ 类型 │   数据1     │   数据2     │   数据3     │   数据4     │ 校验 │
│      │      │  (16bit)    │  (16bit)    │  (16bit)    │  (16bit)    │      │
└──────┴──────┴─────────────┴─────────────┴─────────────┴─────────────┴──────┘

字节0: 0x55      - 固定包头
字节1: Type      - 数据类型标识
字节2-3: Data1   - 第一个数据 (小端序: 低字节在前)
字节4-5: Data2   - 第二个数据
字节6-7: Data3   - 第三个数据
字节8-9: Data4   - 第四个数据 (通常为温度)
字节10: Sum      - 校验和 = (Byte0 + Byte1 + ... + Byte9) & 0xFF
```

### 3.2 数据类型定义

| Type | 名称 | Data1 | Data2 | Data3 | Data4 |
|------|------|-------|-------|-------|-------|
| 0x51 | 加速度 | Ax | Ay | Az | Temperature |
| 0x52 | 角速度 | Wx | Wy | Wz | Temperature |
| 0x53 | 欧拉角 | Roll | Pitch | Yaw | Temperature |
| 0x54 | 磁场 | Hx | Hy | Hz | Temperature |

### 3.3 数据转换公式

#### 加速度 (0x51)
```
Ax(g) = AxRaw / 32768.0 × 16      // 量程 ±16g
Ay(g) = AyRaw / 32768.0 × 16
Az(g) = AzRaw / 32768.0 × 16

// 转换为 m/s²
Ax(m/s²) = Ax(g) × 9.8
```

#### 角速度 (0x52)
```
Wx(°/s) = WxRaw / 32768.0 × 2000  // 量程 ±2000°/s
Wy(°/s) = WyRaw / 32768.0 × 2000
Wz(°/s) = WzRaw / 32768.0 × 2000

// 转换为 rad/s
Wx(rad/s) = Wx(°/s) × π / 180
```

#### 欧拉角 (0x53)
```
Roll(°)  = RollRaw / 32768.0 × 180   // 量程 ±180°
Pitch(°) = PitchRaw / 32768.0 × 180
Yaw(°)   = YawRaw / 32768.0 × 180

// 转换为 rad
Roll(rad) = Roll(°) × π / 180
```

#### 温度
```
Temperature(°C) = TempRaw / 100.0
```

### 3.4 数据帧示例

**欧拉角数据帧 (0x53)**：
```
55 53 00 00 00 00 E8 03 00 00 40

解析:
- 包头: 0x55
- 类型: 0x53 (欧拉角)
- Roll:  0x0000 = 0 → 0°
- Pitch: 0x0000 = 0 → 0°
- Yaw:   0x03E8 = 1000 → 1000/32768×180 = 5.49°
- Temp:  0x0000
- 校验:  0x40 = (0x55+0x53+0x00+...+0x00) & 0xFF
```

---

## 4. 软件实现

### 4.1 数据结构定义

```c
// 文件: main.c (第104-115行)

typedef struct {
    float roll;       // 横滚角 (rad)
    float pitch;      // 俯仰角 (rad)
    float yaw;        // 偏航角 (rad) - 用于里程计
    float gyro_x;     // X轴角速度 (rad/s)
    float gyro_y;     // Y轴角速度 (rad/s)
    float gyro_z;     // Z轴角速度 (rad/s) - 用于里程计
    float accel_x;    // X轴加速度 (m/s²)
    float accel_y;    // Y轴加速度 (m/s²)
    float accel_z;    // Z轴加速度 (m/s²)
    uint8_t updated;  // 数据更新标志
} IMU_Data_t;

IMU_Data_t imu_data = {0};
```

### 4.2 接收状态机

```c
// 文件: main.c (HAL_UART_RxCpltCallback函数)

// 状态机变量
uint8_t imu_rx_buffer[11];  // 接收缓冲区
uint8_t imu_rx_state = 0;   // 状态: 0=寻找包头, 1=接收数据
uint8_t imu_rx_index = 0;   // 接收索引

// 接收流程
switch(imu_rx_state)
{
    case 0: // 寻找包头
        if(rx_temp == 0x55) {
            imu_rx_buffer[0] = 0x55;
            imu_rx_index = 1;
            imu_rx_state = 1;
        }
        break;

    case 1: // 接收数据
        imu_rx_buffer[imu_rx_index++] = rx_temp;
        if(imu_rx_index >= 11) {
            Parse_IMU_Data(imu_rx_buffer);  // 解析完整数据包
            imu_rx_state = 0;
            imu_rx_index = 0;
        }
        break;
}
```

### 4.3 数据解析函数

```c
// 文件: main.c (第460-533行)

void Parse_IMU_Data(uint8_t *data)
{
    // 1. 校验和验证
    uint8_t sum = 0;
    for(int i = 0; i < 10; i++) {
        sum += data[i];
    }
    if(sum != data[10]) {
        imu_check_err_count++;  // 校验错误计数
        return;
    }

    // 2. 根据数据类型解析
    switch(data[1])
    {
        case 0x51: // 加速度
        {
            int16_t ax = (int16_t)(data[3] << 8 | data[2]);
            int16_t ay = (int16_t)(data[5] << 8 | data[4]);
            int16_t az = (int16_t)(data[7] << 8 | data[6]);

            // 转换为 m/s² (量程16g)
            imu_data.accel_x = ax / 32768.0f * 16.0f * 9.8f;
            imu_data.accel_y = ay / 32768.0f * 16.0f * 9.8f;
            imu_data.accel_z = az / 32768.0f * 16.0f * 9.8f;
            break;
        }

        case 0x52: // 角速度
        {
            int16_t gx = (int16_t)(data[3] << 8 | data[2]);
            int16_t gy = (int16_t)(data[5] << 8 | data[4]);
            int16_t gz = (int16_t)(data[7] << 8 | data[6]);

            // 转换为 rad/s (量程2000°/s)
            imu_data.gyro_x = gx / 32768.0f * 2000.0f * PI / 180.0f;
            imu_data.gyro_y = gy / 32768.0f * 2000.0f * PI / 180.0f;
            imu_data.gyro_z = gz / 32768.0f * 2000.0f * PI / 180.0f;
            break;
        }

        case 0x53: // 欧拉角
        {
            int16_t roll = (int16_t)(data[3] << 8 | data[2]);
            int16_t pitch = (int16_t)(data[5] << 8 | data[4]);
            int16_t yaw = (int16_t)(data[7] << 8 | data[6]);

            // 转换为 rad (量程180°)
            imu_data.roll = roll / 32768.0f * PI;
            imu_data.pitch = pitch / 32768.0f * PI;
            float raw_yaw = yaw / 32768.0f * PI;

            // 应用角度归零偏移
            imu_data.yaw = raw_yaw - yaw_offset;

            // 角度归一化到 [-π, π]
            while(imu_data.yaw > PI) imu_data.yaw -= 2.0f * PI;
            while(imu_data.yaw < -PI) imu_data.yaw += 2.0f * PI;

            imu_data.updated = 1;
            break;
        }
    }
}
```

### 4.4 在里程计中使用IMU

```c
// 文件: main.c (Update_Odometry函数)

#if USE_IMU == 1
    // 使用IMU的yaw角作为机器人角度（更精确）
    if(imu_data.updated) {
        odom_data.th = imu_data.yaw;      // 使用IMU角度
        odom_data.wz = imu_data.gyro_z;   // 使用IMU角速度
        imu_data.updated = 0;
    }
#else
    // 仅使用编码器计算角度（精度较低）
    odom_data.th += wz_local * DT;
#endif
```

---

## 5. 角度归零功能

### 5.1 软件归零（自动）

系统启动后自动执行，无需用户干预。

**原理**：
```
启动 → 等待10帧IMU数据 → 记录当前yaw为偏移量 → 之后输出归零后的角度
```

**相关变量**：
```c
float yaw_offset = 0.0f;           // Yaw角度偏移量
uint8_t yaw_calibrated = 0;        // 校准标志
#define YAW_CALIBRATION_FRAMES 10  // 等待帧数
```

**实现代码**：
```c
// 在解析0x53欧拉角时
if(!yaw_calibrated && imu_frame_count >= YAW_CALIBRATION_FRAMES) {
    yaw_offset = raw_yaw;   // 记录初始角度
    yaw_calibrated = 1;     // 标记已校准
}
imu_data.yaw = raw_yaw - yaw_offset;  // 应用偏移
```

### 5.2 硬件归零（手动）

通过长按按键触发，向WT901发送归零指令。

**触发方式**：长按按键（≥500ms）后松开

**指令序列**：
```c
// 步骤1: 解锁
uint8_t unlock_cmd[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};

// 步骤2: Z轴角度归零
uint8_t zero_cmd[5] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
```

**函数实现**：
```c
void IMU_Hardware_Reset_Yaw(void)
{
    // 解锁
    uint8_t unlock_cmd[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    HAL_UART_Transmit(&IMU_UART, unlock_cmd, 5, 100);
    HAL_Delay(50);

    // Z轴归零
    uint8_t zero_cmd[5] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
    HAL_UART_Transmit(&IMU_UART, zero_cmd, 5, 100);
    HAL_Delay(50);

    // 同时执行软件归零
    yaw_calibrated = 0;
    yaw_offset = 0.0f;
    odom_data.th = 0.0f;
    odom_data.x = 0.0f;
    odom_data.y = 0.0f;
}
```

### 5.3 归零方式对比

| 特性 | 软件归零 | 硬件归零 |
|------|----------|----------|
| 触发方式 | 自动（上电） | 手动（长按按键） |
| 响应速度 | 立即生效 | 需等待IMU响应 |
| 是否写入Flash | 否 | 否（可配置） |
| 重复执行 | 每次上电 | 按需执行 |
| 影响范围 | 仅本次运行 | IMU内部归零 |

---

## 6. 配置与校准

### 6.1 WT901配置指令格式

```
┌──────┬──────┬──────┬──────┬──────┐
│ 0xFF │ 0xAA │ Reg  │ DataL│ DataH│
└──────┴──────┴──────┴──────┴──────┘

0xFF 0xAA: 固定帧头
Reg: 寄存器地址
DataL/DataH: 写入数据 (小端序)
```

### 6.2 常用配置指令

| 功能 | 指令 | 说明 |
|------|------|------|
| 解锁 | `FF AA 69 88 B5` | 修改配置前必须先解锁 |
| 保存配置 | `FF AA 00 00 00` | 保存到Flash |
| Z轴角度归零 | `FF AA 01 04 00` | 清零Z轴角度 |
| 加速度校准 | `FF AA 01 01 00` | 加速度计校准 |
| 恢复出厂 | `FF AA 00 01 00` | 恢复默认设置 |
| 设置波特率115200 | `FF AA 04 06 00` | 修改波特率 |
| 设置输出频率100Hz | `FF AA 03 09 00` | 修改输出频率 |

### 6.3 加速度计校准步骤

1. 将模块水平放置
2. 发送解锁指令：`FF AA 69 88 B5`
3. 等待50ms
4. 发送校准指令：`FF AA 01 01 00`
5. 等待3秒完成校准
6. 发送保存指令：`FF AA 00 00 00`

### 6.4 磁力计校准步骤

1. 发送解锁指令：`FF AA 69 88 B5`
2. 发送开始校准：`FF AA 01 07 00`
3. 缓慢旋转模块，覆盖各个方向（约30秒）
4. 发送结束校准：`FF AA 01 00 00`
5. 发送保存指令：`FF AA 00 00 00`

---

## 7. 常见问题

### 7.1 IMU数据不更新

**症状**：`imu_frame_count` 始终为0

**排查步骤**：
1. 检查接线：TX/RX是否交叉连接
2. 检查电源：是否供电正常
3. 检查波特率：确认为115200
4. 检查中断：USART2中断是否使能

### 7.2 角度漂移严重

**可能原因**：
- 磁力计未校准
- 存在磁场干扰
- 温度变化大

**解决方法**：
1. 执行磁力计校准
2. 远离电机和电源线
3. 等待温度稳定后再使用

### 7.3 校验错误多

**症状**：`imu_check_err_count` 持续增加

**可能原因**：
- 通信速率不匹配
- 接线接触不良
- 电磁干扰

**解决方法**：
1. 确认波特率一致
2. 检查接线是否牢固
3. 使用屏蔽线

### 7.4 角度归零不生效

**软件归零不生效**：
- 检查 `yaw_calibrated` 变量
- 确认已接收足够帧数（≥10帧）

**硬件归零不生效**：
- 确认TX线已连接（PA2 → IMU RX）
- 检查解锁指令是否发送成功
- 延时是否足够

### 7.5 调试方法

开启调试模式查看IMU状态：

```c
// main.h
#define DEBUG_MODE  1
```

调试输出格式：
```
#encA,encB,encC,encD,imu_cnt,th,x,y,err
                     ↑           ↑
                IMU帧计数    校验错误计数
```

---

## 附录：WT901寄存器地址表

| 地址 | 名称 | 说明 |
|------|------|------|
| 0x00 | SAVE | 保存配置 |
| 0x01 | CALSW | 校准开关 |
| 0x02 | RSW | 输出内容选择 |
| 0x03 | RATE | 输出频率 |
| 0x04 | BAUD | 波特率 |
| 0x23 | YYMM | 年月 |
| 0x24 | DDHH | 日时 |
| 0x25 | MMSS | 分秒 |
| 0x34-0x36 | AX/AY/AZ | 加速度 |
| 0x37-0x39 | WX/WY/WZ | 角速度 |
| 0x3D-0x3F | Roll/Pitch/Yaw | 欧拉角 |

---

*文档版本: v1.0*
*最后更新: 2026-01-24*
