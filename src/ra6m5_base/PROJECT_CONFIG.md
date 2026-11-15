# Rasc_Core 项目配置手册

## 1. 项目概述

**项目名称**: Rasc_Core (自动拾球小车底盘控制系统)
**硬件平台**: Renesas RA6M5 MCU (Cortex-M33, 256KB SRAM, 1MB Flash)
**开发环境**: Keil uVision 5 + Renesas e2studio (配置)
**HAL框架**: Renesas FSP 6.4.0

### 1.1 功能特性

- 4轮麦克纳姆轮全向移动
- 4路独立PID电机控制
- IMU姿态融合 (WT901)
- ROS上位机通信
- 里程计输出

### 1.2 目录结构

```
E:\Rasc_Core/
├── src/                    # 核心源代码
│   ├── hal_entry.c        # 主程序入口 (341行)
│   ├── motor.c/h          # 电机驱动
│   ├── imu.c/h            # IMU传感器驱动
│   ├── pid.c/h            # PID控制算法
│   └── kinematics.c/h     # 运动学计算
├── ra_gen/                 # FSP自动生成代码
├── ra/                     # Renesas FSP库
├── ra_cfg/                 # 硬件配置
├── configuration.xml      # FSP配置主文件
└── Led.uvprojx            # Keil项目文件
```

---

## 2. 硬件配置

### 2.1 MCU引脚分配

#### 电机PWM引脚 (GPT定时器)

| 电机 | 位置 | PWM引脚 | GPT通道 | 方向脚1 | 方向脚2 |
|-----|------|---------|---------|---------|---------|
| A | 左前 | P415 | GPT0 | P210 | P906 |
| B | 左后 | P405 | GPT1 | P304 | P110 |
| C | 右前 | P113 | GPT2 | P311 | P907 |
| D | 右后 | P111 | GPT3 | P900 | P214 |

#### 编码器引脚 (GPT输入捕获)

| 编码器 | 通道A | 通道B | GPT通道 |
|-------|-------|-------|---------|
| ENC_A | P407 | P408 | GPT6 |
| ENC_B | P004 | P005 | GPT7 |
| ENC_C | P205 | P204 | GPT4 |
| ENC_D | P104 | P105 | GPT8 |

#### UART引脚

| 用途 | SCI通道 | TX引脚 | RX引脚 | 波特率 |
|-----|---------|--------|--------|--------|
| ROS通信 | SCI9 | P109 | P110 | 115200 |
| IMU通信 | SCI4 | P512 | P511 | 115200 |

#### 其他引脚

| 功能 | 引脚 | 说明 |
|-----|------|------|
| 用户按键 | P000 | 低电平有效，用于手动控制 |
| 调试LED | P404 | 状态指示 |

### 2.2 时钟配置

```
主时钟源: HOCO (24MHz)
系统时钟: 200MHz (PLL倍频)
外设时钟:
  - PCLKA: 100MHz (GPT定时器)
  - PCLKB: 50MHz (SCI串口)
```

---

## 3. 通信协议规范

### 3.1 ROS上位机串口协议

#### 3.1.1 物理层配置

```
波特率: 115200
数据位: 8
停止位: 1
校验位: 无
流控制: 无
```

#### 3.1.2 命令帧格式 (上位机 → 下位机)

```
格式: $vx,vy,wz\n
字段:
  $ - 帧头 (ASCII 0x24)
  vx - X方向线速度 (m/s)，正值前进
  vy - Y方向线速度 (m/s)，正值左移
  wz - Z轴角速度 (rad/s)，正值逆时针
  \n - 帧尾 (ASCII 0x0A)

示例:
  $0.5,0.0,0.0\n    // 前进 0.5 m/s
  $0.0,0.3,0.0\n    // 左移 0.3 m/s
  $0.0,0.0,1.57\n   // 原地逆时针旋转 90°/s
  $0.5,0.2,0.5\n    // 复合运动
  $0.0,0.0,0.0\n    // 停止
```

#### 3.1.3 里程计帧格式 (下位机 → 上位机)

```
格式: $x,y,yaw,vx,vy,wz\r\n
周期: 50ms (20Hz)
字段:
  x - X坐标 (m)
  y - Y坐标 (m)
  yaw - 航向角 (rad)，使用IMU数据
  vx - 当前X线速度 (m/s)
  vy - 当前Y线速度 (m/s)
  wz - 当前角速度 (rad/s)
  \r\n - 帧尾

示例:
  $1.234,0.567,0.785,0.500,0.100,0.314\r\n
```

#### 3.1.4 超时保护

```
超时时间: 500ms
行为: 无命令接收时自动停止所有电机
实现: src/hal_entry.c:206-223
```

### 3.2 IMU传感器协议 (WT901)

#### 3.2.1 帧格式

```
帧长度: 11字节
格式: [帧头][类型][数据L][数据H]...[校验和]

帧头: 0x55 (固定)
类型:
  0x51 - 加速度数据
  0x52 - 角速度数据
  0x53 - 欧拉角数据

数据转换公式:
  加速度(g) = (int16_t)(DataH<<8 | DataL) / 32768.0 * 16
  角速度(°/s) = (int16_t)(DataH<<8 | DataL) / 32768.0 * 2000
  欧拉角(°) = (int16_t)(DataH<<8 | DataL) / 32768.0 * 180
```

#### 3.2.2 数据结构

```c
typedef struct {
    float roll;   // 横滚角 (°)
    float pitch;  // 俯仰角 (°)
    float yaw;    // 航向角 (°)
    float acc_x;  // X加速度 (g)
    float acc_y;  // Y加速度 (g)
    float acc_z;  // Z加速度 (g)
    float gyro_x; // X角速度 (°/s)
    float gyro_y; // Y角速度 (°/s)
    float gyro_z; // Z角速度 (°/s)
} IMU_Data_t;

// 全局变量
extern volatile IMU_Data_t robot_imu;
```

---

## 4. 运动学参数配置

### 4.1 物理参数 (需根据实际测量调整)

```c
// 文件: src/kinematics.h

#define WHEEL_RADIUS      0.0475f   // 轮子半径 (m)
#define WHEEL_TRACK       0.2100f   // 左右轮距 (m)
#define WHEEL_BASE        0.1700f   // 前后轴距 (m)
#define ENCODER_PPR       13        // 编码器每转脉冲数
#define GEAR_RATIO        30.0f     // 减速比
#define ENCODER_MODE      4         // 4倍频计数
```

### 4.2 麦轮布局 (X型)

```
        前进方向
           ↑
     A ╲      ╱ C
       ╲    ╱
        ╲  ╱
         ╲╱
         ╱╲
        ╱  ╲
       ╱    ╲
     B ╱      ╲ D

滚轴朝外，45°安装
```

### 4.3 逆运动学公式

```
速度命令(vx, vy, wz) → 轮速(rad/s)

ω_A = (vx - vy - (L_x + L_y) * wz) / R
ω_B = (vx + vy - (L_x + L_y) * wz) / R
ω_C = (vx + vy + (L_x + L_y) * wz) / R
ω_D = (vx - vy + (L_x + L_y) * wz) / R

其中:
  L_x = WHEEL_BASE / 2
  L_y = WHEEL_TRACK / 2
  R = WHEEL_RADIUS
```

---

## 5. PID控制参数

### 5.1 当前参数

```c
// 文件: src/hal_entry.c:180-188

PID_A.Kp = 700.0f;  // 比例系数
PID_A.Ki = 4.0f;    // 积分系数
PID_A.Kd = 0.0f;    // 微分系数 (通常为0)
PID_A.Max_Output = 9500;  // PWM限幅

// 其他三路(B,C,D)参数相同
```

### 5.2 调参建议

```
1. 先调Kp (从小到大):
   - 太小: 响应慢，跟不上目标
   - 太大: 震荡，超调
   - 建议范围: 10.0 ~ 100.0

2. 再调Ki:
   - 太小: 稳态误差
   - 太大: 积分饱和，震荡
   - 建议范围: 0.5 ~ 10.0

3. Kd一般为0:
   - 编码器采样已有滤波
   - 非必要不使用
```

### 5.3 死区补偿

```c
// 文件: src/motor.h

#define MOTOR_DEAD_ZONE 1500  // PWM死区值
// 电机在此PWM值以下无法启动
```

---

## 6. 定时器配置

### 6.1 主循环定时器 (AGT0)

```
周期: 10ms (100Hz)
用途: 主控制循环触发
回调: agt_callback() → flag_10ms = 1
```

### 6.2 PWM定时器 (GPT0-3)

```
频率: 10kHz
占空比: 0-9999 (对应0%-100%)
模式: 锯齿波
```

### 6.3 编码器定时器 (GPT4,6,7,8)

```
模式: 相位计数模式 (4倍频)
计数方向: 双向
溢出: 65535 (16位)
```

---

## 7. 主程序流程

```
hal_entry() {
    ┌─ 初始化阶段 ─────────────────┐
    │ 1. Motor_Init()              │
    │ 2. IMU_Init()                │
    │ 3. 打开GPT/AGT定时器         │
    │ 4. 打开UART (ROS + IMU)      │
    │ 5. 初始化PID参数             │
    └──────────────────────────────┘
                │
                ▼
    ┌─ 主循环 (while(1)) ─────────┐
    │                              │
    │  if (flag_10ms) {           │
    │    ├─ 超时检测 (500ms)       │
    │    ├─ 解析UART命令           │
    │    ├─ 运动学逆解             │
    │    ├─ 读取编码器             │
    │    ├─ 速度滤波               │
    │    ├─ 运动学正解 → 里程计    │
    │    ├─ 4路PID计算             │
    │    ├─ 电机PWM输出            │
    │    └─ 50ms发送里程计 (20Hz)  │
    │  }                          │
    │                              │
    └──────────────────────────────┘
}
```

---

## 8. 编译与烧录

### 8.1 编译步骤 (Keil)

```
1. 打开项目: Led.uvprojx
2. 编译: Build → Rebuild All (F7)
3. 输出: Objects/Led.axf
```

### 8.2 烧录配置

```
调试器: CMSIS-DAP / J-Link
接口: SWD
时钟: 10MHz (建议)
```

### 8.3 FSP配置修改

```
1. 使用e2studio打开项目
2. 修改 configuration.xml
3. 点击 "Generate Project Content"
4. 返回Keil重新编译
```

---

## 9. 调试指南

### 9.1 串口调试

```
工具: 串口助手 (推荐SSCOM)
连接: USB转TTL → P109(TX), P110(RX)
波特率: 115200

测试命令:
  $0.5,0.0,0.0\n  // 前进
  $0.0,0.0,0.0\n  // 停止
```

### 9.2 常见问题

| 问题 | 可能原因 | 解决方案 |
|-----|---------|---------|
| 电机不转 | PWM < 死区 | 增大Kp或减小死区值 |
| 走不直 | 轮径不一致 | 校准WHEEL_RADIUS |
| 转弯漂移 | 轴距测量不准 | 校准WHEEL_TRACK/BASE |
| 里程计漂移 | 编码器参数错误 | 校准ENCODER_PPR/GEAR_RATIO |
| IMU角度跳变 | 校验和错误 | 检查IMU接线 |

---

## 10. ROS集成示例

### 10.1 Python示例 (上位机)

```python
import serial
import struct

class RascDriver:
    def __init__(self, port='/dev/ttyUSB0'):
        self.ser = serial.Serial(port, 115200, timeout=0.1)

    def send_velocity(self, vx, vy, wz):
        """发送速度命令"""
        cmd = f"${vx:.3f},{vy:.3f},{wz:.3f}\n"
        self.ser.write(cmd.encode())

    def read_odometry(self):
        """读取里程计"""
        line = self.ser.readline().decode().strip()
        if line.startswith('$'):
            parts = line[1:].split(',')
            if len(parts) == 6:
                return {
                    'x': float(parts[0]),
                    'y': float(parts[1]),
                    'yaw': float(parts[2]),
                    'vx': float(parts[3]),
                    'vy': float(parts[4]),
                    'wz': float(parts[5])
                }
        return None

    def stop(self):
        """停止"""
        self.send_velocity(0, 0, 0)

# 使用示例
driver = RascDriver('/dev/ttyUSB0')
driver.send_velocity(0.5, 0, 0)  # 前进
```

### 10.2 ROS2节点集成

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import threading

class RascNode(Node):
    def __init__(self):
        super().__init__('rasc_driver')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)

        # 订阅速度命令
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)

        # 发布里程计
        self.odom_pub = self.create_publisher(
            Odometry, '/odom', 10)

        # 里程计读取线程
        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.start()

    def cmd_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        cmd = f"${vx:.3f},{vy:.3f},{wz:.3f}\n"
        self.ser.write(cmd.encode())

    def read_loop(self):
        while rclpy.ok():
            # 读取并发布里程计...
            pass
```

---

## 11. 版本历史

| 版本 | 日期 | 说明 |
|-----|------|------|
| 1.0 | 2024-XX | 初始版本 |

---

## 12. 参考文档

- [PARAMS_README.md](./PARAMS_README.md) - 参数调试详细指南
- [FSP_ROS_UART_CONFIG.md](./FSP_ROS_UART_CONFIG.md) - UART配置指南
- [Renesas FSP文档](https://www.renesas.com/fsp) - 官方HAL文档
