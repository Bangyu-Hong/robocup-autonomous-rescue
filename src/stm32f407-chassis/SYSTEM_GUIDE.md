# STM32F407 麦克纳姆轮机器人下位机系统代码讲解

> 本文档详细介绍该嵌入式项目的软件架构、核心算法和代码实现

---

## 目录

1. [系统概述](#1-系统概述)
2. [硬件配置](#2-硬件配置)
3. [软件架构](#3-软件架构)
4. [核心模块详解](#4-核心模块详解)
5. [通信协议](#5-通信协议)
6. [参数配置指南](#6-参数配置指南)
7. [调试指南](#7-调试指南)

---

## 1. 系统概述

### 1.1 项目简介

本项目是一个基于 **STM32F407** 的四轮麦克纳姆轮机器人下位机控制系统，主要功能包括：

- 四轮独立PID速度闭环控制
- 编码器里程计计算
- WT901BCTTL IMU数据融合
- 与ROS上位机的串口通信
- 本地按键测试功能

### 1.2 系统框图

```
┌─────────────────────────────────────────────────────────────────┐
│                         ROS 上位机                               │
│                    (发送速度指令/接收里程计)                       │
└───────────────────────────┬─────────────────────────────────────┘
                            │ USART1 (115200bps, DMA)
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                      STM32F407 主控制器                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │
│  │ 指令解析    │→ │ 逆运动学    │→ │ 四轮PID速度控制         │  │
│  │ (DMA接收)   │  │ (速度分解)  │  │ (TIM7 10ms中断)         │  │
│  └─────────────┘  └─────────────┘  └───────────┬─────────────┘  │
│                                                 │                │
│  ┌─────────────┐  ┌─────────────┐              ▼                │
│  │ IMU数据融合 │← │ WT901 解析  │    ┌─────────────────────┐   │
│  │ (角度修正)  │  │ (USART2)    │    │ PWM输出 (TIM1)      │   │
│  └──────┬──────┘  └─────────────┘    └──────────┬──────────┘   │
│         │                                        │               │
│         ▼                                        ▼               │
│  ┌─────────────────────────────┐    ┌─────────────────────────┐ │
│  │ 里程计计算 (50Hz)           │    │ 电机驱动 (H桥)          │ │
│  │ 编码器读取 + IMU角度        │    │ 方向控制 + PWM          │ │
│  └─────────────────────────────┘    └─────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        ▼                   ▼                   ▼
   ┌─────────┐        ┌─────────┐         ┌─────────┐
   │ 电机A   │        │ 电机B   │   ...   │ 电机D   │
   │ (左前)  │        │ (左后)  │         │ (右后)  │
   │ +编码器 │        │ +编码器 │         │ +编码器 │
   └─────────┘        └─────────┘         └─────────┘
```

### 1.3 坐标系定义

遵循 **ROS REP-103** 标准（右手定则）：

```
        X+ (前进)
         ↑
         │
  Y+ ←───┼───→ Y- (右移)
  (左移) │
         ↓
        X- (后退)

角度: 逆时针为正 (左转为正)
```

### 1.4 轮子布局

```
    前方 (X+)
      ↑
  ┌───┴───┐
  │ A   C │   A = 左前 (Left Front)
  │ (LF)(RF)│   B = 左后 (Left Rear)
  │       │   C = 右前 (Right Front)
  │ B   D │   D = 右后 (Right Rear)
  │(LR)(RR)│
  └───────┘
```

---

## 2. 硬件配置

### 2.1 时钟配置

| 时钟源 | 频率 | 说明 |
|--------|------|------|
| HSE (外部晶振) | 8 MHz | 主时钟源 |
| SYSCLK (系统时钟) | 168 MHz | PLL倍频后 |
| HCLK (AHB总线) | 168 MHz | CPU主频 |
| PCLK1 (APB1) | 42 MHz | 低速外设 |
| PCLK2 (APB2) | 84 MHz | 高速外设 |

**PLL配置**: `PLLM=8, PLLN=336, PLLP=2, PLLQ=7`

### 2.2 定时器配置

| 定时器 | 功能 | 频率/周期 | 配置文件位置 |
|--------|------|-----------|-------------|
| TIM1 | PWM输出(4路) | ~1kHz | tim.c:69-152 |
| TIM2 | C轮编码器 | 16位+溢出 | tim.c:156-191 |
| TIM3 | A轮编码器 | 16位+溢出 | tim.c:200-244 |
| TIM4 | B轮编码器 | 16位+溢出 | tim.c:246-290 |
| TIM7 | PID控制定时 | 1kHz (10ms采样) | tim.c:292-323 |
| TIM8 | D轮编码器 | 16位+溢出 | tim.c:325-370 |

### 2.3 串口配置

| 串口 | 波特率 | 功能 | 数据格式 |
|------|--------|------|----------|
| USART1 | 115200 | ROS通信 | 8N1, DMA收发 |
| USART2 | 115200 | IMU通信 | 8N1, 中断接收 |

---

## 3. 软件架构

### 3.1 文件结构

```
Core/
├── Inc/
│   ├── main.h          # 全局定义、引脚宏、参数配置
│   ├── tim.h           # 定时器函数声明
│   ├── usart.h         # 串口函数声明
│   └── stm32f4xx_it.h  # 中断函数声明
├── Src/
│   ├── main.c          # 主程序、通信协议、里程计
│   ├── tim.c           # 定时器配置、PID、电机驱动
│   ├── usart.c         # 串口配置、printf重定向
│   ├── gpio.c          # GPIO初始化
│   ├── dma.c           # DMA配置
│   └── stm32f4xx_it.c  # 中断服务程序、PID执行
```

### 3.2 执行流程

#### 3.2.1 系统初始化流程

```c
main()
├─ HAL_Init()                    // HAL库初始化
├─ SystemClock_Config()          // 时钟配置 (8MHz→168MHz)
├─ MX_GPIO_Init()                // GPIO初始化
├─ MX_DMA_Init()                 // DMA初始化
├─ MX_TIM1_Init()                // PWM初始化
├─ MX_TIM2/3/4/8_Init()          // 编码器初始化
├─ MX_TIM7_Init()                // PID定时器初始化
├─ MX_USART1/2_UART_Init()       // 串口初始化
├─ delay_init(168)               // 延时函数初始化
├─ HAL_UART_Receive_DMA()        // 启动ROS指令DMA接收
├─ HAL_UART_Receive_IT()         // 启动IMU中断接收
└─ while(1) 主循环
```

#### 3.2.2 主循环 (main.c:627-660)

```c
while(1)
{
    // 1. 按键处理 (实时)
    key_proc();

    // 2. ROS数据处理 (轮询DMA缓冲区)
    Parse_ROS_Buffer_DMA();

    // 3. 速度指令转换 (逆运动学)
    ROS_Cmd_To_Motor_Speed();

    // 4. 通信超时保护 (500ms)
    if(超时) { 停止所有电机; }

    // 5. 每20ms执行 (50Hz)
    if(HAL_GetTick() - last_time >= 20)
    {
        Update_Odometry();   // 更新里程计
        Send_GyroData();     // 发送数据到ROS
    }
}
```

#### 3.2.3 PID控制中断 (stm32f4xx_it.c:408-445)

```c
// TIM7中断，每1ms触发，每10ms执行PID
HAL_TIM_PeriodElapsedCallback()
{
    if(proc_slow % 10 == 0)  // 10ms周期
    {
        // 1. 获取四轮实时速度
        speedA = get_speedA_1s(10);
        speedB = get_speedB_1s(10);
        speedC = get_speedC_1s(10);
        speedD = get_speedD_1s(10);

        // 2. 更新速度斜坡 (平滑加减速)
        update_speed_ramps();

        // 3. PID控制计算
        pwmA = Velocity_motorA_FeedbackControl(rampTargetA, speedA);
        pwmB = Velocity_motorB_FeedbackControl(rampTargetB, speedB);
        pwmC = Velocity_motorC_FeedbackControl(rampTargetC, speedC);
        pwmD = Velocity_motorD_FeedbackControl(rampTargetD, speedD);

        // 4. 死区处理
        if(目标=0 且 速度<0.1) pwm = 0;

        // 5. 输出到电机
        motor_setspeed(pwmA, pwmB, pwmC, pwmD);
    }
}
```

---

## 4. 核心模块详解

### 4.1 PID速度控制

#### 4.1.1 增量式PID算法

文件位置: `tim.c:916-1033`

```c
// 增量式PID公式
// ΔU = Kp*(e[k]-e[k-1]) + Ki*e[k] + Kd*(e[k]-2*e[k-1]+e[k-2])

int Velocity_motorA_FeedbackControl(float target, float feedback)
{
    static float last_err = 0, prev_err = 0;
    static int pwm = 0;

    float err = target - feedback;  // 当前误差

    // 增量计算
    float increment = Velcity_Kp * (err - last_err)
                    + Velcity_Ki * err
                    + Velcity_Kd * (err - 2*last_err + prev_err);

    pwm += (int)increment;

    // 输出限幅
    if(pwm > 1000) pwm = 1000;
    if(pwm < -1000) pwm = -1000;

    // 更新历史误差
    prev_err = last_err;
    last_err = err;

    return pwm;
}
```

#### 4.1.2 PID参数说明

| 参数 | 值 | 作用 | 调参建议 |
|------|-----|------|----------|
| Kp | 710 | 比例增益，响应速度 | 增大加快响应，过大会震荡 |
| Ki | 25 | 积分增益，消除稳态误差 | 增大减小稳态误差，过大会超调 |
| Kd | 9 | 微分增益，抑制震荡 | 增大抑制震荡，过大会放大噪声 |

#### 4.1.3 速度斜坡控制

文件位置: `tim.c:690-722`

```c
// 目的：平滑加减速，保护电机和齿轮箱
#define SPEED_RAMP_RATE 0.1f  // 每10ms最大变化0.1转/秒

void update_speed_ramps(void)
{
    // A轮斜坡
    if(targetA > rampTargetA)
        rampTargetA = fmin(rampTargetA + SPEED_RAMP_RATE, targetA);
    else
        rampTargetA = fmax(rampTargetA - SPEED_RAMP_RATE, targetA);

    // B、C、D轮同理...
}
```

**效果**: 从0加速到1转/秒需要100ms (10个周期)

### 4.2 编码器处理

#### 4.2.1 编码器读取原理

采用 **16位定时器 + 溢出中断计数** 实现32位精度：

```c
// 文件位置: tim.c:792-874

int64_t get_encodeA(void)
{
    int16_t counter = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);

    // 32位值 = 溢出次数×65536 + 当前计数器值
    return (int64_t)encodeA_count * 65536 + counter;
}

// 溢出中断处理 (stm32f4xx_it.c:456-462)
if(htim == &htim3)
{
    if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
        encodeA_count--;  // 反转溢出
    else
        encodeA_count++;  // 正转溢出
}
```

#### 4.2.2 速度计算

文件位置: `tim.c:876-912`

```c
float get_speedA_1s(int ms)
{
    static int64_t last_encode = 0;
    int64_t encode = get_encodeA();

    // 计算增量
    int64_t delta = encode - last_encode;
    last_encode = encode;

    // 方向修正
    if(ENCODER_A_REVERSE) delta = -delta;

    // 换算为 转/秒
    // delta * (1000/ms) = 每秒脉冲数
    // / 4 = 正交编码4倍频
    // / 13 = 电机线数
    // / 30 = 减速比
    return (float)delta * (1000.0f / ms) / 4.0f / 13.0f / 30.0f;
}
```

### 4.3 麦克纳姆轮运动学

#### 4.3.1 逆运动学 (速度分解)

文件位置: `main.c:428-444`

```c
void ROS_Cmd_To_Motor_Speed(void)
{
    float vx = ros_cmd.vx;   // X方向线速度 (m/s)
    float vy = ros_cmd.vy;   // Y方向线速度 (m/s)
    float wz = ros_cmd.wz;   // Z轴角速度 (rad/s)

    // L = (轮距 + 轴距) / 2
    float L = (WHEEL_BASE + WHEEL_TRACK) / 2.0f;  // 0.19m

    // 麦克纳姆轮逆运动学方程
    float v_lf = vx - vy - L * wz;  // 左前 (A)
    float v_lr = vx + vy - L * wz;  // 左后 (B)
    float v_rf = vx + vy + L * wz;  // 右前 (C)
    float v_rr = vx - vy + L * wz;  // 右后 (D)

    // 线速度 → 轮子转速 (转/秒)
    targetA = v_lf / WHEEL_PERIMETER;
    targetB = v_lr / WHEEL_PERIMETER;
    targetC = v_rf / WHEEL_PERIMETER;
    targetD = v_rr / WHEEL_PERIMETER;
}
```

#### 4.3.2 正运动学 (里程计)

文件位置: `main.c:177-299`

```c
void Update_Odometry(void)
{
    // 1. 获取四轮速度
    // 2. 正运动学计算
    float vx = (v_lf + v_lr + v_rf + v_rr) / 4.0f;
    float vy = (-v_lf + v_lr + v_rf - v_rr) / 4.0f;
    float wz = (-v_lf - v_lr + v_rf + v_rr) / (4.0f * L);

    // 3. 使用IMU修正角度 (如果启用)
    if(USE_IMU && imu_data.updated)
        odom_data.th = imu_data.yaw;
    else
        odom_data.th += wz * DT;

    // 4. 积分计算位置
    odom_data.x += (vx*cos(th) - vy*sin(th)) * DT;
    odom_data.y += (vx*sin(th) + vy*cos(th)) * DT;
}
```

### 4.4 IMU数据处理

#### 4.4.1 WT901BCTTL数据格式

```
数据包: 11字节
┌──────┬──────┬──────────────────────────┬──────┐
│ 0x55 │ Type │ Data (8 bytes)           │ Sum  │
│ 包头 │ 类型 │ 数据                      │ 校验 │
└──────┴──────┴──────────────────────────┴──────┘

Type类型:
- 0x51: 加速度 (Ax, Ay, Az, T)
- 0x52: 角速度 (Wx, Wy, Wz, T)
- 0x53: 欧拉角 (Roll, Pitch, Yaw, T)

校验: Sum = (Byte[0] + ... + Byte[9]) & 0xFF
```

#### 4.4.2 数据解析

文件位置: `main.c:452-515`

```c
void Parse_IMU_Data(uint8_t *buffer)
{
    // 校验
    uint8_t sum = 0;
    for(int i=0; i<10; i++) sum += buffer[i];
    if(sum != buffer[10]) return;  // 校验失败

    switch(buffer[1])
    {
        case 0x53:  // 欧拉角
            int16_t roll_raw  = (buffer[3]<<8) | buffer[2];
            int16_t pitch_raw = (buffer[5]<<8) | buffer[4];
            int16_t yaw_raw   = (buffer[7]<<8) | buffer[6];

            // 转换为弧度 (量程: ±180°)
            imu_data.roll  = roll_raw  / 32768.0f * PI;
            imu_data.pitch = pitch_raw / 32768.0f * PI;
            imu_data.yaw   = yaw_raw   / 32768.0f * PI;
            break;
        // 其他类型处理...
    }
}
```

### 4.5 电机驱动

文件位置: `tim.c:724-790`

```c
void motor_setspeed(int pwmA, int pwmB, int pwmC, int pwmD)
{
    // 方向修正
    if(MOTOR_A_REVERSE) pwmA = -pwmA;
    // ...

    // A轮控制
    if(pwmA >= 0) {
        HAL_GPIO_WritePin(motorA_IN1_GPIO_Port, motorA_IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motorA_IN2_GPIO_Port, motorA_IN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(motorA_IN1_GPIO_Port, motorA_IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motorA_IN2_GPIO_Port, motorA_IN2_Pin, GPIO_PIN_SET);
        pwmA = -pwmA;
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwmA);

    // B、C、D轮同理...
}
```

---

## 5. 通信协议

### 5.1 ROS指令接收

**格式**: `$vx,vy,wz\r\n`

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| vx | float | m/s | X方向线速度 (前进为正) |
| vy | float | m/s | Y方向线速度 (左移为正) |
| wz | float | rad/s | Z轴角速度 (逆时针为正) |

**示例**:
```
$0.5,0.0,0.0\r\n     # 前进0.5m/s
$0.0,0.3,0.0\r\n     # 左移0.3m/s
$0.0,0.0,0.5\r\n     # 左转0.5rad/s
$0.3,0.2,0.1\r\n     # 复合运动
```

### 5.2 里程计发送 (正常模式)

**格式**: `$x,y,th,vx,vy,wz\r\n`

| 字段 | 类型 | 单位 | 说明 |
|------|------|------|------|
| x | float | m | X位置 |
| y | float | m | Y位置 |
| th | float | rad | 偏航角 |
| vx | float | m/s | X方向速度 |
| vy | float | m/s | Y方向速度 |
| wz | float | rad/s | 角速度 |

**发送频率**: 50Hz (每20ms)

### 5.3 调试模式输出

**格式**: `#encA,encB,encC,encD,imu_cnt,th,x,y,err\r\n`

同时在TIM7中断中输出: `SPD: A=x.xx B=x.xx C=x.xx D=x.xx\r\n`

---

## 6. 参数配置指南

### 6.1 参数配置位置

所有可调参数集中在 `main.h` 中：

```c
// ========== 调试开关 ==========
#define DEBUG_MODE       0    // 0=正常, 1=调试

// ========== 机器人参数 ==========
#define WHEEL_DIAMETER   0.095f   // 轮径 (米)
#define WHEEL_BASE       0.210f   // 轮距 (米)
#define WHEEL_TRACK      0.170f   // 轴距 (米)

// ========== 编码器参数 ==========
#define ENCODER_LINES    13       // 电机线数
#define GEAR_RATIO       30.0f    // 减速比

// ========== 方向修正 ==========
#define MOTOR_A_REVERSE    0      // 电机方向
#define ENCODER_A_REVERSE  0      // 编码器方向

// ========== PID参数 ==========
#define Velcity_Kp     710
#define Velcity_Ki     25
#define Velcity_Kd     9

// ========== 速度斜坡 ==========
#define SPEED_RAMP_RATE  0.1f     // 转/秒·周期
```

### 6.2 PID调参步骤

1. **设置Ki=0, Kd=0**，只调Kp
2. **增大Kp**直到系统开始震荡
3. **将Kp减小到震荡值的60%**
4. **增大Ki**直到稳态误差消除
5. **增大Kd**抑制超调和震荡
6. **微调**三个参数达到最佳效果

### 6.3 方向修正方法

1. 给定正向目标速度
2. 观察轮子实际转向
3. 如果轮子反转，将对应的 `MOTOR_X_REVERSE` 设为1
4. 观察编码器速度显示
5. 如果速度为负，将对应的 `ENCODER_X_REVERSE` 设为1

---

## 7. 调试指南

### 7.1 开启调试模式

修改 `main.h`:
```c
#define DEBUG_MODE  1
```

### 7.2 调试输出内容

1. **USART1 输出** (50Hz):
   ```
   #encA,encB,encC,encD,imu_cnt,th,x,y,err
   ```

2. **TIM7 中断输出** (10Hz):
   ```
   SPD: A=1.23 B=1.23 C=1.23 D=1.23
   ```

### 7.3 常见问题排查

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 电机不转 | PWM未输出 | 检查TIM1初始化 |
| 电机抖动 | PID参数不当 | 降低Kp或增大Kd |
| 速度不准 | 编码器方向错误 | 修改ENCODER_X_REVERSE |
| 转向相反 | 电机方向错误 | 修改MOTOR_X_REVERSE |
| 里程计漂移 | IMU未启用 | 设置USE_IMU=1 |
| 通信超时 | 波特率不匹配 | 确认115200 |

### 7.4 按键测试功能

- **短按**: X轴速度增加0.1m/s，超过1.2m/s清零
- **长按(≥500ms)**: 停止所有电机

---

## 附录: 关键代码行号索引

| 功能 | 文件 | 行号 |
|------|------|------|
| 时钟配置 | main.c | 676-716 |
| 主循环 | main.c | 627-660 |
| ROS指令解析 | main.c | 352-368 |
| 逆运动学 | main.c | 428-444 |
| 里程计计算 | main.c | 177-299 |
| IMU解析 | main.c | 452-515 |
| PID中断 | stm32f4xx_it.c | 408-445 |
| PID函数 | tim.c | 916-1033 |
| 编码器读取 | tim.c | 792-874 |
| 速度计算 | tim.c | 876-912 |
| 电机驱动 | tim.c | 724-790 |
| 速度斜坡 | tim.c | 690-722 |

---

*文档版本: v1.0*
*最后更新: 2026-01-24*
