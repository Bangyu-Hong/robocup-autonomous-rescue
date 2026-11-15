# 串口通信协议解析指南

> 本文档详细介绍下位机与上位机(ROS)之间的串口通信协议，以及IMU通信协议

---

## 目录

1. [通信概述](#1-通信概述)
2. [ROS通信协议](#2-ros通信协议)
3. [IMU通信协议](#3-imu通信协议)
4. [数据流详解](#4-数据流详解)
5. [软件实现](#5-软件实现)
6. [ROS端示例代码](#6-ros端示例代码)
7. [调试与测试](#7-调试与测试)

---

## 1. 通信概述

### 1.1 通信架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                              通信架构图                                  │
└─────────────────────────────────────────────────────────────────────────┘

    ┌──────────────┐         USART1          ┌──────────────┐
    │              │ ◄────────────────────── │              │
    │   ROS        │   里程计数据 (50Hz)      │   STM32      │
    │   上位机     │ ────────────────────►   │   下位机     │
    │              │   速度指令               │              │
    └──────────────┘                         └──────────────┘
                                                    ▲
                                                    │ USART2
                                                    │ IMU数据
                                             ┌──────────────┐
                                             │   WT901      │
                                             │   IMU        │
                                             └──────────────┘
```

### 1.2 串口配置

| 端口 | 功能 | 波特率 | 数据位 | 停止位 | 校验 | 模式 |
|------|------|--------|--------|--------|------|------|
| USART1 | ROS通信 | 115200 | 8 | 1 | 无 | DMA收发 |
| USART2 | IMU通信 | 115200 | 8 | 1 | 无 | 中断接收 |

### 1.3 数据流向

```
【下行数据流 - ROS → STM32】
ROS发布速度指令 → USB转TTL → USART1 RX (DMA) → 解析 → 逆运动学 → 电机控制

【上行数据流 - STM32 → ROS】
编码器+IMU → 里程计计算 → 格式化 → USART1 TX (DMA) → USB转TTL → ROS接收

【IMU数据流】
WT901 → USART2 RX (中断) → 解析 → 融合到里程计
```

---

## 2. ROS通信协议

### 2.1 速度指令协议（ROS → STM32）

#### 数据格式

```
$vx,vy,wz\r\n

起始符: $
分隔符: ,
结束符: \r\n (回车换行)
```

#### 字段说明

| 字段 | 类型 | 单位 | 范围 | 说明 |
|------|------|------|------|------|
| vx | float | m/s | -2.0 ~ 2.0 | X方向线速度（前进为正） |
| vy | float | m/s | -2.0 ~ 2.0 | Y方向线速度（左移为正） |
| wz | float | rad/s | -3.14 ~ 3.14 | Z轴角速度（逆时针为正） |

#### 示例

```
$0.5,0.0,0.0\r\n      // 前进 0.5 m/s
$-0.3,0.0,0.0\r\n     // 后退 0.3 m/s
$0.0,0.2,0.0\r\n      // 左移 0.2 m/s
$0.0,-0.2,0.0\r\n     // 右移 0.2 m/s
$0.0,0.0,0.5\r\n      // 左转 0.5 rad/s
$0.0,0.0,-0.5\r\n     // 右转 0.5 rad/s
$0.3,0.1,0.2\r\n      // 复合运动：前进+左移+左转
$0.0,0.0,0.0\r\n      // 停止
```

#### 坐标系定义

```
        X+ (前进)
         ↑
         │
  Y+ ←───┼───→ Y- (右移)
 (左移)  │
         ↓
        X- (后退)

角速度: 逆时针为正 (俯视，左转为正)
```

### 2.2 里程计协议（STM32 → ROS）

#### 正常模式数据格式

```
$x,y,th,vx,vy,wz\r\n

起始符: $
分隔符: ,
结束符: \r\n
```

#### 字段说明

| 字段 | 类型 | 单位 | 精度 | 说明 |
|------|------|------|------|------|
| x | float | m | 0.001 | X位置 |
| y | float | m | 0.001 | Y位置 |
| th | float | rad | 0.001 | 偏航角（航向角） |
| vx | float | m/s | 0.001 | X方向速度 |
| vy | float | m/s | 0.001 | Y方向速度 |
| wz | float | rad/s | 0.001 | 角速度 |

#### 示例

```
$0.000,0.000,0.000,0.000,0.000,0.000\r\n   // 静止在原点
$1.234,0.567,0.785,0.500,0.000,0.100\r\n   // 运动中
$-0.500,1.200,-1.571,0.000,0.000,0.000\r\n // 到达某位置
```

#### 发送频率

- **默认频率**: 50Hz (每20ms发送一次)
- **可调整**: 修改 main.c 中的定时判断

```c
// main.c 主循环
if(HAL_GetTick() - last_time >= 20)  // 20ms = 50Hz
{
    Update_Odometry();
    Send_GyroData();
}
```

### 2.3 调试模式数据格式

当 `DEBUG_MODE = 1` 时使用此格式：

```
#encA,encB,encC,encD,imu_cnt,th,x,y,err\r\n

起始符: #
分隔符: ,
结束符: \r\n
```

#### 字段说明

| 字段 | 类型 | 说明 |
|------|------|------|
| encA | int | A轮编码器原始值（左前） |
| encB | int | B轮编码器原始值（左后） |
| encC | int | C轮编码器原始值（右前） |
| encD | int | D轮编码器原始值（右后） |
| imu_cnt | uint32 | IMU帧接收计数 |
| th | float | 当前角度 (rad) |
| x | float | X位置 (m) |
| y | float | Y位置 (m) |
| err | uint32 | IMU校验错误计数 |

#### 示例

```
#1234,-567,890,-1234,1000,0.785,1.000,0.500,0\r\n
```

### 2.4 速度调试输出

当 `DEBUG_MODE = 1` 时，额外通过TIM7中断输出速度信息（10Hz）：

```
SPD: A=1.23 B=1.23 C=1.23 D=1.23\r\n
```

---

## 3. IMU通信协议

### 3.1 WT901数据协议

#### 帧格式

```
┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
│ Byte0│ Byte1│ Byte2│ Byte3│ Byte4│ Byte5│ Byte6│ Byte7│ Byte8│ Byte9│Byte10│
├──────┼──────┼──────┼──────┼──────┼──────┼──────┼──────┼──────┼──────┼──────┤
│ 0x55 │ Type │ D0L  │ D0H  │ D1L  │ D1H  │ D2L  │ D2H  │ D3L  │ D3H  │ Sum  │
└──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘

总长度: 11字节
字节序: 小端序 (低字节在前)
校验和: Sum = (Byte0 + Byte1 + ... + Byte9) & 0xFF
```

#### 数据类型

| Type | 名称 | D0 | D1 | D2 | D3 |
|------|------|-----|-----|-----|-----|
| 0x51 | 加速度 | Ax | Ay | Az | Temp |
| 0x52 | 角速度 | Wx | Wy | Wz | Temp |
| 0x53 | 欧拉角 | Roll | Pitch | Yaw | Temp |

#### 数据转换

```c
// 加速度 (m/s²)
accel = raw / 32768.0f * 16.0f * 9.8f;

// 角速度 (rad/s)
gyro = raw / 32768.0f * 2000.0f * PI / 180.0f;

// 欧拉角 (rad)
angle = raw / 32768.0f * PI;
```

### 3.2 WT901配置指令

#### 指令格式

```
┌──────┬──────┬──────┬──────┬──────┐
│ 0xFF │ 0xAA │ Reg  │ DataL│ DataH│
└──────┴──────┴──────┴──────┴──────┘

总长度: 5字节
```

#### 常用指令

| 功能 | 指令序列 | 说明 |
|------|----------|------|
| 解锁 | `FF AA 69 88 B5` | 修改配置前必须执行 |
| 保存 | `FF AA 00 00 00` | 保存到Flash |
| Z轴归零 | `FF AA 01 04 00` | 航向角清零 |
| 加速度校准 | `FF AA 01 01 00` | 水平放置后执行 |
| 恢复出厂 | `FF AA 00 01 00` | 重置所有设置 |

---

## 4. 数据流详解

### 4.1 速度指令处理流程

```
┌─────────────────────────────────────────────────────────────────┐
│                    速度指令处理流程                              │
└─────────────────────────────────────────────────────────────────┘

  ROS发送: $0.5,0.0,0.3\r\n
              │
              ▼
┌─────────────────────────────────────┐
│ USART1 DMA接收 (循环缓冲区50字节)    │
│ ros_rx_buffer[]                     │
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ Parse_ROS_Buffer_DMA()              │
│ - 检测'$'起始符                      │
│ - 检测'\n'结束符                     │
│ - 提取完整指令                       │
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ Parse_ROS_Command()                 │
│ - sscanf解析三个浮点数              │
│ - ros_cmd.vx = 0.5                  │
│ - ros_cmd.vy = 0.0                  │
│ - ros_cmd.wz = 0.3                  │
│ - ros_cmd.updated = 1               │
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ ROS_Cmd_To_Motor_Speed()            │
│ 逆运动学计算:                        │
│ - v_lf = vx - vy - L*wz             │
│ - v_lr = vx + vy - L*wz             │
│ - v_rf = vx + vy + L*wz             │
│ - v_rr = vx - vy + L*wz             │
│ 转换为轮速 (转/秒):                  │
│ - targetA/B/C/D = v / 轮周长        │
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ TIM7中断 (10ms) - PID控制           │
│ - 获取实际速度                       │
│ - 速度斜坡平滑                       │
│ - PID计算PWM                        │
│ - 输出到电机                         │
└─────────────────────────────────────┘
```

### 4.2 里程计发送流程

```
┌─────────────────────────────────────────────────────────────────┐
│                    里程计发送流程                                │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────┐
│ 编码器读取 (TIM2/3/4/8)             │
│ - 定时器计数值                       │
│ - 溢出计数                          │
│ - 计算轮速                          │
└────────────────┬────────────────────┘
                 │
                 ├─────────────────────────┐
                 │                         │
                 ▼                         ▼
┌─────────────────────────┐  ┌─────────────────────────┐
│ 编码器里程计计算        │  │ IMU数据 (USART2)        │
│ - 轮速→机器人速度       │  │ - 角速度                │
│ - 正运动学              │  │ - 欧拉角 (yaw)          │
└────────────────┬────────┘  └────────────┬────────────┘
                 │                         │
                 └────────────┬────────────┘
                              │ 数据融合
                              ▼
┌─────────────────────────────────────┐
│ Update_Odometry() (50Hz)            │
│ - 使用IMU角度 (如果可用)             │
│ - 位置积分                          │
│ - 角度归一化                        │
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ Send_GyroData()                     │
│ - 格式化: $x,y,th,vx,vy,wz\r\n     │
│ - DMA发送                           │
└────────────────┬────────────────────┘
                 │
                 ▼
            USART1 TX → ROS接收
```

### 4.3 IMU数据处理流程

```
┌─────────────────────────────────────────────────────────────────┐
│                    IMU数据处理流程                               │
└─────────────────────────────────────────────────────────────────┘

  WT901发送: 55 53 XX XX XX XX XX XX XX XX SUM
              │
              ▼
┌─────────────────────────────────────┐
│ USART2 中断接收 (单字节)            │
│ HAL_UART_RxCpltCallback()           │
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ 状态机处理                          │
│ State 0: 寻找包头 0x55              │
│ State 1: 接收剩余10字节             │
└────────────────┬────────────────────┘
                 │ 收满11字节
                 ▼
┌─────────────────────────────────────┐
│ Parse_IMU_Data()                    │
│ 1. 校验和验证                       │
│ 2. 判断数据类型                     │
│ 3. 数据转换                         │
│ 4. 软件归零处理                     │
└────────────────┬────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────┐
│ 更新imu_data结构体                  │
│ - roll, pitch, yaw                  │
│ - gyro_x, gyro_y, gyro_z            │
│ - accel_x, accel_y, accel_z         │
│ - updated = 1                       │
└─────────────────────────────────────┘
```

---

## 5. 软件实现

### 5.1 ROS指令解析

```c
// 文件: main.c

// DMA缓冲区解析 (循环缓冲区处理)
void Parse_ROS_Buffer_DMA(void)
{
    static uint16_t last_pos = 0;
    uint16_t curr_pos = ROS_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

    while(last_pos != curr_pos)
    {
        uint8_t ch = ros_rx_buffer[last_pos];

        if(ch == '$') {
            ros_rx_index = 0;  // 新帧开始
        }

        if(ros_rx_index < ROS_RX_BUFFER_SIZE - 1) {
            ros_cmd_buffer[ros_rx_index++] = ch;
        }

        if(ch == '\n') {
            ros_cmd_buffer[ros_rx_index] = '\0';
            Parse_ROS_Command(ros_cmd_buffer);  // 解析完整指令
            ros_rx_index = 0;
        }

        last_pos = (last_pos + 1) % ROS_RX_BUFFER_SIZE;
    }
}

// 指令解析
void Parse_ROS_Command(char *buffer)
{
    if(buffer[0] != '$') return;

    int result = sscanf(buffer + 1, "%f,%f,%f",
                       &ros_cmd.vx,
                       &ros_cmd.vy,
                       &ros_cmd.wz);

    if(result == 3) {
        ros_cmd.updated = 1;
        ros_cmd.last_time = HAL_GetTick();
    }
}
```

### 5.2 里程计发送

```c
// 文件: main.c

void Send_GyroData(void)
{
    static char tx_buffer[150];
    int len;

    // DMA忙检测
    if(huart1.gState != HAL_UART_STATE_READY) {
        return;
    }

#if DEBUG_MODE == 1
    // 调试模式
    len = sprintf(tx_buffer, "#%d,%d,%d,%d,%lu,%.3f,%.3f,%.3f,%lu\r\n",
                  encoder.front_left,
                  encoder.left,
                  encoder.front_right,
                  encoder.right,
                  imu_frame_count,
                  odom_data.th,
                  odom_data.x,
                  odom_data.y,
                  imu_check_err_count);
#else
    // 正常模式
    len = sprintf(tx_buffer, "$%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                  odom_data.x, odom_data.y, odom_data.th,
                  odom_data.vx, odom_data.vy, odom_data.wz);
#endif

    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)tx_buffer, len);
}
```

### 5.3 通信超时保护

```c
// 文件: main.c (主循环)

#define ROS_CMD_TIMEOUT_MS  500  // 500ms超时

// 超时检测
if(ros_cmd.last_time > 0 &&
   (HAL_GetTick() - ros_cmd.last_time > ROS_CMD_TIMEOUT_MS))
{
    // 超时停止电机
    targetA = 0;
    targetB = 0;
    targetC = 0;
    targetD = 0;
}
```

---

## 6. ROS端示例代码

### 6.1 Python节点示例

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32麦轮机器人ROS通信节点
"""

import rospy
import serial
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
import tf

class MecanumRobotNode:
    def __init__(self):
        rospy.init_node('mecanum_robot_node')

        # 串口配置
        self.serial_port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baud_rate = rospy.get_param('~baud', 115200)

        # 打开串口
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)

        # 订阅速度指令
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # 发布里程计
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        # TF广播
        self.tf_broadcaster = tf.TransformBroadcaster()

        # 定时发送里程计
        rospy.Timer(rospy.Duration(0.02), self.read_odom)

        rospy.loginfo("Mecanum robot node started")

    def cmd_vel_callback(self, msg):
        """发送速度指令到STM32"""
        cmd = "${:.3f},{:.3f},{:.3f}\r\n".format(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z
        )
        self.ser.write(cmd.encode())

    def read_odom(self, event):
        """读取并发布里程计"""
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode().strip()

            if line.startswith('$'):
                try:
                    parts = line[1:].split(',')
                    if len(parts) == 6:
                        x = float(parts[0])
                        y = float(parts[1])
                        th = float(parts[2])
                        vx = float(parts[3])
                        vy = float(parts[4])
                        wz = float(parts[5])

                        self.publish_odom(x, y, th, vx, vy, wz)
                except ValueError:
                    pass

    def publish_odom(self, x, y, th, vx, vy, wz):
        """发布里程计消息"""
        current_time = rospy.Time.now()

        # 创建四元数
        odom_quat = quaternion_from_euler(0, 0, th)

        # 发布TF
        self.tf_broadcaster.sendTransform(
            (x, y, 0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # 创建并发布Odometry消息
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = odom_quat[0]
        odom.pose.pose.orientation.y = odom_quat[1]
        odom.pose.pose.orientation.z = odom_quat[2]
        odom.pose.pose.orientation.w = odom_quat[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

    def run(self):
        rospy.spin()
        self.ser.close()

if __name__ == '__main__':
    try:
        node = MecanumRobotNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
```

### 6.2 C++节点示例

```cpp
// mecanum_robot_node.cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <serial/serial.h>
#include <sstream>

class MecanumRobotNode {
public:
    MecanumRobotNode() : nh_("~") {
        // 获取参数
        nh_.param<std::string>("port", port_, "/dev/ttyUSB0");
        nh_.param<int>("baud", baud_, 115200);

        // 打开串口
        try {
            ser_.setPort(port_);
            ser_.setBaudrate(baud_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
            ser_.setTimeout(timeout);
            ser_.open();
        } catch (serial::IOException& e) {
            ROS_ERROR("Unable to open serial port");
            return;
        }

        // 订阅和发布
        cmd_sub_ = nh_.subscribe("/cmd_vel", 10, &MecanumRobotNode::cmdVelCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 10);

        // 定时读取
        timer_ = nh_.createTimer(ros::Duration(0.02), &MecanumRobotNode::readOdom, this);

        ROS_INFO("Mecanum robot node started");
    }

private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        char cmd[64];
        snprintf(cmd, sizeof(cmd), "$%.3f,%.3f,%.3f\r\n",
                 msg->linear.x, msg->linear.y, msg->angular.z);
        ser_.write(cmd);
    }

    void readOdom(const ros::TimerEvent& event) {
        if (ser_.available() > 0) {
            std::string line = ser_.readline();

            if (line[0] == '$') {
                float x, y, th, vx, vy, wz;
                if (sscanf(line.c_str() + 1, "%f,%f,%f,%f,%f,%f",
                           &x, &y, &th, &vx, &vy, &wz) == 6) {
                    publishOdom(x, y, th, vx, vy, wz);
                }
            }
        }
    }

    void publishOdom(float x, float y, float th, float vx, float vy, float wz) {
        ros::Time current_time = ros::Time::now();

        // TF广播
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th);
        tf_broadcaster_.sendTransform(odom_trans);

        // 发布Odometry
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = wz;

        odom_pub_.publish(odom);
    }

    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;
    ros::Publisher odom_pub_;
    ros::Timer timer_;
    tf::TransformBroadcaster tf_broadcaster_;
    serial::Serial ser_;
    std::string port_;
    int baud_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mecanum_robot_node");
    MecanumRobotNode node;
    ros::spin();
    return 0;
}
```

---

## 7. 调试与测试

### 7.1 串口调试工具测试

使用串口助手发送测试指令：

```
# 前进测试
$0.3,0.0,0.0\r\n

# 左移测试
$0.0,0.3,0.0\r\n

# 旋转测试
$0.0,0.0,0.5\r\n

# 停止
$0.0,0.0,0.0\r\n
```

### 7.2 ROS命令行测试

```bash
# 启动节点后，发布速度指令
rostopic pub /cmd_vel geometry_msgs/Twist "linear:
  x: 0.3
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"

# 查看里程计数据
rostopic echo /odom
```

### 7.3 数据校验

检查点清单：

| 检查项 | 正常情况 | 异常处理 |
|--------|----------|----------|
| 串口连接 | 指示灯闪烁 | 检查线路 |
| 数据格式 | 以$开头 | 检查起始符 |
| 数据更新 | 50Hz稳定 | 检查定时器 |
| 角度范围 | -π ~ π | 检查归一化 |
| 速度响应 | 立即响应 | 检查解析 |
| IMU帧计数 | 持续增加 | 检查IMU连接 |

### 7.4 常见问题

| 问题 | 可能原因 | 解决方法 |
|------|----------|----------|
| 收不到数据 | TX/RX接反 | 交换连接 |
| 数据乱码 | 波特率不匹配 | 确认115200 |
| 指令无响应 | 格式错误 | 检查$和\r\n |
| 里程漂移 | IMU未校准 | 执行归零 |
| 延迟大 | DMA未使能 | 检查DMA配置 |

---

## 附录：协议快速参考卡

```
┌────────────────────────────────────────────────────────────────┐
│                    通信协议快速参考                             │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│  【速度指令 ROS→STM32】                                        │
│  格式: $vx,vy,wz\r\n                                          │
│  示例: $0.5,0.0,0.3\r\n                                       │
│  单位: m/s, m/s, rad/s                                        │
│                                                                │
│  【里程计 STM32→ROS】                                          │
│  格式: $x,y,th,vx,vy,wz\r\n                                   │
│  示例: $1.234,0.567,0.785,0.500,0.000,0.100\r\n               │
│  单位: m, m, rad, m/s, m/s, rad/s                             │
│  频率: 50Hz                                                    │
│                                                                │
│  【调试模式】                                                   │
│  格式: #encA,encB,encC,encD,imu_cnt,th,x,y,err\r\n            │
│                                                                │
│  【IMU数据 WT901→STM32】                                       │
│  格式: 55 Type D0L D0H D1L D1H D2L D2H D3L D3H Sum            │
│  长度: 11字节                                                  │
│                                                                │
│  【串口配置】                                                   │
│  USART1 (ROS): 115200, 8N1, DMA                               │
│  USART2 (IMU): 115200, 8N1, 中断                              │
│                                                                │
└────────────────────────────────────────────────────────────────┘
```

---

*文档版本: v1.0*
*最后更新: 2026-01-24*
