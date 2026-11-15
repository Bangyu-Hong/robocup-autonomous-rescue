#ifndef __IMU_H
#define __IMU_H

#include "hal_data.h" // 引入瑞萨底层库

// 定义 IMU 数据结构体
typedef struct {
    // 欧拉角 (单位：度 °) —— 底盘导航的核心！
    float roll;  // X轴角度 (横滚)
    float pitch; // Y轴角度 (俯仰)
    float yaw;   // Z轴角度 (航向) 

    // 加速度 (单位：g)
    float acc_x;
    float acc_y;
    float acc_z;
    
    // 角速度 (单位：度/秒 °/s)
    float gyro_x;
    float gyro_y;
    float gyro_z;
} IMU_Data_t;

// 使用 volatile 防止编译器优化，暴露给其他文件使用
extern volatile IMU_Data_t robot_imu;

// ==================== 调试计数器 (用于诊断) ====================
extern volatile uint32_t imu_rx_count;      // 收到的字节总数
extern volatile uint32_t imu_frame_count;   // 成功解析的帧数
extern volatile uint32_t imu_angle_count;   // 收到的角度包数量
extern volatile uint32_t imu_checksum_fail; // 校验和失败次数
extern volatile uint8_t last_frame[11];     // 最后一帧原始数据
extern volatile uint8_t last_calc_sum;      // 计算出的校验和
extern volatile uint8_t last_recv_sum;      // 接收到的校验和

// ==================== 外部函数声明 ====================

// 初始化 IMU (包含上电校准)
void IMU_Init(void);

// 手动触发校准 (加速度校准 + 角度清零)
void IMU_Calibrate(void);

// 仅清零 Z 轴角度 (yaw) - 硬件+软件双重归零
void IMU_ResetYaw(void);

// 仅软件层清零Yaw角度 (不发送硬件指令，快速归零)
void IMU_SoftResetYaw(void);

#endif /* __IMU_H */