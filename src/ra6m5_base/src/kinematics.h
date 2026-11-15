#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include "hal_data.h"

// ============================================================
// 【重要】以下参数需要根据实际硬件测量后修改！
// ============================================================

// --- 机械结构参数 (单位: 米) ---
#define WHEEL_RADIUS      0.0475f   // 车轮半径 (用卡尺测直径÷2)
#define WHEEL_TRACK       0.2100f   // 左右轮中心距离
#define WHEEL_BASE        0.1700f   // 前后轴中心距离

// --- 编码器与电机参数 ---
#define ENCODER_PPR       13       // 编码器每转脉冲数 (查电机规格书)
#define GEAR_RATIO        30.0f       // 减速比 (查电机规格书)
#define ENCODER_MODE      4         // 4倍频计数

// --- 自动计算的参数 (不要手动修改) ---
#define PULSES_PER_REV    (ENCODER_PPR * GEAR_RATIO * ENCODER_MODE)  // 轮子转一圈的总脉冲数
#define CONTROL_PERIOD    0.01f     // 控制周期 10ms

// 脉冲速度 -> 物理速度 (m/s) 的转换系数
// 公式: v(m/s) = 脉冲数/周期 × 2πR / 每转脉冲数 / 周期时间
#define PULSE_TO_MPS      (2.0f * 3.14159265f * WHEEL_RADIUS / (float)PULSES_PER_REV / CONTROL_PERIOD)

// 物理速度 (m/s) -> 脉冲速度 的转换系数
#define MPS_TO_PULSE      (1.0f / PULSE_TO_MPS)

// 运动学参数 (轮距+轴距的一半)
#define KINEMATIC_PARAM   ((WHEEL_TRACK + WHEEL_BASE) / 2.0f)

typedef struct {
    float vx;  
    float vy;  
    float wz;  
    float odom_x;
    float odom_y;
    float odom_yaw;
} Kinematics_t;

extern Kinematics_t robot_odom;

void Kinematics_Update(float v1, float v2, float v3, float v4, float dt);

#endif /* __KINEMATICS_H */  // <--- 文件必须在这里结束！