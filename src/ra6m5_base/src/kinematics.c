#include "kinematics.h"
#include <math.h>

Kinematics_t robot_odom = {0};

/**
 * @brief  运动学更新 (每 10ms 调用一次)
 * @param  v1,v2,v3,v4: 四个轮子的真实速度 (单位: 脉冲/周期)
 *         v1=左前A, v2=左后B, v3=右前C, v4=右后D
 * @param  dt: 计算周期 (单位: 秒, 如 0.01)
 *
 * @note   麦轮布局 (X型，滚轴朝外):
 *             前
 *         A ╲  ╱ C
 *           ╲╱
 *           ╱╲
 *         B ╱  ╲ D
 *             后
 */
void Kinematics_Update(float v1, float v2, float v3, float v4, float dt)
{
    // 1. 将脉冲速度转换为物理线速度 (m/s)
    // 使用 kinematics.h 中自动计算的转换系数
    float vA = v1 * PULSE_TO_MPS;
    float vB = v2 * PULSE_TO_MPS;
    float vC = v3 * PULSE_TO_MPS;
    float vD = v4 * PULSE_TO_MPS;

    // 2. 麦克纳姆轮正解公式 (解算出小车坐标系下的 vx, vy, wz)
    // 注意：此公式对应X型麦轮布局，与逆解公式配套
    robot_odom.vx = (vA + vB + vC + vD) / 4.0f;
    robot_odom.vy = (-vA + vB - vC + vD) / 4.0f;
    robot_odom.wz = (-vA + vB + vC - vD) / (4.0f * KINEMATIC_PARAM);

    // 3. 里程计积分 (世界坐标系转换)
    // 利用当前航向角 robot_odom.odom_yaw 将速度投影到世界 X, Y 轴
    float cos_yaw = cosf(robot_odom.odom_yaw);
    float sin_yaw = sinf(robot_odom.odom_yaw);

    float delta_x = (robot_odom.vx * cos_yaw - robot_odom.vy * sin_yaw) * dt;
    float delta_y = (robot_odom.vx * sin_yaw + robot_odom.vy * cos_yaw) * dt;

    robot_odom.odom_x += delta_x;
    robot_odom.odom_y += delta_y;
    // robot_odom.odom_yaw 由 IMU 提供，不在这里积分
}