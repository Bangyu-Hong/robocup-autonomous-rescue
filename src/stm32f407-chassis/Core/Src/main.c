/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "delay.h"
#include <stdio.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
unsigned int proc_slow;
unsigned char key_flag;
static unsigned char key_val,key_down,key_up,key_old;
unsigned char keytime_flag;
uint16_t keytime_count;
uint8_t rx_temp; 


extern float targetA;
extern float targetB;
extern float targetC;
extern float targetD;

// ======================== 机器人参数配置区 ========================
// 坐标系定义 (符合ROS REP-103标准 / 右手定则):
// X轴: 机器人前方 (正值=前进, 负值=后退)
// Y轴: 机器人左方 (正值=左移, 负值=右移)
// Z轴: 垂直向上
// 角度: 逆时针为正 (正值=左转, 负值=右转)
// 单位: 距离=米(m), 速度=米/秒(m/s), 角度=弧度(rad), 角速度=弧度/秒(rad/s)

#define ROBOT_TYPE  1  // 0=差速驱动, 1=麦克纳姆轮, 2=全向轮

// 轮子参数
#define WHEEL_DIAMETER   0.095f   // 轮子直径 (单位: 米) - 实测值，请确保准确!
#define WHEEL_RADIUS     (WHEEL_DIAMETER / 2.0f)
#define WHEEL_BASE       0.210f   // 轮距: 左右轮中心距离 (单位: 米)
#define WHEEL_TRACK      0.170f   // 轴距: 前后轮中心距离 (单位: 米)

// 编码器参数
// 13线编码器, 30减速比
#define ENCODER_LINES    13       // 电机线数 (13线)
#define GEAR_RATIO       30.0f    // 减速比 (30)
#define PULSES_PER_REV   (ENCODER_LINES * 4 * GEAR_RATIO)  // 轮子转一圈的总脉冲数 (13*4*30 = 1560)

// 计算常数
#define PI               3.14159265358979f
#define WHEEL_PERIMETER  (2.0f * PI * WHEEL_RADIUS)         // 轮子周长
#define METER_PER_PULSE  (WHEEL_PERIMETER / PULSES_PER_REV) // 每个脉冲对应的距离

// 采样周期 (秒) - 与发送频率对应
#define DT               0.02f     // 20ms = 0.02s (50Hz)

// IMU配置
#define USE_IMU          1        // 1=使用IMU修正角度, 0=仅使用编码器
#define IMU_UART         huart2   // IMU连接的串口，请修改为实际串口（如huart2、huart3等）

// 注意：DEBUG_MODE 已移至 main.h 中定义，便于其他文件访问

// ======================== WT901BCTTL IMU数据 ========================
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

// WT901BCTTL接收缓冲区
uint8_t imu_rx_buffer[11];  // WT901BCTTL单帧数据11字节
uint8_t imu_rx_state = 0;   // 接收状态机
uint8_t imu_rx_index = 0;   // 接收索引
uint32_t imu_frame_count = 0; // IMU帧接收计数（用于调试）
uint32_t imu_check_err_count = 0; // IMU校验错误计数

// ======================== IMU超时保护 ========================
volatile uint32_t imu_last_rx_time = 0;    // 上次接收IMU数据的时间戳
#define IMU_RX_TIMEOUT_MS  50              // IMU接收超时时间（ms），超过则重置状态机

// ======================== IMU角度归零相关 ========================
float yaw_offset = 0.0f;           // Yaw角度偏移量（软件归零用）
uint8_t yaw_calibrated = 0;        // 标记是否已校准（0=未校准，1=已校准）
#define YAW_CALIBRATION_FRAMES 10  // 等待10帧IMU数据后进行校准

// ======================== ROS速度指令接收 ========================
// 格式: $vx,vy,wz\r\n
typedef struct {
    float vx;         // 目标X方向线速度 (m/s)
    float vy;         // 目标Y方向线速度 (m/s)
    float wz;         // 目标Z轴角速度 (rad/s)
    uint8_t updated;  // 数据更新标志
    uint32_t last_time; // 上次接收时间（用于超时保护）
} ROS_Cmd_t;

ROS_Cmd_t ros_cmd = {0};

// ======================== 本地测试速度（按键控制）========================
float test_vx = 0.0f;  // 按键控制的X轴测试速度 (m/s)

// 通信超时时间（毫秒）- 超过此时间未收到指令则停止电机
#define ROS_CMD_TIMEOUT_MS  500

// ROS指令接收缓冲区
#define ROS_RX_BUFFER_SIZE 50
uint8_t ros_rx_buffer[ROS_RX_BUFFER_SIZE];
uint8_t ros_rx_index = 0;
uint8_t ros_rx_byte;  // 单字节接收缓冲

// ======================== IDLE中断相关 ========================
volatile uint16_t ros_dma_last_pos = 0;  // DMA上次读取位置（volatile防止优化）
volatile uint8_t ros_data_ready = 0;     // 数据就绪标志（IDLE中断触发时置1）

// ======================== 编码器数据 ========================
typedef struct {
    int32_t left;         // 左轮编码器计数
    int32_t right;        // 右轮编码器计数
    int32_t front_left;   // 左前轮编码器（麦克纳姆轮/全向轮）
    int32_t front_right;  // 右前轮编码器（麦克纳姆轮/全向轮）
    int32_t last_left;    // 上次左轮计数
    int32_t last_right;   // 上次右轮计数
    int32_t last_front_left;
    int32_t last_front_right;
} Encoder_t;

Encoder_t encoder = {0};

// ======================== 里程计数据 ========================
typedef struct {
    float x;     // 累积坐标X (m)
    float y;     // 累积坐标Y (m)
    float th;    // 偏航角Yaw (rad)
    float vx;    // X方向线速度 (m/s)
    float vy;    // Y方向线速度 (m/s)
    float wz;    // Z轴角速度 (rad/s)
    float roll;  // 横滚角 (rad)
    float pitch; // 俯仰角 (rad)
    float wx;    // X轴角速度 (rad/s)
    float wy;    // Y轴角速度 (rad/s)
} OdomData_t;

OdomData_t odom_data = {0};

// ======================== 里程计计算函数 ========================
void Update_Odometry(void)
{
    // 使用tim.c中的函数获取编码器值（已处理溢出）
    // 实际通道布局: A=左前(LF), B=右前(RF), C=左后(LR), D=右后(RR)
    encoder.front_left = get_encodeA();    // 左前轮 (A=LF)
    encoder.left = get_encodeC();          // 左后轮 (C=LR)
    encoder.front_right = get_encodeB();   // 右前轮 (B=RF)
    encoder.right = get_encodeD();         // 右后轮 (D=RR)

    // 计算编码器增量 (左后、右后)
    int32_t delta_left = encoder.left - encoder.last_left;
    int32_t delta_right = encoder.right - encoder.last_right;

    // 保存当前值作为下次的上次值
    encoder.last_left = encoder.left;
    encoder.last_right = encoder.right;

#if ROBOT_TYPE == 0  // 差速驱动机器人
    // 计算左右轮移动距离
    float dist_left = delta_left * METER_PER_PULSE;
    float dist_right = delta_right * METER_PER_PULSE;

    // 计算机器人中心移动距离
    float dist_center = (dist_left + dist_right) / 2.0f;

#if USE_IMU == 1
    // 使用IMU的yaw角作为机器人角度（更精确）
    if(imu_data.updated) {
        odom_data.th = imu_data.yaw;
        odom_data.wz = imu_data.gyro_z;  // 使用IMU的角速度
        odom_data.roll = imu_data.roll;   // 横滚角
        odom_data.pitch = imu_data.pitch; // 俯仰角
        odom_data.wx = imu_data.gyro_x;   // X轴角速度
        odom_data.wy = imu_data.gyro_y;   // Y轴角速度
        imu_data.updated = 0;
    }

    // 使用IMU角度更新位置
    float delta_x = dist_center * cosf(odom_data.th);
    float delta_y = dist_center * sinf(odom_data.th);

    odom_data.x += delta_x;
    odom_data.y += delta_y;

    // 计算线速度
    odom_data.vx = dist_center / DT;
    odom_data.vy = 0.0f;  // 差速驱动没有侧向速度
    // wz已经从IMU获取

#else
    // 仅使用编码器计算（无IMU）
    float delta_theta = (dist_right - dist_left) / WHEEL_BASE;

    // 更新位置（使用中点法，更精确）
    float delta_x = dist_center * cosf(odom_data.th + delta_theta / 2.0f);
    float delta_y = dist_center * sinf(odom_data.th + delta_theta / 2.0f);

    odom_data.x += delta_x;
    odom_data.y += delta_y;
    odom_data.th += delta_theta;

    // 角度归一化到 [-π, π]
    while(odom_data.th > PI) odom_data.th -= 2.0f * PI;
    while(odom_data.th < -PI) odom_data.th += 2.0f * PI;

    // 计算速度
    odom_data.vx = dist_center / DT;
    odom_data.vy = 0.0f;  // 差速驱动没有侧向速度
    odom_data.wz = delta_theta / DT;
#endif

#elif ROBOT_TYPE == 1  // 麦克纳姆轮机器人
    // 获取四个轮子的编码器增量
    // 逻辑轮位: LF=A, LR=C, RF=B, RR=D
    int32_t delta_fl = encoder.front_left - encoder.last_front_left;   // 左前
    int32_t delta_fr = encoder.front_right - encoder.last_front_right; // 右前

    encoder.last_front_left = encoder.front_left;
    encoder.last_front_right = encoder.front_right;

    // 计算各轮移动距离
    float dist_lf = delta_fl * METER_PER_PULSE;      // 左前 (A)
    float dist_lr = delta_left * METER_PER_PULSE;    // 左后 (C)
    float dist_rf = delta_fr * METER_PER_PULSE;      // 右前 (B)
    float dist_rr = delta_right * METER_PER_PULSE;   // 右后 (D)

    // 麦克纳姆轮正运动学 (Forward Kinematics)
    // 逻辑轮位: LF=A, LR=C, RF=B, RR=D
    float vx_local = (dist_lf + dist_lr + dist_rf + dist_rr) / 4.0f / DT;
    float vy_local = (-dist_lf + dist_lr + dist_rf - dist_rr) / 4.0f / DT;
    float wz_local = (-dist_lf - dist_lr + dist_rf + dist_rr) / (2.0f * (WHEEL_BASE + WHEEL_TRACK)) / DT;

#if USE_IMU == 1
    // 使用IMU的yaw角作为机器人角度（更精确）
    if(imu_data.updated) {
        odom_data.th = imu_data.yaw;
        odom_data.wz = imu_data.gyro_z;  // 使用IMU的角速度
        odom_data.roll = imu_data.roll;   // 横滚角
        odom_data.pitch = imu_data.pitch; // 俯仰角
        odom_data.wx = imu_data.gyro_x;   // X轴角速度
        odom_data.wy = imu_data.gyro_y;   // Y轴角速度
        imu_data.updated = 0;
    } else {
        // IMU数据未更新时，使用编码器计算的角速度
        odom_data.wz = wz_local;
        odom_data.th += odom_data.wz * DT;
    }
#else
    // 仅使用编码器计算角度
    odom_data.wz = wz_local;
    odom_data.th += odom_data.wz * DT;
#endif

    // 转换到全局坐标系
    odom_data.vx = vx_local * cosf(odom_data.th) - vy_local * sinf(odom_data.th);
    odom_data.vy = vx_local * sinf(odom_data.th) + vy_local * cosf(odom_data.th);

    // 更新位置
    odom_data.x += odom_data.vx * DT;
    odom_data.y += odom_data.vy * DT;

    // 角度归一化
    while(odom_data.th > PI) odom_data.th -= 2.0f * PI;
    while(odom_data.th < -PI) odom_data.th += 2.0f * PI;

#elif ROBOT_TYPE == 2  // 全向轮机器人（三轮120°布局）
    // 根据实际布局计算，此处为示例
    // 请根据您的全向轮布局修改计算公式

#endif
}

// ======================== 发送里程计数据函数 ========================
// 格式: $x,y,th,vx,vy,wz\r\n
void Send_GyroData(void)
{
    static char tx_buffer[200]; // 使用static确保DMA发送期间内存有效
    int len;

    // DMA忙检测：如果上次发送未完成，跳过本次发送
    if(huart1.gState != HAL_UART_STATE_READY) {
        return;  // 上次DMA发送未完成，跳过
    }

#if DEBUG_MODE == 1
    // 调试模式：输出编码器原始值 + IMU帧计数 + 里程计 + IMU姿态
    // 格式: #encA(LF),encB(LR),encC(RF),encD(RR),imu_cnt,th,x,y,check_err,roll,pitch,wx,wy\r\n
    len = sprintf(tx_buffer, "#%d,%d,%d,%d,%lu,%.3f,%.3f,%.3f,%lu,%.3f,%.3f,%.3f,%.3f\r\n",
                  encoder.front_left,   // 编码器A原始值 (左前)
                  encoder.left,         // 编码器B原始值 (左后)
                  encoder.front_right,  // 编码器C原始值 (右前)
                  encoder.right,        // 编码器D原始值 (右后)
                  imu_frame_count,      // IMU帧接收计数
                  odom_data.th,         // 偏航角
                  odom_data.x,          // X位置
                  odom_data.y,          // Y位置
                  imu_check_err_count,  // 校验错误计数
                  odom_data.roll,       // 横滚角
                  odom_data.pitch,      // 俯仰角
                  odom_data.wx,         // X轴角速度
                  odom_data.wy);        // Y轴角速度
#else
    // 正常模式：格式化里程计数据为ROS标准格式（使用\r\n作为结束符）
    // 格式: $x,y,th,vx,vy,wz,roll,pitch,wx,wy\r\n
    len = sprintf(tx_buffer, "$%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                  odom_data.x, odom_data.y, odom_data.th,
                  odom_data.vx, odom_data.vy, odom_data.wz,
                  odom_data.roll, odom_data.pitch,
                  odom_data.wx, odom_data.wy);
#endif

    // 通过USART1 DMA发送数据到ROS
    // 注意：使用DMA发送可以减少CPU占用，防止阻塞其他高优先级任务（如PID中断）
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)tx_buffer, len);
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ======================== ROS速度指令解析函数 ========================
// 解析ROS速度指令：$vx,vy,wz\r\n
void Parse_ROS_Command(char *buffer)
{
    // 检查起始符
    if(buffer[0] != '$') return;

    // 使用sscanf解析三个浮点数
    int result = sscanf(buffer + 1, "%f,%f,%f",
                       &ros_cmd.vx,
                       &ros_cmd.vy,
                       &ros_cmd.wz);

    // 解析成功（读取到3个数据）
    if(result == 3) {
        ros_cmd.updated = 1;  // 标记数据已更新
        ros_cmd.last_time = HAL_GetTick();  // 记录接收时间（用于超时保护）
    }
}

extern DMA_HandleTypeDef hdma_usart1_rx;

// ======================== ROS数据DMA处理函数（优化版）========================
// 修复：正确处理DMA循环缓冲区回绕
// 提升：由IDLE中断触发，提高实时性
void Parse_ROS_Buffer_DMA(void)
{
    // 获取DMA当前传输位置 (NDTR寄存器存储的是剩余传输量)
    uint16_t curr_pos = ROS_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

    // 获取上次读取位置（使用volatile变量，可在IDLE中断中更新）
    uint16_t last_pos = ros_dma_last_pos;

    if (curr_pos == last_pos) return;  // 无新数据

    // 简单的协议解析状态机（static变量保持状态）
    static char line_buf[64];
    static uint8_t line_idx = 0;

    // ★★★ 修复循环缓冲区回绕逻辑 ★★★
    // 计算需要处理的数据量
    uint16_t data_len;
    if (curr_pos >= last_pos) {
        // 正常情况：没有回绕
        data_len = curr_pos - last_pos;
    } else {
        // 回绕情况：curr_pos < last_pos
        data_len = ROS_RX_BUFFER_SIZE - last_pos + curr_pos;
    }

    // 处理新接收的数据
    for (uint16_t i = 0; i < data_len; i++)
    {
        uint8_t data = ros_rx_buffer[last_pos];

        // 更新读取位置（循环缓冲区）
        last_pos++;
        if(last_pos >= ROS_RX_BUFFER_SIZE) last_pos = 0;

        // 协议解析状态机
        if (data == '$') {
            line_idx = 0;
            line_buf[line_idx++] = data;
        } else if (line_idx > 0) {
            if (data == '\n') { // 指令结束
                 line_buf[line_idx] = 0; // 字符串结束符
                 Parse_ROS_Command(line_buf); // 调用解析函数
                 line_idx = 0;
            } else if (line_idx < 63) {
                 line_buf[line_idx++] = data;
            } else {
                 line_idx = 0; // 缓冲区溢出，重置
            }
        }
    }

    // 更新全局位置（供下次使用）
    ros_dma_last_pos = last_pos;
}

// ======================== ROS速度指令转电机目标速度 ========================
// 将ROS的vx, vy, wz转换为四个麦克纳姆轮的目标速度
void ROS_Cmd_To_Motor_Speed(void)
{
    if(!ros_cmd.updated) return;  // 没有新指令，不处理

#if ROBOT_TYPE == 0  // 差速驱动
    // 差速驱动运动学：vx=线速度, wz=角速度
    float v_left = ros_cmd.vx - (WHEEL_BASE / 2.0f) * ros_cmd.wz;
    float v_right = ros_cmd.vx + (WHEEL_BASE / 2.0f) * ros_cmd.wz;

    // 转换为轮子转速（转/秒）
    targetA = v_left / WHEEL_PERIMETER;
    targetB = v_right / WHEEL_PERIMETER;
    targetC = 0;
    targetD = 0;

#elif ROBOT_TYPE == 1  // 麦克纳姆轮
    // 麦克纳姆轮逆运动学 (Inverse Kinematics)
    // vx, vy, wz -> 四个轮子的线速度
    // 实际通道布局: A=左前(LF), B=右前(RF), C=左后(LR), D=右后(RR)
    float L = (WHEEL_BASE + WHEEL_TRACK) / 2.0f;  // 半轮距

    float v_lf = ros_cmd.vx - ros_cmd.vy - L * ros_cmd.wz;  // 左前 (A)
    float v_rf = ros_cmd.vx + ros_cmd.vy + L * ros_cmd.wz;  // 右前 (B)
    float v_lr = ros_cmd.vx + ros_cmd.vy - L * ros_cmd.wz;  // 左后 (C)
    float v_rr = ros_cmd.vx - ros_cmd.vy + L * ros_cmd.wz;  // 右后 (D)

    // 转换为轮子转速（转/秒）
    targetA = v_lf / WHEEL_PERIMETER;  // 左前 (A=LF)
    targetB = v_rf / WHEEL_PERIMETER;  // 右前 (B=RF)
    targetC = v_lr / WHEEL_PERIMETER;  // 左后 (C=LR)
    targetD = v_rr / WHEEL_PERIMETER;  // 右后 (D=RR)

#endif

    ros_cmd.updated = 0;  // 清除更新标志
}

// ======================== WT901BCTTL IMU数据解析函数 ========================
// 解析WT901BCTTL数据包
void Parse_IMU_Data(uint8_t *data)
{
    uint8_t sum = 0;

    // 校验和计算（前10字节相加）
    for(int i = 0; i < 10; i++) {
        sum += data[i];
    }

    // 校验不通过，丢弃数据
    if(sum != data[10]) {
        imu_check_err_count++;
        return;
    }

    // 根据数据类型解析
    switch(data[1])
    {
        case 0x51: // 加速度数据
        {
            int16_t ax = (int16_t)(data[3] << 8 | data[2]);
            int16_t ay = (int16_t)(data[5] << 8 | data[4]);
            int16_t az = (int16_t)(data[7] << 8 | data[6]);

            // 转换为m/s²（量程16g）
            imu_data.accel_x = ax / 32768.0f * 16.0f * 9.8f;
            imu_data.accel_y = ay / 32768.0f * 16.0f * 9.8f;
            imu_data.accel_z = az / 32768.0f * 16.0f * 9.8f;
            break;
        }

        case 0x52: // 角速度数据
        {
            int16_t gx = (int16_t)(data[3] << 8 | data[2]);
            int16_t gy = (int16_t)(data[5] << 8 | data[4]);
            int16_t gz = (int16_t)(data[7] << 8 | data[6]);

            // 转换为rad/s（量程2000°/s）
            imu_data.gyro_x = gx / 32768.0f * 2000.0f * PI / 180.0f;
            imu_data.gyro_y = gy / 32768.0f * 2000.0f * PI / 180.0f;
            imu_data.gyro_z = gz / 32768.0f * 2000.0f * PI / 180.0f;
            break;
        }

        case 0x53: // 角度数据（欧拉角 - 最重要）
        {
            int16_t roll = (int16_t)(data[3] << 8 | data[2]);
            int16_t pitch = (int16_t)(data[5] << 8 | data[4]);
            int16_t yaw = (int16_t)(data[7] << 8 | data[6]);

            // 转换为rad（量程180°）
            imu_data.roll = roll / 32768.0f * PI;
            imu_data.pitch = pitch / 32768.0f * PI;
            float raw_yaw = yaw / 32768.0f * PI;

            // 软件归零：等待足够帧数后自动记录初始角度作为偏移量
            if(!yaw_calibrated && imu_frame_count >= YAW_CALIBRATION_FRAMES) {
                yaw_offset = raw_yaw;
                yaw_calibrated = 1;
            }

            // 应用偏移量，得到归零后的角度
            imu_data.yaw = raw_yaw - yaw_offset;

            // 角度归一化到 [-π, π]
            while(imu_data.yaw > PI) imu_data.yaw -= 2.0f * PI;
            while(imu_data.yaw < -PI) imu_data.yaw += 2.0f * PI;

            imu_data.updated = 1;  // 标记数据已更新
            imu_frame_count++;     // 调试：计数IMU帧
            break;
        }

        default:
            break;
    }
}

// ======================== WT901陀螺仪归零函数 ========================
// 软件归零：重新记录当前角度为偏移量
void IMU_Software_Reset_Yaw(void)
{
    yaw_calibrated = 0;  // 清除校准标志，下次收到IMU数据时重新校准
    yaw_offset = 0.0f;
    odom_data.th = 0.0f; // 同时清零里程计角度
    odom_data.x = 0.0f;  // 清零位置
    odom_data.y = 0.0f;
}

// 硬件归零：向WT901发送Z轴归零指令
void IMU_Hardware_Reset_Yaw(void)
{
    // WT901 Z轴角度归零指令
    // 步骤1: 解锁 (FF AA 69 88 B5)
    uint8_t unlock_cmd[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    HAL_UART_Transmit(&IMU_UART, unlock_cmd, 5, 100);
    HAL_Delay(50);  // 等待解锁完成

    // 步骤2: 设置Z轴角度归零 (FF AA 01 04 00)
    // 寄存器0x01，写入0x0004表示Z轴角度归零
    uint8_t zero_cmd[5] = {0xFF, 0xAA, 0x01, 0x04, 0x00};
    HAL_UART_Transmit(&IMU_UART, zero_cmd, 5, 100);
    HAL_Delay(50);

    // 步骤3: 保存配置 (FF AA 00 00 00) - 可选，保存到Flash
    // uint8_t save_cmd[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
    // HAL_UART_Transmit(&IMU_UART, save_cmd, 5, 100);

    // 同时执行软件归零，确保立即生效
    yaw_calibrated = 0;
    yaw_offset = 0.0f;
    odom_data.th = 0.0f;
    odom_data.x = 0.0f;
    odom_data.y = 0.0f;
}


extern float test_vx;  // 本地测试速度

void key_proc(void)
{
    if(key_flag!=0)return;
    key_flag=1;
    key_val = key_scan();
    key_down = key_val &(key_old^key_val);
    key_up = ~key_val&(key_old^key_val);
    key_old = key_val;

    if(key_down)
    {
        keytime_flag = 1;  // 计时开始
    }
    if(keytime_count<500)  // 短按
        {
            if(key_up)  // 按键抬起
            {
                keytime_flag = keytime_count = 0;  // 状态位清零

                // 新功能：X轴速度增加0.2m/s，到1m/s后清零
                test_vx += 0.1f;
                if(test_vx > 1.2f) test_vx = 0.0f;

                // 将X轴速度转换为四轮目标速度（麦克纳姆轮前进运动）
                // 轮子布局: A=左前(LF), B=左后(LR), C=右前(RF), D=右后(RR)
                // 前进时四轮同向同速
                float wheel_speed = test_vx / WHEEL_PERIMETER;  // 转换为 转/秒
								targetA = wheel_speed;  // 左前
								targetB = wheel_speed;  // 左后
                targetC = wheel_speed;  // 右前
                targetD = wheel_speed;  // 右后
            }
        }
        else  // 长按（≥500ms）
        {
            if(key_old)  // 按住时
            {
                // 长按停止所有电机
                test_vx = 0.0f;
                targetA = 0;
                targetB = 0;
                targetC = 0;
                targetD = 0;
            }
            if(key_up)  // 按键抬起
            {
                keytime_flag = keytime_count = 0;  // 状态位清零

                // 长按抬起时触发IMU硬件归零 + 里程计清零
                IMU_Hardware_Reset_Yaw();
            }
        }
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  MX_TIM7_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();  // ★★★ TIM2 用于 C轮编码器（16位模式）★★★
  MX_TIM5_Init();  // ★★★ TIM5 已禁用 ★★★
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    delay_init(168);

    // 启动USART1串口接收（接收ROS速度指令）
    HAL_UART_Receive_DMA(&huart1, ros_rx_buffer, ROS_RX_BUFFER_SIZE);

    // ★★★ 启用USART1 IDLE中断，提高ROS指令接收实时性 ★★★
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    // 启动IMU串口接收（中断模式）
    HAL_UART_Receive_IT(&IMU_UART, &rx_temp, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_time = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    key_proc();

    // ★★★ ROS指令处理（IDLE中断触发为主，轮询为辅）★★★
    // IDLE中断会在空闲时立即调用 Parse_ROS_Buffer_DMA()
    // 这里保留轮询作为备份，确保数据不会丢失
    Parse_ROS_Buffer_DMA();

    // ★★★ 处理ROS速度指令（立即应用，提高实时性）★★★
    // 当收到新指令时，updated标志会被置1
    // IDLE中断已经调用过一次，这里再检查确保不遗漏
    if(ros_cmd.updated) {
        ROS_Cmd_To_Motor_Speed();
    }

    // ======================== 通信超时保护 ========================
    // 如果超过500ms未收到ROS指令，自动停止电机（安全保护）
    if(ros_cmd.last_time > 0 && (HAL_GetTick() - ros_cmd.last_time > ROS_CMD_TIMEOUT_MS)) {
        targetA = 0;
        targetB = 0;
        targetC = 0;
        targetD = 0;
        // 注意：不重置last_time，这样会持续保持停止状态直到收到新指令
    }

    // 每20ms执行一次里程计更新和发送 (50Hz)
    if(HAL_GetTick() - last_time >= 20)
    {
        last_time = HAL_GetTick();

        // 更新里程计数据
        Update_Odometry();

        // 发送里程计数据到ROS
        Send_GyroData();
    }


//      targetA = 4;
//      targetB = 4;
//      targetC = 4;
//      targetD = 4;
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// ======================== USART1 IDLE中断处理（ROS通信）========================
// 当串口空闲时触发，立即处理接收到的数据
void USART1_IDLE_Handler(void)
{
    // 清除IDLE标志位
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);

    // 立即处理DMA缓冲区中的数据
    Parse_ROS_Buffer_DMA();

    // 如果有新指令，立即应用到电机（提高实时性）
    if(ros_cmd.updated) {
        ROS_Cmd_To_Motor_Speed();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // ======================== USART1：DMA模式接收，由IDLE中断处理 ========================
    // Parse_ROS_Buffer_DMA() 现在由IDLE中断触发，不再依赖主循环轮询

    // ======================== USART2：接收IMU数据（带超时保护）========================
    if(huart->Instance == IMU_UART.Instance)
    {
        uint32_t current_time = HAL_GetTick();

        // ★★★ 超时保护：如果超过50ms没收到完整帧，重置状态机 ★★★
        if(imu_rx_state != 0 && (current_time - imu_last_rx_time > IMU_RX_TIMEOUT_MS)) {
            imu_rx_state = 0;
            imu_rx_index = 0;
        }

        // 更新接收时间戳
        imu_last_rx_time = current_time;

        switch(imu_rx_state)
        {
            case 0: // 寻找包头
                if(rx_temp == 0x55) {
                    imu_rx_buffer[0] = 0x55;
                    imu_rx_state = 1;
                    imu_rx_index = 1;
                }
                // ★★★ 帧同步恢复：非0x55数据直接丢弃，继续等待包头 ★★★
                break;

            case 1: // 接收数据
                imu_rx_buffer[imu_rx_index++] = rx_temp;
                if(imu_rx_index >= 11) {
                    // 接收完成，解析数据
                    Parse_IMU_Data(imu_rx_buffer);
                    imu_rx_state = 0;
                    imu_rx_index = 0;
                }
                // ★★★ 帧同步恢复：检查第2字节是否为有效类型码 ★★★
                else if(imu_rx_index == 2) {
                    // WT901有效类型码：0x50-0x5F
                    if(rx_temp < 0x50 || rx_temp > 0x5F) {
                        // 无效类型码，可能是帧错位，重新寻找包头
                        imu_rx_state = 0;
                        imu_rx_index = 0;
                        // 如果当前字节是0x55，作为新帧的包头
                        if(rx_temp == 0x55) {
                            imu_rx_buffer[0] = 0x55;
                            imu_rx_state = 1;
                            imu_rx_index = 1;
                        }
                    }
                }
                break;
        }

        // 继续接收下一个字节
        HAL_UART_Receive_IT(&IMU_UART, &rx_temp, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
