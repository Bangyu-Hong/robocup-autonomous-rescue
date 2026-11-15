#include "hal_data.h"
#include "motor.h"
#include "pid.h"
#include <stdio.h>
#include <math.h>
#include "imu.h"
#include "kinematics.h"

#define KEY_PIN BSP_IO_PORT_00_PIN_00

// ==========================================
// 串口发送函数
// ==========================================
// 发送单个字符
void UART_SendChar(uint8_t c)
{
    R_SCI_UART_Write(&g_uart_ros_ctrl, &c, 1);
    while (g_uart_ros_ctrl.p_reg->SSR_b.TEND == 0);
}

// 发送字符串
void UART_SendString(const char *str)
{
    while (*str) {
        UART_SendChar((uint8_t)*str++);
    }
}

// printf 重定向 (ARM Compiler 6)
int fputc(int ch, FILE *f)
{
    (void)f;
    UART_SendChar((uint8_t)ch);
    return ch;
}

// ==========================================
// 全局状态变量区
// ==========================================
// 运行标志位 (受按键和上位机指令控制)
volatile uint8_t run_flag = 0;

// 10ms 定时器心跳标志位 & 发送计数器
volatile uint8_t flag_10ms = 0;
volatile uint16_t send_cnt = 0; 

// 在文件上方，和其他全局变量放在一起
volatile int16_t pwm_A, pwm_B, pwm_C, pwm_D;

// 实例化 4 个增量式 PID 大脑
IncPID_TypeDef pid_A, pid_B, pid_C, pid_D;

// 全局速度与脉冲变量 (必须用 int16_t)
volatile int16_t count_A = 0, count_B = 0, count_C = 0, count_D = 0;
int16_t last_count_A = 0, last_count_B = 0, last_count_C = 0, last_count_D = 0;
int16_t speed_A = 0, speed_B = 0, speed_C = 0, speed_D = 0;

// 滤波后的当前速度 (脉冲/10ms)
volatile float filt_speed_A = 0.0f, filt_speed_B = 0.0f, filt_speed_C = 0.0f, filt_speed_D = 0.0f;

// 目标速度 (脉冲/10ms) - 由上位机指令逆解而来
volatile float target_speed_A = 0.0f, target_speed_B = 0.0f, target_speed_C = 0.0f, target_speed_D = 0.0f;

// ==========================================
// ROS 通信大脑区
// ==========================================
char ros_rx_buf[64];               // 接收缓存
volatile uint8_t ros_rx_index = 0; // 当前接收索引
volatile uint8_t ros_cmd_ready = 0;// 收到完整命令标志
volatile uint16_t ros_timeout_cnt = 0; // 命令超时看门狗计数器

// 速度转换系数现在由 kinematics.h 中的 MPS_TO_PULSE 自动计算
// 不再需要手动设置 VEL_TO_PULSE 

// ==========================================
// 中断回调函数区
// ==========================================

// 1. 10ms 节拍器中断回调
void timer_10ms_callback(timer_callback_args_t *p_args)
{
    if (TIMER_EVENT_CYCLE_END == p_args->event)
    {
        flag_10ms = 1; 
    }
}

// 2. ROS 上位机串口接收中断回调 (FSP 里配给上位机的 UART 绑定此函数)
// 注意：需要在 FSP 中配置好 g_uart_ros 后才能使用
static uint8_t ros_rx_byte; // 用于串口接收的临时变量

void ros_uart_callback(uart_callback_args_t *p_args)
{
    if (p_args->event == UART_EVENT_RX_CHAR)
    {
        char c = (char)p_args->data;

        if (c == '\n') // 收到换行符，说明一帧结束
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

        // 【重要】重新启动下一次接收，否则只能收一个字节！
        //R_SCI_UART_Read(&g_uart_ros_ctrl, &ros_rx_byte, 1);
    }
}

// ==========================================
// 辅助函数区
// ==========================================
// 非阻塞式按键扫描
uint8_t Key_Scan(void)
{
    static uint8_t last_key = 1;
    static uint8_t debounce_cnt = 0;
    bsp_io_level_t key;

    R_IOPORT_PinRead(&g_ioport_ctrl, KEY_PIN, &key);

    if (key == 0) // 按下
    {
        if (last_key == 1) 
        {
            debounce_cnt++;
            if (debounce_cnt >= 2) // 连续20ms确认
            {
                last_key = 0;
                return 1; 
            }
        }
    }
    else 
    {
        last_key = 1;
        debounce_cnt = 0;
    }
    return 0;
}

// PID 清零
void PID_Reset(void)
{
    pid_A.ControlVelocity=0; pid_A.Last_Bias=0; pid_A.Last_Last_Bias=0;
    pid_B.ControlVelocity=0; pid_B.Last_Bias=0; pid_B.Last_Last_Bias=0;
    pid_C.ControlVelocity=0; pid_C.Last_Bias=0; pid_C.Last_Last_Bias=0;
    pid_D.ControlVelocity=0; pid_D.Last_Bias=0; pid_D.Last_Last_Bias=0;
}


// ==========================================
// 终极主循环
// ==========================================
void hal_entry(void)
{
    timer_status_t status_A, status_B, status_C, status_D;

    /* 1. 初始化底层硬件 */
    Motor_Init();
    IMU_Init();
    
    // ============================================================
    // 【ROS 上位机串口初始化】
    // ============================================================
    R_SCI_UART_Open(&g_uart_ros_ctrl, &g_uart_ros_cfg);
    //R_SCI_UART_Read(&g_uart_ros_ctrl, &ros_rx_byte, 1);

    // 启动测试输出 (直接发送，不用printf)
    UART_SendString("System Ready!\r\n");

    // ============================================================
    // 【重要】PID参数需要根据实际电机特性调试！
    // ============================================================
    float my_Kp = 700.0f;   // 比例系数 (建议范围: 10~100)
    float my_Ki = 4.0f;    // 积分系数 (建议范围: 0.5~10)
    float my_Kd = 0.0f;    // 微分系数 (一般设为0)
    int16_t my_MaxPWM = 10000; 
     
    IncPID_Init(&pid_A, my_Kp, my_Ki, my_Kd, my_MaxPWM);
    IncPID_Init(&pid_B, my_Kp, my_Ki, my_Kd, my_MaxPWM);
    IncPID_Init(&pid_C, my_Kp, my_Ki, my_Kd, my_MaxPWM);
    IncPID_Init(&pid_D, my_Kp, my_Ki, my_Kd, my_MaxPWM);

    /* 2. 开启测速与心跳定时器 */
    R_GPT_Open(&g_timer_enc_A_ctrl, &g_timer_enc_A_cfg); R_GPT_Start(&g_timer_enc_A_ctrl);
    R_GPT_Open(&g_timer_enc_B_ctrl, &g_timer_enc_B_cfg); R_GPT_Start(&g_timer_enc_B_ctrl);
    R_GPT_Open(&g_timer_enc_C_ctrl, &g_timer_enc_C_cfg); R_GPT_Start(&g_timer_enc_C_ctrl);
    R_GPT_Open(&g_timer_enc_D_ctrl, &g_timer_enc_D_cfg); R_GPT_Start(&g_timer_enc_D_ctrl);

    R_GPT_Open(&g_timer_10ms_ctrl, &g_timer_10ms_cfg);
    R_GPT_Start(&g_timer_10ms_ctrl);

    while (1)
    {
        if (flag_10ms == 1)
        {
            flag_10ms = 0; 
         
            // ------------------------------------------
            // 步骤一：【加强版】安全看门狗
            // ------------------------------------------
            ros_timeout_cnt++;
            if (ros_timeout_cnt >= 50) // 0.5秒未收到指令
            {
                // 1. 目标速度归零
                target_speed_A = 0; target_speed_B = 0; 
                target_speed_C = 0; target_speed_D = 0;
                
                // 2. 【新增】强制让运行标志位失效
                run_flag = 0; 
                
                // 3. 【新增】直接给电机断电，并清空 PID 历史记忆
                Motor_SetAllSpeed(0, 0, 0, 0);
                PID_Reset(); 
                
                ros_timeout_cnt = 50; 
            }

            // ------------------------------------------
            // 步骤二：接收上位机命令并进行"运动学逆解"
            // ------------------------------------------
            if (ros_cmd_ready == 1)
            {
                ros_cmd_ready = 0;

                float cmd_vx = 0, cmd_vy = 0, cmd_wz = 0;
                int parsed_num = sscanf(ros_rx_buf, "$%f,%f,%f", &cmd_vx, &cmd_vy, &cmd_wz);

                if (parsed_num == 3)
                {
                    ros_timeout_cnt = 0;
                    run_flag = 1;

                    float vA = cmd_vx - cmd_vy - cmd_wz * KINEMATIC_PARAM;
                    float vB = cmd_vx + cmd_vy - cmd_wz * KINEMATIC_PARAM;
                    float vC = cmd_vx + cmd_vy + cmd_wz * KINEMATIC_PARAM;
                    float vD = cmd_vx - cmd_vy + cmd_wz * KINEMATIC_PARAM;

                    target_speed_A = vA * MPS_TO_PULSE;
                    target_speed_B = vB * MPS_TO_PULSE;
                    target_speed_C = vC * MPS_TO_PULSE;
                    target_speed_D = vD * MPS_TO_PULSE;
                }
            }

            // ------------------------------------------
            // 步骤三：实体按键控制 (可覆盖上位机指令)
            // ------------------------------------------
            if(Key_Scan())
            {
                run_flag = !run_flag;
                if(run_flag == 0)
                {
                    Motor_SetAllSpeed(0,0,0,0);
                    PID_Reset();
                    target_speed_A = 0; target_speed_B = 0; 
                    target_speed_C = 0; target_speed_D = 0;
                }
                else 
                {
                    last_count_A = count_A; last_count_B = count_B;
                    last_count_C = count_C; last_count_D = count_D;
                }
            }
            
            // ------------------------------------------
            // 步骤四：读取编码器并计算真实速度
            // ------------------------------------------
            R_GPT_StatusGet(&g_timer_enc_A_ctrl, &status_A);
            R_GPT_StatusGet(&g_timer_enc_B_ctrl, &status_B);
            R_GPT_StatusGet(&g_timer_enc_C_ctrl, &status_C);
            R_GPT_StatusGet(&g_timer_enc_D_ctrl, &status_D);

            // 编码器读取（右侧C、D电机方向取反）
            count_A = (int16_t)status_A.counter;
            count_B = (int16_t)status_B.counter;
            count_C = -(int16_t)status_C.counter;
            count_D = -(int16_t)status_D.counter; 
                
            speed_A = count_A - last_count_A;
            speed_B = count_B - last_count_B;
            speed_C = count_C - last_count_C;
            speed_D = count_D - last_count_D;

            last_count_A = count_A; last_count_B = count_B;
            last_count_C = count_C; last_count_D = count_D;

            // 一阶低通滤波 (滤波系数可调: 0.5~0.9, 越大越平滑但响应越慢)
            #define FILTER_COEFF 0.7f
            filt_speed_A = FILTER_COEFF * filt_speed_A + (1.0f - FILTER_COEFF) * speed_A;
            filt_speed_B = FILTER_COEFF * filt_speed_B + (1.0f - FILTER_COEFF) * speed_B;
            filt_speed_C = FILTER_COEFF * filt_speed_C + (1.0f - FILTER_COEFF) * speed_C;
            filt_speed_D = FILTER_COEFF * filt_speed_D + (1.0f - FILTER_COEFF) * speed_D;
            
            // ------------------------------------------
            // 步骤五：运动学正解与里程计更新
            // ------------------------------------------
            Kinematics_Update(filt_speed_A, filt_speed_B, filt_speed_C, filt_speed_D, CONTROL_PERIOD);
            robot_odom.odom_yaw = robot_imu.yaw * (3.14159265f / 180.0f);
            
            // ------------------------------------------
            // 步骤六：向下控制底层执行器 (4路独立 PID)
            // ------------------------------------------
            if(run_flag)
            {
                pwm_A = IncPID_Calc(&pid_A, target_speed_A, filt_speed_A);
                pwm_B = IncPID_Calc(&pid_B, target_speed_B, filt_speed_B);
                pwm_C = IncPID_Calc(&pid_C, target_speed_C, filt_speed_C);
                pwm_D = IncPID_Calc(&pid_D, target_speed_D, filt_speed_D);

                Motor_SetAllSpeed(pwm_A, pwm_B, pwm_C, pwm_D);
            }
            else 
            {
                Motor_SetAllSpeed(0, 0, 0, 0);
            }

            // --------------------------------- ---------
            // 步骤七：向上汇报给 ROS
            // ------------------------------------------
            send_cnt++;
            if (send_cnt >= 2) // 20ms 汇报一次 (50Hz)
            {
                send_cnt = 0;
                printf("$%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
                       robot_odom.odom_x,
                       robot_odom.odom_y,
                       robot_odom.odom_yaw,
                       robot_odom.vx,
                       robot_odom.vy,
                       robot_odom.wz);
            }

        } 
    }
}
