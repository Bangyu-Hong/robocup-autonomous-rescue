#include "motor.h"

/* ================== 根据你的硬件引脚配置宏定义 ================== */
// 【电机 A 左前】方向引脚
#define AIN1_PORT BSP_IO_PORT_02_PIN_10
#define AIN2_PORT BSP_IO_PORT_09_PIN_06

// 【电机 B 左后】方向引脚
#define BIN1_PORT BSP_IO_PORT_03_PIN_04
#define BIN2_PORT BSP_IO_PORT_01_PIN_10

// 【电机 C 右前】方向引脚
#define CIN1_PORT BSP_IO_PORT_03_PIN_11
#define CIN2_PORT BSP_IO_PORT_09_PIN_07

// 【电机 D 右后】方向引脚
#define DIN1_PORT BSP_IO_PORT_09_PIN_00
#define DIN2_PORT BSP_IO_PORT_02_PIN_14

// ============================================================
// 【重要】电机死区补偿值，需要实测！
// 测试方法：逐渐增大PWM，记录电机刚开始转动时的PWM值
// ============================================================
#define MOTOR_DEAD_ZONE   1500   // PWM死区值 (0~10000范围内)

/* ================== 底层接口函数实现区 ================== */

/**
 * @brief  初始化电机底层硬件 (启动对应的 GPT 定时器)
 */
void Motor_Init(void)
{
    // 先把全部方向脚拉到刹车/关闭状态，避免复位后桥臂短暂导通。
    R_IOPORT_PinWrite(&g_ioport_ctrl, AIN1_PORT, BSP_IO_LEVEL_LOW);
    R_IOPORT_PinWrite(&g_ioport_ctrl, AIN2_PORT, BSP_IO_LEVEL_LOW);
    R_IOPORT_PinWrite(&g_ioport_ctrl, BIN1_PORT, BSP_IO_LEVEL_LOW);
    R_IOPORT_PinWrite(&g_ioport_ctrl, BIN2_PORT, BSP_IO_LEVEL_LOW);
    R_IOPORT_PinWrite(&g_ioport_ctrl, CIN1_PORT, BSP_IO_LEVEL_LOW);
    R_IOPORT_PinWrite(&g_ioport_ctrl, CIN2_PORT, BSP_IO_LEVEL_LOW);
    R_IOPORT_PinWrite(&g_ioport_ctrl, DIN1_PORT, BSP_IO_LEVEL_LOW);
    R_IOPORT_PinWrite(&g_ioport_ctrl, DIN2_PORT, BSP_IO_LEVEL_LOW);

    R_GPT_Open(&g_timer0_ctrl, &g_timer0_cfg);
    R_GPT_Open(&g_timer1_ctrl, &g_timer1_cfg);
    R_GPT_Open(&g_timer2_ctrl, &g_timer2_cfg);
    R_GPT_Open(&g_timer3_ctrl, &g_timer3_cfg);

    // 当前电机驱动使用反相 PWM，10000 对应停机。
    R_GPT_DutyCycleSet(&g_timer0_ctrl, MOTOR_MAX_PWM, GPT_IO_PIN_GTIOCA);
    R_GPT_DutyCycleSet(&g_timer1_ctrl, MOTOR_MAX_PWM, GPT_IO_PIN_GTIOCA);
    R_GPT_DutyCycleSet(&g_timer2_ctrl, MOTOR_MAX_PWM, GPT_IO_PIN_GTIOCA);
    R_GPT_DutyCycleSet(&g_timer3_ctrl, MOTOR_MAX_PWM, GPT_IO_PIN_GTIOCA);

    R_GPT_Start(&g_timer0_ctrl);
    R_GPT_Start(&g_timer1_ctrl);
    R_GPT_Start(&g_timer2_ctrl);
    R_GPT_Start(&g_timer3_ctrl);
}

/**
 * @brief  设置单个电机速度与方向
 * @param  motor_id : 电机编号 (MOTOR_A_LF ~ MOTOR_D_RR)
 * @param  speed    : 设定速度 (-10000 到 10000, 正数前进，负数后退，0 停止)
 */
void Motor_SetSpeed(Motor_ID_t motor_id, int16_t speed)
{
    // 1. 速度限幅与绝对值计算 (必须先算 abs_speed)
    uint32_t abs_speed = (speed > 0) ? (uint32_t)speed : (uint32_t)(-speed);
    if (abs_speed > MOTOR_MAX_PWM) abs_speed = MOTOR_MAX_PWM;

    // 2. 电机死区补偿：当PWM大于0但小于死区值时，强制提升到死区值
    // 这样可以克服电机静摩擦力，让小速度也能转动
    if (abs_speed > 0 && abs_speed < MOTOR_DEAD_ZONE)
    {
        abs_speed = MOTOR_DEAD_ZONE;
    }

    // 2. 提取方向标志 (1: 前进, -1: 后退, 0: 停止)
    int8_t dir = (speed > 0) ? 1 : ((speed < 0) ? -1 : 0);

    bsp_io_port_pin_t in1, in2;
    timer_ctrl_t *p_timer_ctrl;

    // 3. 根据电机 ID 映射底层引脚与定时器控制块
    switch (motor_id)
    {
        case MOTOR_A_LF:
            in1 = AIN1_PORT; in2 = AIN2_PORT;
            p_timer_ctrl = &g_timer1_ctrl;  // A轮 PWM: P405
            break;
        case MOTOR_B_LR:
            in1 = BIN1_PORT; in2 = BIN2_PORT;
            p_timer_ctrl = &g_timer0_ctrl;  // B轮 PWM: P512
            break;
        case MOTOR_C_RF:
            in1 = CIN1_PORT; in2 = CIN2_PORT;
            p_timer_ctrl = &g_timer2_ctrl;
            break;
        case MOTOR_D_RR:
            in1 = DIN1_PORT; in2 = DIN2_PORT;
            p_timer_ctrl = &g_timer3_ctrl;
            break;
        default:
            return;
    }

    // 4. 执行 IO 翻转控制方向
    if (dir == 1) {        // 前进
        R_IOPORT_PinWrite(&g_ioport_ctrl, in1, BSP_IO_LEVEL_HIGH);
        R_IOPORT_PinWrite(&g_ioport_ctrl, in2, BSP_IO_LEVEL_LOW);
    } else if (dir == -1) { // 后退
        R_IOPORT_PinWrite(&g_ioport_ctrl, in1, BSP_IO_LEVEL_LOW);
        R_IOPORT_PinWrite(&g_ioport_ctrl, in2, BSP_IO_LEVEL_HIGH);
    } else {               // 停止
        R_IOPORT_PinWrite(&g_ioport_ctrl, in1, BSP_IO_LEVEL_LOW);
        R_IOPORT_PinWrite(&g_ioport_ctrl, in2, BSP_IO_LEVEL_LOW);
    }

    // 5. 软件极性反转并更新 PWM 占空比 (放在最后算绝对安全！)
    uint32_t real_pwm = MOTOR_MAX_PWM - abs_speed; 
    R_GPT_DutyCycleSet(p_timer_ctrl, real_pwm, GPT_IO_PIN_GTIOCA);
}

/**
 * @brief  同时设置四个电机速度
 */
void Motor_SetAllSpeed(int16_t speed_A, int16_t speed_B, int16_t speed_C, int16_t speed_D)
{
    Motor_SetSpeed(MOTOR_A_LF, speed_A);
    Motor_SetSpeed(MOTOR_B_LR, speed_B);
    Motor_SetSpeed(MOTOR_C_RF, speed_C);
    Motor_SetSpeed(MOTOR_D_RR, speed_D);
}
