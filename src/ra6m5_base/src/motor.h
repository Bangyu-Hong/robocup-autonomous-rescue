#ifndef __MOTOR_H
#define __MOTOR_H

#include "hal_data.h"

/* 定义 PWM 的最大计数值 (对应你 FSP 中设置的 Period 10000) */
#define MOTOR_MAX_PWM 10000 

/* 使用枚举来定义电机 */
typedef enum {
    MOTOR_A_LF = 0,  // 左前 (PWM: P415)
    MOTOR_B_LR,      // 左后 (PWM: P405)
    MOTOR_C_RF,      // 右前 (PWM: P113)
    MOTOR_D_RR       // 右后 (PWM: P111)
} Motor_ID_t;

/* 外部可调用的接口函数 */
void Motor_Init(void);
void Motor_SetSpeed(Motor_ID_t motor_id, int16_t speed);
void Motor_SetAllSpeed(int16_t speed_A, int16_t speed_B, int16_t speed_C, int16_t speed_D);

#endif /* __MOTOR_H */