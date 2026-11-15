#ifndef __PID_H
#define __PID_H

#include "stdint.h"
#include <math.h> // 需要用到 fabs 函数

// ==========================================
// 增量式 PID 结构体 (一个结构体就能当一个电机的独立大脑)
// ==========================================
typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float Last_Bias;       // 上一次偏差 e(k-1)
    float Last_Last_Bias;  // 上上一次偏差 e(k-2)
    
    int16_t ControlVelocity; // 当前积累的控制输出 (PWM)
    int16_t Max_Output;      // 输出限幅 (比如你的 1000)
} IncPID_TypeDef;

/* 函数声明 */
void IncPID_Init(IncPID_TypeDef *pid, float p, float i, float d, int16_t max_out);
int16_t IncPID_Calc(IncPID_TypeDef *pid, float target, float current);

#endif