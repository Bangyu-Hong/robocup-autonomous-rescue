#include "pid.h"

/**
 * @brief  初始化增量式 PID 参数
 */
void IncPID_Init (IncPID_TypeDef *pid, float p, float i, float d, int16_t max_out)
{
    pid->Kp = p;
    pid->Ki = i;
    pid->Kd = d;
    
    pid->Last_Bias = 0.0f;
    pid->Last_Last_Bias = 0.0f;
    pid->ControlVelocity = 0;
    
    pid->Max_Output = max_out; 
}

/**
 * @brief  增量式 PID 核心计算 (百分百复刻你的数学逻辑)
 * @param  pid     : 对应电机的 PID 结构体指针
 * @param  target  : 目标速度
 * @param  current : 编码器真实速度
 * @return 算出的 PWM 占空比数值
 */
int16_t IncPID_Calc(IncPID_TypeDef *pid, float target, float current)
{
    float Bias = target - current;

    // 目标为0且速度已经很低时，重置PID状态并停止
    // 使用 fabs 比较避免浮点精度问题
    if (fabs(target) < 0.01f && fabs(current) < 0.5f) {
        pid->ControlVelocity = 0;
        pid->Last_Bias = 0.0f;
        pid->Last_Last_Bias = 0.0f;
        return 0;
    }

    // 增量式PID核心公式
   float delta;

		delta = pid->Kp * (Bias - pid->Last_Bias)
      + pid->Ki * Bias
      + pid->Kd * (Bias - 2.0f * pid->Last_Bias + pid->Last_Last_Bias);

		pid->ControlVelocity += (int16_t)delta;

    // 更新历史偏差
    pid->Last_Last_Bias = pid->Last_Bias;
    pid->Last_Bias = Bias;

    // 输出限幅
    if (pid->ControlVelocity >= pid->Max_Output) pid->ControlVelocity = pid->Max_Output;
    if (pid->ControlVelocity <= -pid->Max_Output) pid->ControlVelocity = -pid->Max_Output;

    return pid->ControlVelocity;
}