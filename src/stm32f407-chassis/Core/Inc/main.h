/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

// ======================== 全局调试配置 ========================
// 调试模式 (1=输出调试信息, 0=只输出标准里程计数据)
#define DEBUG_MODE       0

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
// ★★★ 电机D引脚定义 ★★★
#define motorD_IN1_Pin GPIO_PIN_2
#define motorD_IN1_GPIO_Port GPIOE
#define motorD_IN2_Pin GPIO_PIN_3
#define motorD_IN2_GPIO_Port GPIOE
// ★★★ 电机C IN1引脚定义 ★★★
#define motorC_IN1_Pin GPIO_PIN_4
#define motorC_IN1_GPIO_Port GPIOE
// ★★★ KEY 按键 PA0，高电平触发 ★★★
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOA
#define motorC_EncoderCH2_Pin GPIO_PIN_1
#define motorC_EncoderCH2_GPIO_Port GPIOA
#define motorC_EncoderCH1_Pin GPIO_PIN_5
#define motorC_EncoderCH1_GPIO_Port GPIOA
#define motorA_EncoderCH1_Pin GPIO_PIN_6
#define motorA_EncoderCH1_GPIO_Port GPIOA
#define motorA_EncoderCH2_Pin GPIO_PIN_7
#define motorA_EncoderCH2_GPIO_Port GPIOA
#define motorA_IN1_Pin GPIO_PIN_7
#define motorA_IN1_GPIO_Port GPIOE
#define motorA_IN2_Pin GPIO_PIN_8
#define motorA_IN2_GPIO_Port GPIOE
#define motorA_PWM_Pin GPIO_PIN_9
#define motorA_PWM_GPIO_Port GPIOE
#define motorB_IN1_Pin GPIO_PIN_10
#define motorB_IN1_GPIO_Port GPIOE
#define motorB_PWM_Pin GPIO_PIN_11
#define motorB_PWM_GPIO_Port GPIOE
#define motorB_IN2_Pin GPIO_PIN_12
#define motorB_IN2_GPIO_Port GPIOE
#define motorC_PWM_Pin GPIO_PIN_13
#define motorC_PWM_GPIO_Port GPIOE
#define motorD_PWM_Pin GPIO_PIN_14
#define motorD_PWM_GPIO_Port GPIOE
#define motorC_IN2_Pin GPIO_PIN_15
#define motorC_IN2_GPIO_Port GPIOE
#define motorB_EncoderCH2_Pin GPIO_PIN_12
#define motorB_EncoderCH2_GPIO_Port GPIOD
#define motorB_EncoderCH1_Pin GPIO_PIN_13
#define motorB_EncoderCH1_GPIO_Port GPIOD
#define motorD_EncoderCH2_Pin GPIO_PIN_6
#define motorD_EncoderCH2_GPIO_Port GPIOC
#define motorD_EncoderCH1_Pin GPIO_PIN_7
#define motorD_EncoderCH1_GPIO_Port GPIOC
#define Usart1_TX_Pin GPIO_PIN_9
#define Usart1_TX_GPIO_Port GPIOA
#define Usart1_RX_Pin GPIO_PIN_10
#define Usart1_RX_GPIO_Port GPIOA
#define Usart2_TX_Pin GPIO_PIN_2
#define Usart2_TX_GPIO_Port GPIOA
#define Usart2_RX_Pin GPIO_PIN_3
#define Usart2_RX_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
// ======================== 电机和编码器方向配置 ========================
// 电机方向修正宏 (0=正常, 1=反向)
#define MOTOR_A_REVERSE 0
#define MOTOR_B_REVERSE 0
#define MOTOR_C_REVERSE 0
#define MOTOR_D_REVERSE 0
                                                                                                                                                                                                                                                        

// 编码器方向修正 (0=正常, 1=反向)
#define ENCODER_A_REVERSE 0
#define ENCODER_B_REVERSE 1
#define ENCODER_C_REVERSE 1
#define ENCODER_D_REVERSE 0

// PID参数
#define Velcity_Kp     925
#define Velcity_Ki     82
#define Velcity_Kd     27

// 速度斜坡参数 (用于平滑加减速，保护电机)
// SPEED_RAMP_RATE: 每个PID周期(10ms)目标速度最大变化量(转/秒)
// 例如: 0.5 表示从0加速到1转/秒需要20ms (2个周期)
// 设置为0则禁用斜坡，使用即时目标速度
// ★★★ 优化：增大斜坡速率，提高指令响应速度 ★★★
#define SPEED_RAMP_RATE     0.5f   // 每10ms变化0.5转/秒，约20ms完成1转/秒变化
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
