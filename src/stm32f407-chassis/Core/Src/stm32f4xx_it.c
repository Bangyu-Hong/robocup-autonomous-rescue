/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;  // ★★★ TIM5 已禁用 ★★★
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */
extern unsigned int proc_slow;
extern unsigned char key_flag;
extern unsigned char keytime_flag;
extern uint16_t keytime_count;

extern  int encodeA_count;
extern  int encodeB_count;
extern  int encodeC_count;
extern  int encodeD_count;

extern int64_t EncodeA_now;
extern int64_t EncodeB_now;
extern int64_t EncodeC_now;
extern int64_t EncodeD_now;

extern float speedA;
extern float speedB;
extern float speedC;
extern float speedD;

// 调试用：编码器增量值
extern float varA;
extern float varB;
extern float varC;
extern float varD;

extern float targetA;
extern float targetB;
extern float targetC;
extern float targetD;

// 斜坡目标速度 (用于平滑加减速)
extern float rampTargetA;
extern float rampTargetB;
extern float rampTargetC;
extern float rampTargetD;

int16_t pwmA;
int16_t pwmB;
int16_t pwmC;
int16_t pwmD;

extern uint8_t usart_rx_buf[USART_REC_LEN];
extern uint8_t usart_rx_flag;

// ★★★ IDLE中断处理函数声明 ★★★
extern void USART1_IDLE_Handler(void);
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
    // ★★★ IDLE中断检测：串口空闲时触发数据解析 ★★★
    if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) != RESET)
    {
        // 调用IDLE处理函数（定义在main.c中）
        USART1_IDLE_Handler();
        // 注意：IDLE标志位在USART1_IDLE_Handler中清除
    }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  // 注意：USART1接收使用DMA+IDLE中断模式，不需要在这里处理接收
  // DMA循环模式自动接收，IDLE中断触发解析

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */

  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim7)
	{
		if(++proc_slow == 100) proc_slow = 0;
        if(proc_slow % 20 == 0) key_flag = 0;
        if(keytime_flag == 1) keytime_count++;
        
        
        // PID控制频率为100Hz (每10ms执行一次)
        if(proc_slow % 10 == 0)
        {
           // 注意：这里必须传入10，因为采样间隔是10ms
           speedA = get_speedA_1s(10);
           speedB = get_speedB_1s(10);
           speedC = get_speedC_1s(10);
           speedD = get_speedD_1s(10);

            // ✅ Step 1：更新速度斜坡（平滑加减速）
            update_speed_ramps();

            // ✅ Step 2：使用斜坡目标执行PID控制
            pwmA = Velocity_motorA_FeedbackControl(rampTargetA, speedA);
            pwmB = Velocity_motorB_FeedbackControl(rampTargetB, speedB);
            pwmC = Velocity_motorC_FeedbackControl(rampTargetC, speedC);
            pwmD = Velocity_motorD_FeedbackControl(rampTargetD, speedD);

            // ✅ Step 3：死区处理（斜坡目标为0且速度为0时强制PWM=0）
            if(rampTargetA == 0 && fabs(speedA) < 0.1f) pwmA = 0;
            if(rampTargetB == 0 && fabs(speedB) < 0.1f) pwmB = 0;
            if(rampTargetC == 0 && fabs(speedC) < 0.1f) pwmC = 0;
            if(rampTargetD == 0 && fabs(speedD) < 0.1f) pwmD = 0;

            // ✅ Step 4：输出到电机
            motor_setspeed(pwmA, pwmB, pwmC, pwmD);
        }

        // PID调试输出 - 每100ms输出一次（仅在调试模式下）
#if DEBUG_MODE == 1
        if(proc_slow % 100 == 0)
        {
           // 输出ABCD四轮速度信息 (单位: 转/秒)
           printf("SPD: A=%.2f B=%.2f C=%.2f D=%.2f\r\n",
                  speedA, speedB, speedC, speedD);
        }
#endif
    }

    // ★★★ TIM2 用于 C轮编码器（16位+溢出计数）★★★
    if(htim == &htim2)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
            encodeC_count--;
        else
            encodeC_count++;
	}
    // ★★★ TIM3 用于 A轮编码器（16位+溢出计数）★★★
    if(htim == &htim3)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3))
            encodeA_count--;
        else
            encodeA_count++;
	}
    // TIM4 用于 B轮编码器
    if(htim == &htim4)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4))
            encodeB_count--;
        else
            encodeB_count++;
    }
    // TIM8 用于 D轮编码器
    if(htim == &htim8)
	{
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim8))
            encodeD_count--;
        else
            encodeD_count++;
	}
    // 注意：TIM5 已禁用，C轮改回使用 TIM2（16位模式）
}


/* USER CODE END 1 */