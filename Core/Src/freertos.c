/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId LEDTaskHandle;
osThreadId KEYTaskHandle;
osThreadId IICTaskHandle;
osThreadId ETHTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartLEDTask(void const * argument);
void StartKEYTask(void const * argument);
void StartIICTask(void const * argument);
void StartETHTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of LEDTask */
  osThreadDef(LEDTask, StartLEDTask, osPriorityNormal, 0, 128);
  LEDTaskHandle = osThreadCreate(osThread(LEDTask), NULL);

  /* definition and creation of KEYTask */
  osThreadDef(KEYTask, StartKEYTask, osPriorityIdle, 0, 128);
  KEYTaskHandle = osThreadCreate(osThread(KEYTask), NULL);

  /* definition and creation of IICTask */
  osThreadDef(IICTask, StartIICTask, osPriorityIdle, 0, 128);
  IICTaskHandle = osThreadCreate(osThread(IICTask), NULL);

  /* definition and creation of ETHTask */
  osThreadDef(ETHTask, StartETHTask, osPriorityIdle, 0, 128);
  ETHTaskHandle = osThreadCreate(osThread(ETHTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartLEDTask */
/**
  * @brief  Function implementing the LEDTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void const * argument)
{
  /* USER CODE BEGIN StartLEDTask */
	uint16_t led_pin;
  /* Infinite loop */
  for(;;)
  {
    switch (led_mode) {
		case 0:
			led_pin = LED0_Pin;
			break;
		case 1:
			led_pin = LED1_Pin;
			break;
		case 2:
			led_pin = LED0_Pin | LED1_Pin;
		default:
			break;
	}
    HAL_GPIO_WritePin(LED0_GPIO_Port, led_pin, GPIO_PIN_RESET);
    osDelay(250);
    HAL_GPIO_WritePin(LED0_GPIO_Port, led_pin, GPIO_PIN_SET);
    osDelay(250);
  }
  /* USER CODE END StartLEDTask */
}

/* USER CODE BEGIN Header_StartKEYTask */
/**
* @brief Function implementing the KEYTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartKEYTask */
void StartKEYTask(void const * argument)
{
  /* USER CODE BEGIN StartKEYTask */
  /* Infinite loop */
  for(;;)
  {
    if(HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET)
    {
    	led_mode = 0;
    }
    if(HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
	{
		led_mode = 1;
	}
    if(HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
	{
		led_mode = 2;
	}
  }
  /* USER CODE END StartKEYTask */
}

/* USER CODE BEGIN Header_StartIICTask */
/**
* @brief Function implementing the IICTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIICTask */
void StartIICTask(void const * argument)
{
  /* USER CODE BEGIN StartIICTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartIICTask */
}

/* USER CODE BEGIN Header_StartETHTask */
/**
* @brief Function implementing the ETHTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartETHTask */
void StartETHTask(void const * argument)
{
  /* USER CODE BEGIN StartETHTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartETHTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
