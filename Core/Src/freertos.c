/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os2.h"
#include "usart.h"

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
uint32_t indx;
/* USER CODE END Variables */
/* Definitions for task1 */
osThreadId_t task1Handle;
const osThreadAttr_t task1_attributes = {
  .name = "task1",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for normalTask */
osThreadId_t normalTaskHandle;
const osThreadAttr_t normalTask_attributes = {
  .name = "normalTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for highTask */
osThreadId_t highTaskHandle;
const osThreadAttr_t highTask_attributes = {
  .name = "highTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for lowTask */
osThreadId_t lowTaskHandle;
const osThreadAttr_t lowTask_attributes = {
  .name = "lowTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for BinSem */
osSemaphoreId_t BinSemHandle;
const osSemaphoreAttr_t BinSem_attributes = {
  .name = "BinSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask1(void *argument);
void StartNormalTask(void *argument);
void StartHighTask(void *argument);
void StartLowTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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

  /* Create the semaphores(s) */
  /* creation of BinSem */
  BinSemHandle = osSemaphoreNew(1, 1, &BinSem_attributes);

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
  /* creation of task1 */
  task1Handle = osThreadNew(StartTask1, NULL, &task1_attributes);

  /* creation of normalTask */
  normalTaskHandle = osThreadNew(StartNormalTask, NULL, &normalTask_attributes);

  /* creation of highTask */
  highTaskHandle = osThreadNew(StartHighTask, NULL, &highTask_attributes);

  /* creation of lowTask */
  lowTaskHandle = osThreadNew(StartLowTask, NULL, &lowTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask1 */
/**
  * @brief  Function implementing the task1 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask1 */
void StartTask1(void *argument)
{
  /* USER CODE BEGIN StartTask1 */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
    //osDelay(50);
  }
  /* USER CODE END StartTask1 */
}

/* USER CODE BEGIN Header_StartNormalTask */
/**
* @brief Function implementing the normalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartNormalTask */
void StartNormalTask(void *argument)
{
  /* USER CODE BEGIN StartNormalTask */
  /* Infinite loop */
  for(;;)
  {
	char *str1 = "Entered MediumTask\n";
	HAL_UART_Transmit(&huart3, (uint8_t *) str1, strlen (str1), 100);


	char *str2 = "Leaving MediumTask\n\n";
	HAL_UART_Transmit(&huart3, (uint8_t *) str2, strlen (str2), 100);
	osDelay(500);
  }
  /* USER CODE END StartNormalTask */
}

/* USER CODE BEGIN Header_StartHighTask */
/**
* @brief Function implementing the highTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHighTask */
void StartHighTask(void *argument)
{
  /* USER CODE BEGIN StartHighTask */
  /* Infinite loop */
  for(;;)
  {
	char *str1 = "Entered HighTask and waiting for Semaphore\n";
	HAL_UART_Transmit(&huart3, (uint8_t *) str1, strlen (str1), 100);

	osSemaphoreAcquire(BinSemHandle, osWaitForever);

	char *str3 = "Semaphore acquired by HIGH Task\n";
	HAL_UART_Transmit(&huart3, (uint8_t *) str3, strlen (str3), 100);

	char *str2 = "Leaving HighTask and releasing Semaphore\n\n";
	HAL_UART_Transmit(&huart3, (uint8_t *) str2, strlen (str2), 100);

	osSemaphoreRelease(BinSemHandle);
	osDelay(500);
  }
  /* USER CODE END StartHighTask */
}

/* USER CODE BEGIN Header_StartLowTask */
/**
* @brief Function implementing the lowTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLowTask */
void StartLowTask(void *argument)
{
  /* USER CODE BEGIN StartLowTask */
  /* Infinite loop */
  for(;;)
  {
	char *str1 = "Entered LOWTask and waiting for semaphore\n";
	HAL_UART_Transmit(&huart3, (uint8_t *) str1, strlen (str1), 100);

	osSemaphoreAcquire(BinSemHandle, osWaitForever);
	char *str3 = "Semaphore acquired by LOW Task\n";
	HAL_UART_Transmit(&huart3, (uint8_t *) str3, strlen (str3), 100);

	while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13));  // wait till the pin go low

	char *str2 = "Leaving LOWTask and releasing Semaphore\n\n";
	HAL_UART_Transmit(&huart3, (uint8_t *) str2, strlen (str2), 100);

	osSemaphoreRelease(BinSemHandle);
	osDelay(500);
  }
  /* USER CODE END StartLowTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

