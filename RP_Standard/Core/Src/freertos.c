/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
osThreadId defaultTaskHandle;
osThreadId LED_taskHandle;
osThreadId Monitor_taskHandle;
osThreadId System_taskHandle;
osThreadId Ctrl_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Start_LED_task(void const * argument);
void Start_Monitor_task(void const * argument);
void Start_System_task(void const * argument);
void Start_Ctrl_task(void const * argument);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LED_task */
  osThreadDef(LED_task, Start_LED_task, osPriorityRealtime, 0, 128);
  LED_taskHandle = osThreadCreate(osThread(LED_task), NULL);

  /* definition and creation of Monitor_task */
  osThreadDef(Monitor_task, Start_Monitor_task, osPriorityAboveNormal, 0, 128);
  Monitor_taskHandle = osThreadCreate(osThread(Monitor_task), NULL);

  /* definition and creation of System_task */
  osThreadDef(System_task, Start_System_task, osPriorityHigh, 0, 128);
  System_taskHandle = osThreadCreate(osThread(System_task), NULL);

  /* definition and creation of Ctrl_task */
  osThreadDef(Ctrl_task, Start_Ctrl_task, osPriorityHigh, 0, 512);
  Ctrl_taskHandle = osThreadCreate(osThread(Ctrl_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start_LED_task */
/**
* @brief Function implementing the LED_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_LED_task */
__weak void Start_LED_task(void const * argument)
{
  /* USER CODE BEGIN Start_LED_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_LED_task */
}

/* USER CODE BEGIN Header_Start_Monitor_task */
/**
* @brief Function implementing the Monitor_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Monitor_task */
__weak void Start_Monitor_task(void const * argument)
{
  /* USER CODE BEGIN Start_Monitor_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Monitor_task */
}

/* USER CODE BEGIN Header_Start_System_task */
/**
* @brief Function implementing the System_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_System_task */
__weak void Start_System_task(void const * argument)
{
  /* USER CODE BEGIN Start_System_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_System_task */
}

/* USER CODE BEGIN Header_Start_Ctrl_task */
/**
* @brief Function implementing the Ctrl_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Ctrl_task */
__weak void Start_Ctrl_task(void const * argument)
{
  /* USER CODE BEGIN Start_Ctrl_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Start_Ctrl_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
