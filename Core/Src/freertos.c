/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ProcessCAN_MSG_ */
osThreadId_t ProcessCAN_MSG_Handle;
const osThreadAttr_t ProcessCAN_MSG__attributes = {
  .name = "ProcessCAN_MSG_",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for SendCAN_MSG_ */
osThreadId_t SendCAN_MSG_Handle;
const osThreadAttr_t SendCAN_MSG__attributes = {
  .name = "SendCAN_MSG_",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for processDatatask */
osThreadId_t processDatataskHandle;
const osThreadAttr_t processDatatask_attributes = {
  .name = "processDatatask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for queue_can_receive */
osMessageQueueId_t queue_can_receiveHandle;
const osMessageQueueAttr_t queue_can_receive_attributes = {
  .name = "queue_can_receive"
};
/* Definitions for queue_can_send */
osMessageQueueId_t queue_can_sendHandle;
const osMessageQueueAttr_t queue_can_send_attributes = {
  .name = "queue_can_send"
};
/* Definitions for queue_process_data */
osMessageQueueId_t queue_process_dataHandle;
const osMessageQueueAttr_t queue_process_data_attributes = {
  .name = "queue_process_data"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ProcessCAN_MSG(void *argument);
void SendCAN_MSG(void *argument);
void Process_data_task(void *argument);

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

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of queue_can_receive */
  queue_can_receiveHandle = osMessageQueueNew (4, sizeof(CanPacket), &queue_can_receive_attributes);

  /* creation of queue_can_send */
  queue_can_sendHandle = osMessageQueueNew (4, sizeof(CanPacket), &queue_can_send_attributes);

  /* creation of queue_process_data */
  queue_process_dataHandle = osMessageQueueNew (4, sizeof(SensorData), &queue_process_data_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ProcessCAN_MSG_ */
  ProcessCAN_MSG_Handle = osThreadNew(ProcessCAN_MSG, NULL, &ProcessCAN_MSG__attributes);

  /* creation of SendCAN_MSG_ */
  SendCAN_MSG_Handle = osThreadNew(SendCAN_MSG, NULL, &SendCAN_MSG__attributes);

  /* creation of processDatatask */
  processDatataskHandle = osThreadNew(Process_data_task, NULL, &processDatatask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ProcessCAN_MSG */
/**
* @brief Function implementing the ProcessCAN_MSG_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ProcessCAN_MSG */
__weak void ProcessCAN_MSG(void *argument)
{
  /* USER CODE BEGIN ProcessCAN_MSG */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ProcessCAN_MSG */
}

/* USER CODE BEGIN Header_SendCAN_MSG */
/**
* @brief Function implementing the SendCAN_MSG_ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SendCAN_MSG */
__weak void SendCAN_MSG(void *argument)
{
  /* USER CODE BEGIN SendCAN_MSG */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SendCAN_MSG */
}

/* USER CODE BEGIN Header_Process_data_task */
/**
* @brief Function implementing the processDatatask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Process_data_task */
__weak void Process_data_task(void *argument)
{
  /* USER CODE BEGIN Process_data_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Process_data_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

