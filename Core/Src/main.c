/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BOARD_ID 1
#define SAMPLES_PER_TIME 64
#define OFFSET 1
#define CORRECTION_FACTOR 1
#define CAN_DEVICE_ID DEVICE_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t adcint[4] = {0};
extern uint32_t TxMailbox;
uint8_t canRX[8] = {};
uint8_t canTX[8] = {};
extern module_cfg configs;
bool aux = 0;
uint16_t adc_count = 0;

extern uint16_t bufferTensao[64];
extern uint16_t bufferTensaoVA[64];
extern uint16_t bufferTensaoVB[64];
extern uint16_t bufferTensaoVC[64];
extern osMessageQueueId_t queue_can_sendHandle;
extern osMessageQueueId_t queue_can_receiveHandle;
extern osMessageQueueId_t queue_process_dataHandle;
SignalQ sgnalQual[3] = {0};
SensorData sensorData;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_CAN_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);

	HAL_ADCEx_Calibration_Start(&hadc);
	HAL_ADC_Start_DMA(&hadc, &adcint, 3);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void ReceiveCAN_MSG(void *argument)
{
  /* USER CODE BEGIN ProcessCAN_MSG */
	CanPacket canMSG = {0};
  /* Infinite loop */
  for(;;)
  {
	BaseType_t xStatus = xQueueReceive(queue_can_receiveHandle, &canMSG, portMAX_DELAY);
	if (xStatus == pdPASS)
	{
		// conseguiu tirar da fila

	}
    osDelay(1);
  }
  /* USER CODE END ProcessCAN_MSG */
}

void SendCAN_MSG(void *argument)
{
  /* USER CODE BEGIN SendCAN_MSG */
	CanPacket canMsg = {0};
  /* Infinite loop */
  for(;;)
  {
	BaseType_t xStatus = xQueueReceive(queue_can_sendHandle, &canMsg, portMAX_DELAY);
	if (xStatus == pdPASS)
	{
		// conseguiu tirar da fila

		TxHeader.StdId             = canMsg.packet.canID;      // ID do dispositivo
		TxHeader.RTR               = CAN_RTR_DATA;       		//(Remote Transmission Request) especifica Remote Fraame ou Data Frame.
		TxHeader.IDE               = CAN_ID_STD;    			//define o tipo de id (standard ou extended
		TxHeader.DLC               = CAN_SIZE;      			//Tamanho do pacote 0 - 8 bytes
		TxHeader.TransmitGlobalTime = DISABLE;
		int status = HAL_CAN_AddTxMessage (&hcan, &TxHeader, canMsg.packet.canBuffer.canData, &TxMailbox);
		if(status)
		{
			Error_Handler();
		}

	}
    osDelay(1);
  }
  /* USER CODE END SendCAN_MSG */
}


void Process_data_task(void *argument)
{
  /* USER CODE BEGIN Process_data_task */
	uint16_t count = 0;
	SignalQ signalQ[3] = {0};
	SensorData sensorData = {0};

	Data data;
  /* Infinite loop */
  for(;;)
  {
	BaseType_t xStatus = xQueueReceive(queue_process_dataHandle, &sensorData, portMAX_DELAY);
	if (xStatus == pdPASS)
	{
			// conseguiu tirar da fila
		calculate_analog(&data, &sensorData);
//		calculate_RMS(signalQ,&data);
//		calculate_Phase(signalQ,&data);
//		calculate_freq(signalQ,&data);
		count++;
		if (count == SAMPLES_PER_TIME)
		{
			MeanValues meanValues[3] = {0};
			calculate_mean(meanValues, signalQ);
			send_data_to_queue(meanValues);

			memset(signalQ, 0, sizeof(signalQ));
			count = 0;
		}
		memset(&sensorData, 0, sizeof(sensorData));
	}
    osDelay(1);
  }
  /* USER CODE END Process_data_task */
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

	UartPacket uartPacket = {0};

	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, canRX);

	DecodeCanPacket(RxHeader.StdId, &uartPacket, canRX);

//	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);


}

void sendCanMsg_test(int delay)
{
	  uint8_t tx[7] = {1,2,3,4,5,6,7};
	  TxHeader.StdId             = 0x0;     // ID do dispositivo
	  TxHeader.RTR               = CAN_RTR_DATA;       //(Remote Transmission Request) especifica Remote Fraame ou Data Frame.
	  TxHeader.IDE               = CAN_ID_STD;    //define o tipo de id (standard ou extended
	  TxHeader.DLC               = 7;      //Tamanho do pacote 0 - 8 bytes
	  TxHeader.TransmitGlobalTime = DISABLE;

	  int status = HAL_CAN_AddTxMessage(&hcan, &TxHeader, tx, &TxMailbox);
	  if(status)
	  {
		 Error_Handler();
	  }
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  HAL_Delay(delay);
}


void calculate_analog(Data *data, SensorData *sensorData)
{
	for (int i = 0; i < SAMPLES_PER_TIME; i++)
	{
		data->sensorData_values[VA][i] = (( sensorData->sensorData_buff[VA][i] * MAX_ADC ) * MAX_ANALOG - OFFSET) * CORRECTION_FACTOR;
		data->sensorData_values[VB][i] = (( sensorData->sensorData_buff[VB][i] * MAX_ADC ) * MAX_ANALOG - OFFSET) * CORRECTION_FACTOR;
		data->sensorData_values[VC][i] = (( sensorData->sensorData_buff[VC][i] * MAX_ADC ) * MAX_ANALOG - OFFSET) * CORRECTION_FACTOR;
	}
}

void calculate_RMS(float RMS[], Data *data)
{
	float sum_square[3] = {0};

	for (int i = 0; i < SAMPLES_PER_TIME; i++)
	{
		sum_square[VA] += data->sensorData_values[VA][i];
		sum_square[VB] += data->sensorData_values[VB][i];
	    sum_square[VC] += data->sensorData_values[VC][i];
	}
	RMS[VA] = sqrt(sum_square[VA] / SAMPLES_PER_TIME);
	RMS[VB] = sqrt(sum_square[VB] / SAMPLES_PER_TIME);
	RMS[VC] = sqrt(sum_square[VC] / SAMPLES_PER_TIME);
}

void calculate_Phase(SensorData *data)
{

}

void fill_data(CanPacket *message, MeanValues values)
{
	message->packet.canBuffer.canDataFields.data[0] = (uint8_t)( ( (uint16_t)values.meanRMS >> 8) & 0xFF);
	message->packet.canBuffer.canDataFields.data[1] = (uint8_t)(   (uint16_t)values.meanRMS & 0xFF );

	message->packet.canBuffer.canDataFields.data[2] = (uint8_t)( ( (uint16_t)values.meanPhase >> 8) & 0xFF);
	message->packet.canBuffer.canDataFields.data[3] = (uint8_t)(   (uint16_t)values.meanPhase & 0xFF );

	message->packet.canBuffer.canDataFields.data[4] = (uint8_t)( ( (uint16_t)values.meanFreq >> 8) & 0xFF);
	message->packet.canBuffer.canDataFields.data[5] = (uint8_t)(   (uint16_t)values.meanFreq & 0xFF );

	// teste correto da conversao para 2 uint8_t e ao contrario
//    uint16_t testeint;
//    uint8_t teste8;
//    uint8_t teste82;
//    float f = 1145.56;
//    teste8 = (uint8_t)(((uint16_t)f >> 8) & 0xFF);
//    teste82 = (uint8_t)((uint16_t)f  & 0xFF);
//    printf("%d", ((uint16_t)teste8 << 8) |  teste82 );
}

void send_data_to_queue(MeanValues values[])
{
	CanPacket message = {0};

	for (int i = 0; i < 3; i++)
	{
		message.packet.canID = CAN_DEVICE_ID;
		message.packet.canBuffer.canDataFields.ctrl0.value = 0; //revisar
		message.packet.canBuffer.canDataFields.ctrl1.value = i; //numero do sensor

		fill_data(&message, values[i]); // a cada tensao (a b c) envia uma mensagem can com rms, fase e freq

		BaseType_t xStatus = xQueueSendToBackFromISR(queue_can_sendHandle, &message, 0);
		if (xStatus != pdPASS)
		{
	//			fila estourou
		}
		HAL_Delay(1);
	}




}

void calculate_mean(MeanValues meanValues[3] , SignalQ signalQ[3])
{
	float sumValues[3][3] = {0};
	for (int i = 0; i < 60; i++)
	{
		sumValues[VA][0]  += signalQ[VA].rms[i];
		sumValues[VA][1]  += signalQ[VB].phase[i];
		sumValues[VA][2]  += signalQ[VC].freq[i];

		sumValues[VB][0]   += signalQ[VA].rms[i];
		sumValues[VB][1]  += signalQ[VB].phase[i];
		sumValues[VB][2]  += signalQ[VC].freq[i];

		sumValues[VC][0]  += signalQ[VA].rms[i];
		sumValues[VC][1]  += signalQ[VB].phase[i];
		sumValues[VC][2]  += signalQ[VC].freq[i];
	}
	meanValues[VA].meanRMS = sumValues[VA][0] / 64;
	meanValues[VA].meanRMS = sumValues[VA][1] / 64;
	meanValues[VA].meanRMS = sumValues[VA][2] / 64;

	meanValues[VB].meanRMS = sumValues[VB][0] / 64;
	meanValues[VB].meanRMS = sumValues[VB][1] / 64;
	meanValues[VB].meanRMS = sumValues[VB][2] / 64;

	meanValues[VC].meanRMS = sumValues[VC][0] / 64;
	meanValues[VC].meanRMS = sumValues[VC][1] / 64;
	meanValues[VC].meanRMS = sumValues[VC][2] / 64;
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	if(htim->Instance==TIM2)
	{
		if (adc_count < SAMPLES_PER_TIME)
		{
			sensorData.sensorData_buff[VA][adc_count] = bufferTensaoVA[adc_count];
			sensorData.sensorData_buff[VB][adc_count] = bufferTensaoVB[adc_count];
			sensorData.sensorData_buff[VC][adc_count] = bufferTensaoVC[adc_count];
			adc_count++;
			if (adc_count == 64)
			{
				BaseType_t xStatus = xQueueSendToBackFromISR(queue_process_dataHandle, &sensorData.sensorData_buff, 0);
				if (xStatus != pdPASS)
				{
//				 	fila estourou
				}
				aux = 1;
				adc_count = 0;
			}
		}
	}
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
//	  NVIC_SystemReset();
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
