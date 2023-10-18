/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "crypto.h"
#include "stdio.h"
#include <string>
#include "rs.hpp"
#include "implifi_obj.hpp"

using namespace std;

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//282, 564, 1128
#define UART_RX_BUF_SIZE 2*(128 + 12) + 2 // 128: # of data bytes, 12: # of Reed-Solomon bytes, *2: manchester halves data rate, +2: two leading 0x00 bytes
#define USB_TX_BUF_SIZE 256               // Arbitrarily chosen Just needs to be bigger than 132. (And USB is configured to have max buf size of 512)
#define USB_PRINT_DIAGNOSTICS false       // prints 5 bytes of diagnostics if true, prints full message if false

#define UART_BAUD_RATE 57600
#define UART_STOP_BITS UART_STOPBITS_1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

//UART DMA variables
uint8_t uartRxBuf[UART_RX_BUF_SIZE];
char usbTxBuf[USB_TX_BUF_SIZE];
char usbTxBuf_tmp[USB_TX_BUF_SIZE];

ImpLiFiClass lifi_obj;   // Holds all the values/functions for Manchester, AES, & Reed-Solomon

bool start_decryption_flag = false; // Gets set when a frame of data is received, is cleared when it is done decoding

char usb_print_msg_str[128];
char usb_print_diagnostics_str[5];

const char new_line_arr[4] = {0x0A, 0x0A, 0x0D, 0x00}; // Holds value of \n\n\r\0

// Stuff for AGC gain
uint32_t adc_reading;
int agc_lvl;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void PrintLiFiMessageUSB(void);

void SetAGCLevel(int lvl);
void UpdateAGC(uint32_t adc_val_in);
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
  __CRC_CLK_ENABLE();   // Needed for AES
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_CRC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uartRxBuf, UART_RX_BUF_SIZE); // Begin UART DMA
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  HAL_TIM_Base_Start_IT(&htim2);

  // These two chars are constant
  usb_print_diagnostics_str[2] = 0x2F;
  usb_print_diagnostics_str[4] = '\0';
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(start_decryption_flag) // goes high when 282 bytes or idle line detection occurs
	  {
		  if(lifi_obj.DecryptData(lifi_obj.manchester_encoded_buff_cpy)) // If it successfully decodes
		  {
			  PrintLiFiMessageUSB();
		  }
		  start_decryption_flag = false;
	  }

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = UART_BAUD_RATE;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOP_BITS;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
	 GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/** void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
  *
  * @brief  Handles incoming UART Data after it is stuffed in uartRxBuf
  *
  * @note   Called once UART frame is done being received. Note this is overriding
  *         a weak-defined function in the HAL Library.
  *
  * @param  *huart: Pointer to UART_HandleTypeDef of UART that triggered interrupt
  *
  * @param  Size: Number of bytes received until buffer is full or idle line is detected
  *
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(Size == UART_RX_BUF_SIZE) // If 282 bytes were received
	{
		std::copy(uartRxBuf + 2, uartRxBuf + UART_RX_BUF_SIZE, lifi_obj.manchester_encoded_buff_cpy);
		start_decryption_flag = true;
	}

	// restart/re-enable UART DMA and interrupt
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uartRxBuf, UART_RX_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

/*
 *  TO-DO: This is really ugly but works and I don't have
 *  time to properly handle. *¯\_(ツ)_/¯* When line-of-sight
 *  is interrupted, HAL UART DMA throws error (0x02 and 0x04,
 *  which are framing error and noise error). I just reset
 *  the whole UART DMA and it'll read in the next frame.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	start_decryption_flag = false;
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uartRxBuf, UART_RX_BUF_SIZE);
	__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

/** void PrintLiFiMessageUSB(void)
  *
  * @brief  Prints LiFi data to computer via USB Virtual COM Port
  *
  * @note   If USB_PRINT_DIAGNOSTICS, will print data formatted as Aaron specified.
  *         If not, will convert the Message Index ID (first 2 bytes of received
  *         data) to a decimal string, the 'Lifi Test ABC...' data, two newlines,
  *         a carrier return, and null-terminating chars.
  *
  * @retval None
  */
void PrintLiFiMessageUSB(void)
{
	if(USB_PRINT_DIAGNOSTICS)
	{
		usb_print_diagnostics_str[0] = lifi_obj.output_message[0];
		usb_print_diagnostics_str[1] = lifi_obj.output_message[1];
		usb_print_diagnostics_str[3] = lifi_obj.num_errs_val;

		sprintf(usbTxBuf, "%s", usb_print_diagnostics_str);
		CDC_Transmit_FS((uint8_t *) usbTxBuf, 5);
	}
	else
	{
		sprintf(usbTxBuf_tmp, "%u", (uint16_t)(lifi_obj.output_message[0] << 8) + lifi_obj.output_message[1]); // convert uint16_t to base 10 string
		uint16_t str_len_tmp = (uint16_t)strlen(usbTxBuf_tmp);

		//usbTxBuf_tmp[str_len_tmp] = '/';
		//str_len_tmp = (uint16_t)strlen(usbTxBuf_tmp);

		//usbTxBuf_tmp[str_len_tmp] = (char)lifi_obj.num_errs_val;
		//str_len_tmp = (uint16_t)strlen(usbTxBuf_tmp);

		//usbTxBuf_tmp[str_len_tmp] = ':';
		//str_len_tmp = (uint16_t)strlen(usbTxBuf_tmp);

		std::copy(lifi_obj.output_message + 2, lifi_obj.output_message + 128, usbTxBuf_tmp + str_len_tmp);

		std::copy(new_line_arr, new_line_arr + sizeof(new_line_arr)-1, usbTxBuf_tmp + str_len_tmp + 126); // Insert 2 newlines, carrier return, and null terminating

		sprintf(usbTxBuf, "%s", usbTxBuf_tmp);
		CDC_Transmit_FS((uint8_t *) usbTxBuf, strlen(usbTxBuf) + 1);
	}
}

/** void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  *
  * @brief  Controls MUX channel select for AGC
  *
  * @param  int lvl: MUX gain level to be selected (0 through 5)
  *
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1);
		adc_reading = HAL_ADC_GetValue(&hadc1);

		UpdateAGC(adc_reading);
	}
}

/** void SetAGCLevel(int lvl)
  *
  * @brief  Controls MUX channel select for AGC
  *
  * @param  int lvl: MUX gain level to be selected (0 through 5)
  *
  * @retval None
  */
void SetAGCLevel(int lvl)
{
	if(lvl == 1) //Y0
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);
	}
	else if(lvl == 2) //Y1
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
	}
	else if(lvl == 3) //Y2
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
	}
	else if(lvl == 4) //Y4
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	}
	else if(lvl == 5) //Y6
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_SET);
	}
	else //Y3 (Lowest gain level)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	}
}

/** void UpdateAGC(uint32_t adc_val_in)
  *
  * @brief  Uses most recent ADC reading to determine thresholding
  * for increase/decrease gain of transimpedance op-amp.
  *
  * @param  uint32_t adc_val_in: Sampled ADC value
  *
  * @retval None
  */
void UpdateAGC(uint32_t adc_val_in)
{
	//ORIGINAL NUMBERS THAT WORK: 900 -> 1800

	//if((adc_val_in > 1800) && (agc_lvl > 0)) //DC value is too high -> decrease gain
	if((adc_val_in > 2200) && (agc_lvl > 0)) //DC value is too high -> decrease gain
	{
		agc_lvl--;
		SetAGCLevel(agc_lvl);
	}
	//else if((adc_val_in < 1050) && (agc_lvl < 4)) //DC value is too low -> increase gain
	//else if((adc_val_in < 1250) && (agc_lvl < 4)) //DC value is too low -> increase gain
	else if((adc_val_in < 1250) && (agc_lvl < 3)) //DC value is too low -> increase gain
	{
		agc_lvl++;
		SetAGCLevel(agc_lvl);
	}
}

/* USER CODE END 4 */

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
