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
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
uint32_t timeoutCnt = 0;
uint16_t cycleCnt = 0;
uint32_t sigArr[200] = {0};
uint32_t sendSigArr[200] = {0};
uint8_t sector[4096] = {0};
uint32_t data[3] = {20405, 12345, 54321};
uint32_t dataRead[1024] = {0};
uint16_t sendCnt = 0;

SPIF_HandleTypeDef spif;
IR_REMOTE_HandleTypeDef hir, hir2;
IR_REMOTE_MGMT_HandleTypeDef mgmt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CheckButtons()
{
	if((GPIOB->IDR & (1<<3)) == 0)
	{
		HAL_Delay(300);
		if((GPIOB->IDR & (1<<3)) == 0)
		{
			if (IR_REMOTE_MGMT_StartListening_IT(&mgmt) == IR_REMOTE_BAD_STATUS) return;
			ssd1306_SetCursor(30, 40);
			ssd1306_WriteChar('L', Font_16x24, White);
			ssd1306_UpdateScreen();
			HAL_Delay(100);
			while(mgmt.mode)
			{
				if(timeoutCnt++ >= 20000000) break;
			}
			if(timeoutCnt > 19999999)
			{
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteChar('X', Font_16x24, White);
				ssd1306_UpdateScreen();
				HAL_Delay(1000);
				ssd1306_SetCursor(0, 0);
				ssd1306_WriteChar(' ', Font_16x24, White);
				ssd1306_UpdateScreen();
				mgmt.mode = IR_REMOTE_MODE_IDLE;
				ssd1306_SetCursor(30, 40);
				ssd1306_WriteChar('I', Font_16x24, White);
				ssd1306_UpdateScreen();
				timeoutCnt = 0;
				return;
			}
			else
			{
				IR_REMOTE_PutSignal(&hir, sigArr, mgmt.currentSignalNum);
				for(int i = 0; i < 200; i++) sigArr[i] = 0;
				IR_REMOTE_PackMetadata(&hir);
				SPIF_EraseSector(&spif, 2);
				HAL_Delay(10);
				SPIF_WriteAddress(&spif, SPIF_SectorToAddress(2), (uint8_t*)hir.signalBlock, SPIF_SECTOR_SIZE);
				ssd1306_SetCursor(60,40);
				ssd1306_WriteChar('+', Font_16x24, White);
				SPIF_ReadAddress(&spif, SPIF_SectorToAddress(2), (uint8_t*)hir2.signalBlock, SPIF_SECTOR_SIZE);
				timeoutCnt = 0;
			}


			ssd1306_SetCursor(30, 40);
			ssd1306_WriteChar('I', Font_16x24, White);
			ssd1306_UpdateScreen();

		}
		else
		{
			HAL_Delay(50);
			IR_REMOTE_GetSignal(&hir, sendSigArr, mgmt.currentSignalNum);
			IR_REMOTE_MGMT_StartSending_IT(&mgmt);
		}

	}
	else if((GPIOB->IDR & (1<<4)) == 0)
	{
		HAL_Delay(300);
		if((GPIOB->IDR & (1<<4)) == 0)
				{
					HAL_Delay(50);
					if(IR_REMOTE_DeleteSignal(&hir, mgmt.currentSignalNum))
					{
					IR_REMOTE_PackMetadata(&hir);
					SPIF_EraseSector(&spif, 2);
					HAL_Delay(10);
					SPIF_WriteAddress(&spif, SPIF_SectorToAddress(2), (uint8_t*)hir.signalBlock, SPIF_SECTOR_SIZE);
					ssd1306_SetCursor(60,40);
					ssd1306_WriteChar('-', Font_16x24, White);
					SPIF_ReadAddress(&spif, SPIF_SectorToAddress(2), (uint8_t*)hir2.signalBlock, SPIF_SECTOR_SIZE);
					ssd1306_UpdateScreen();
					}

				}
				else
				{
					HAL_Delay(50);
					mgmt.currentSignalNum = (mgmt.currentSignalNum + 1) % 5;
					ssd1306_SetCursor(60,40);
				    if(hir.signalOffset[mgmt.currentSignalNum] == 0 || hir.signalOffset[mgmt.currentSignalNum] == 0xFFFFFFFF) ssd1306_WriteChar('-', Font_16x24, White);
					else ssd1306_WriteChar('+', Font_16x24, White);
					ssd1306_SetCursor(0, 40);
					ssd1306_WriteChar((char)(48+mgmt.currentSignalNum), Font_16x24, White);
					ssd1306_UpdateScreen();
				}

	}

}
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  SPIF_Init(&spif, &hspi1, SPI1_NSS_GPIO_Port, SPI1_NSS_Pin);
  ssd1306_Init();

  IR_REMOTE_FlushData(&hir);
  //SPIF_EraseSector(&spif, SPIF_SectorToAddress(2));
  //SPIF_WriteAddress(&spif, SPIF_SectorToAddress(2), (uint8_t*)hir.signalBlock, SPIF_SECTOR_SIZE);
  SPIF_ReadAddress(&spif, SPIF_SectorToAddress(2), (uint8_t*)hir.signalBlock, SPIF_SECTOR_SIZE);
  IR_REMOTE_UnpackMetadata(&hir);
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim11);

  mgmt.currentSignalNum = 0;
  mgmt.mode = IR_REMOTE_MODE_IDLE;

  ssd1306_SetCursor(0, 40);
  ssd1306_WriteChar((char)(48+mgmt.currentSignalNum), Font_16x24, White);
  ssd1306_SetCursor(30, 40);
  ssd1306_WriteChar('I', Font_16x24, White);
  ssd1306_SetCursor(60,40);
  if(hir.signalOffset[mgmt.currentSignalNum] == 0 || hir.signalOffset[mgmt.currentSignalNum] == 0xFFFFFFFF) ssd1306_WriteChar('-', Font_16x24, White);
  else ssd1306_WriteChar('+', Font_16x24, White);
  ssd1306_UpdateScreen();
  TIM11->CR1 &= ~(1);
  TIM11->CCR1 = 1316;
  TIM11->CCER = 0x0001;

  TIM5->CR1 &= ~(1);
  TIM5->SR = 0;
  TIM5->CNT = 0;
  TIM5->DIER = 1;
  TIM5->ARR = 120000;

  TIM2->SR = 0;
  TIM2->CR1 &= ~(1);
  TIM2->SR = 0;
  TIM2->CNT = 0;
  TIM2->DIER = 1;

  EXTI->IMR = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  CheckButtons();

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
