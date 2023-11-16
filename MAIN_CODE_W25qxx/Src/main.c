/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "w25qxx.h"
#include "w25qxxConf.h"
#include <string.h>

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
#define MAX_SECTOR_SIZE 4
#define MAX_DATA_SIZE   3
uint8_t Read_Buf[3];
uint8_t Write_Buf[]= {0,1,2};
uint8_t SectorNum=0,BlockSize=0,incValue=0,sectorSize=0;
uint16_t offsetByte=0;
int X=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
SectorNum = 1;
W25qxx_Init(); //function insitetited

while(BlockSize < 16) // block of the 16*4096 = 65536(total page of the block 64kb)
{
	HAL_GPIO_WritePin(TEST_PIN1_GPIO_Port,TEST_PIN3_Pin,GPIO_PIN_SET); //set the valve of pin3 in GPIO port
	W25qxx_EraseSector(SectorNum);//sectornum will be erase the sector 
	HAL_GPIO_WritePin(TEST_PIN1_GPIO_Port,TEST_PIN3_Pin,GPIO_PIN_RESET); //Reset the valve of pin3 in GPIO port 
	offsetByte = 0;
	while(sectorSize < 4) //sectorsize of the 4*16=4096(one page of the sector will splitted the single page4kb)
	{
		Write_Buf[0] = incValue;
		HAL_GPIO_WritePin(TEST_PIN1_GPIO_Port,TEST_PIN2_Pin,GPIO_PIN_SET);//set the valve of pin2 
		W25qxx_WriteSector(Write_Buf, SectorNum, offsetByte, MAX_DATA_SIZE);//writeing the valve of pin2		
		HAL_GPIO_WritePin(TEST_PIN1_GPIO_Port,TEST_PIN2_Pin,GPIO_PIN_RESET);//Reset the valve of pin2
		
		HAL_GPIO_WritePin(TEST_PIN1_GPIO_Port,TEST_PIN1_Pin,GPIO_PIN_SET);//set the valve of pin1
		W25qxx_ReadSector(Read_Buf, SectorNum, offsetByte, MAX_DATA_SIZE);//reading the valve of pin1	
		HAL_GPIO_WritePin(TEST_PIN1_GPIO_Port,TEST_PIN1_Pin,GPIO_PIN_RESET);//Reset the valve of pin1
		
		if(memcmp(Write_Buf, Read_Buf, MAX_DATA_SIZE)== 0)//compering the valve will same= 0 set will see and pass 
		{
		  sectorSize++;//increamenting sectorsize
			incValue++;//increamenting the incvalue
			offsetByte+=MAX_DATA_SIZE;// checking the offsetbyte + = size of max 
		}
	}
	 sectorSize=0;
	 SectorNum++;//increamenting the sectornum of the page in the note book
	 BlockSize++;// increamenting the blocksize of the page in the note book
}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(HAL_GPIO_ReadPin(GPIOA,PUSH_BUTTON1_Pin)==GPIO_PIN_RESET)//Button1 is pressed , Trun ON the led 
		{
			
			(HAL_GPIO_WritePin(GPIOA,LED_Pin,GPIO_PIN_SET));//if pressed led Switch ON
			X=1;//int x=1 the led is ON
		}
			if(HAL_GPIO_ReadPin(GPIOA,PUSH_BUTTON2_Pin)==GPIO_PIN_RESET)//Button2 is pressed, trun OFF the led 
		{
			
			(HAL_GPIO_WritePin(GPIOA,LED_Pin,GPIO_PIN_RESET));//if pressed led Switch OFF
			X=0;//int x=0 the led is OFF
		}
		
		//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);//continue in-built led will be toggle with GPIOC 
   // HAL_Delay(500);//delay 0.5 second 
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV16;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
