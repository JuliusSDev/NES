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
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "hrtim.h"
#include "i2c.h"
#include "usart.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usb_otg.h"
#include "gpio.h"
#include "fmc.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
#include <stdio.h>
#define HTS221_ADDRESS 0x5F <<1
#define CTRL_REG1 0x20
#define HUMIDITY_OUT_L 0x28
#define HUMIDITY_OUT_H 0x29
#define H0_rH_x2 0x30
#define H1_rH_x2 0x31
#define H0_T0_OUT_L 0x36
#define H0_T0_OUT_H 0x37
#define H1_T0_OUT_L 0x3A
#define H1_T0_OUT_H 0x3B
void setup_HTS211(void){
  // sets sensor to on and the data stream to continuos at 12.5Hz
  uint8_t pData = 0x83;
  if(HAL_I2C_Mem_Write(&hi2c2, HTS221_ADDRESS, CTRL_REG1, 1, &pData, 1, HAL_MAX_DELAY)!= HAL_OK) while(1){};
}

float read_humidity_HTS211(void){
  // 1. Read the value of coefficients H0_rH_x2 and H1_rH_x2 from registers 0x30 & 0x31
  uint8_t H0_rH_x2_val, H1_rH_x2_val;

  if (HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, H0_rH_x2,1,&H0_rH_x2_val,1,HAL_MAX_DELAY) != HAL_OK) while(1){};
  if (HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, H1_rH_x2,1,&H1_rH_x2_val,1,HAL_MAX_DELAY) != HAL_OK) while(1){};

  // 2. Divide by two the content of registers 0x30 (H0_rH_x2) and 0x31 (H1_rH_x2) in order to obtain the value of
  //    coefficients H0_rH and H1_rH
  float H0_rH = H0_rH_x2_val / 2.0;
  float H1_rH = H1_rH_x2_val / 2.0;

  // 3. Read the value of H0_T0_OUT from registers 0x36 & 0x37
  uint8_t H0_T0_OUT_L_val, H0_T0_OUT_H_val;

  if (HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, H0_T0_OUT_L,1,&H0_T0_OUT_L_val,1,HAL_MAX_DELAY) != HAL_OK) while(1){};
  if (HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, H0_T0_OUT_H,1,&H0_T0_OUT_H_val,1,HAL_MAX_DELAY) != HAL_OK) while(1){};

  // 4. Read the value of H1_T0_OUT from registers 0x3A & 0x3B
  uint8_t H1_T0_OUT_L_val, H1_T0_OUT_H_val;
  if (HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, H1_T0_OUT_L,1,&H1_T0_OUT_L_val,1,HAL_MAX_DELAY) != HAL_OK) while(1){};
  if (HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, H1_T0_OUT_H,1,&H1_T0_OUT_H_val,1,HAL_MAX_DELAY) != HAL_OK) while(1){};

// 5. Read the humidity value in raw counts H_T_OUT from registers 0x28 & 0x29
  uint8_t HUMIDITY_OUT_L_val, HUMIDITY_OUT_H_val;
  if (HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, HUMIDITY_OUT_L,1,&HUMIDITY_OUT_L_val,1,HAL_MAX_DELAY) != HAL_OK) while(1);
  if (HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, HUMIDITY_OUT_H,1,&HUMIDITY_OUT_H_val,1,HAL_MAX_DELAY) != HAL_OK) while(1);

  int16_t H0_T0_OUT = (H0_T0_OUT_H_val << 8) | H0_T0_OUT_L_val;
  int16_t H1_T0_OUT = (H1_T0_OUT_H_val << 8) | H1_T0_OUT_L_val;

  int16_t HUMIDITY_OUT = (HUMIDITY_OUT_H_val << 8) | HUMIDITY_OUT_L_val;

  // 6. Compute the RH [%] value, by linear interpolation, applying the formula below:
  float humidity = H0_rH + (((HUMIDITY_OUT - H0_T0_OUT) * (H1_rH - H0_rH)) / (H1_T0_OUT - H0_T0_OUT));
  return humidity;
}
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
/* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_DAC1_Init();
  MX_FMC_Init();
  MX_HRTIM_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_I2C4_Init();
  MX_LPUART1_UART_Init();
  MX_UART4_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART1_Init();
  MX_USART2_UART_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI5_Init();
  MX_SPI6_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USB_OTG_FS_USB_Init();



  // main-function
  setup_HTS211();

  uint8_t msg[128] = {'\0'};
  while (1)
  {
    float humidity = read_humidity_HTS211();

    // recalculate due to issues with printing floats... I don't know why, but it gets cut out???
    uint16_t humidity_int = (uint16_t)(humidity * 10);
    snprintf((char*)msg, sizeof(msg), "Humid: %d.%d \r\n", humidity_int/10, humidity_int %10);
    HAL_UART_Transmit(&huart4,msg,sizeof(msg),HAL_MAX_DELAY);

    HAL_Delay(200);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 9;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 3072;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
