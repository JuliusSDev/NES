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

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include <stdio.h>
#define HTS221_ADDRESS 0x5F <<1
#define CTRL_REG1 0x20
#define STATUS_REG      0x27
#define TEMP_OUT_L      0x2A
#define TEMP_OUT_H      0x2B
#define TEMP0_CALIB_L   0x32
#define TEMP0_CALIB_H   0x33
#define TEMP1_CALIB_L   0x3C
#define TEMP1_CALIB_H   0x3D
#define T0_T1_MSB       0x35

void setup_HTS211(void){
  // sets sensor to on and the data stream to continuos at 12.5Hz
  uint8_t pData = 0x81;
  if(HAL_I2C_Mem_Write(&hi2c2, HTS221_ADDRESS, CTRL_REG1, 1, &pData, 1, HAL_MAX_DELAY)!= HAL_OK) while(1){};
}

float read_temperature_HTS211(void){
  // 1. Read the value of coefficients T0_degC_x8 and T1_degC_x8 from registers 0x32 & 0x33.
  uint8_t T0_degC_x8, T1_degC_x8;
  if (HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, 0x32,1,&T0_degC_x8,1,HAL_MAX_DELAY) != HAL_OK) while(1){};
  if (HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, 0x33,1,&T1_degC_x8,1,HAL_MAX_DELAY) != HAL_OK) while(1){};

  // 2. Divide by 8 the content of registers 0x32 (T0_degC_x8) and 0x33 (T1_degC_x8) in order to obtain the value
  // of coefficients T0_degC and T1_degCs
  uint8_t T0_degC = T0_degC_x8 / 8;
  uint8_t T1_degC = T1_degC_x8 / 8;

  // Step 3: Read the MSB bits of T1_degC and T0_degC from register 0x35 to compute T0_DegC and T1_DegC
  if (HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, T0_T1_MSB,1,&T0_degC_x8,1,HAL_MAX_DELAY) != HAL_OK) while(1){};
  T0_degC += ((T0_degC_x8 & 0x03) << 8) / 8.0;
  T1_degC += ((T0_degC_x8 & 0x0C) >> 2) * 256 / 8.0;

  // Step 4: Read from registers 0x3C & 0x3D the value of T0_OUT
  int16_t T0_out, T1_out;
  HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, TEMP1_CALIB_L, 1, (uint8_t *)&T0_out, 1, 1000);
  HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, TEMP1_CALIB_H, 1, (uint8_t *)&T0_out + 1, 1, 1000);

  // Step 5: Read the value of T1_OUT from registers 0x3E & 0x3F
  HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, TEMP1_CALIB_L + 2, 1, (uint8_t *)&T1_out, 1, 1000);
  HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, TEMP1_CALIB_H + 2, 1, (uint8_t *)&T1_out + 1, 1, 1000);

  // Step 6: Read the value T_OUT (ADC_OUT) from 0x2A & 0x2B registers
  uint8_t temp_out_l, temp_out_h;
  HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, TEMP_OUT_L, 1, &temp_out_l, 1, 1000);
  HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDRESS, TEMP_OUT_H, 1, &temp_out_h, 1, 1000);

  int16_t temp_raw = ((int16_t)temp_out_h << 8) | temp_out_l;

  // Step 7: Compute the T [degC] value by linear interpolation
  float temperature = ((float)(temp_raw - T0_out)) * (T1_degC - T0_degC) / (T1_out - T0_out) + T0_degC;
  return temperature;
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  setup_HTS211();
  
  uint8_t msg[128] = {'\0'};
  while (1)
  {
    
    float temp = read_temperature_HTS211();
    // recalculate due to issues with printing floats... I don't know why, but it gets cut out???
    uint16_t temp_int = (uint16_t)(temp * 10);
    snprintf((char*)msg, sizeof(msg), "Temp: %d.%d \r\n", temp_int/10, temp_int %10);
    HAL_UART_Transmit(&huart4,msg,sizeof(msg),HAL_MAX_DELAY);

    HAL_Delay(200);
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
