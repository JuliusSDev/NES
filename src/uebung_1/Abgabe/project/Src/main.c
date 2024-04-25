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
#include "stdbool.h"
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
void showDigit(uint8_t digit){
#define A SEGA_Pin
#define B SEGB_Pin
#define C SEGC_Pin
#define D SEGD_Pin
#define E SEGE_Pin
#define F SEGF_Pin
#define G SEGG_Pin
  uint16_t bitmap = 0x0000;
  switch (digit)
  {
  case 0:
    bitmap = A | B | C | D | E | F;
    break;
  case 1:
    bitmap = B | C;
    break;
  case 2:
    bitmap = A | B | D | E | G;
    break;
  case 3:
    bitmap = A | B | C | D | G;
    break;
  case 4:
    bitmap = B | C | F | G;
    break;
  case 5:
    bitmap = A | C | D | F | G;
    break;
  case 6:
    bitmap = A | C | D | E | F | G;
    break;
  case 7:
    bitmap = A | B | C;
    break;
  case 8:
    bitmap = A | B | C | D | E | F | G;
    break;
  case 9:
    bitmap = A | B | C | D |F | G;
    break;
  default:
    bitmap = A | D | E | F | G;
    HAL_GPIO_WritePin(GPIOJ, bitmap, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOJ, ~(bitmap| SEGDP_Pin), GPIO_PIN_RESET);
    while (1)
    {
     HAL_GPIO_TogglePin(GPIOJ, SEGDP_Pin);
     HAL_Delay(50); 
    }
    break;
  }
    HAL_GPIO_WritePin(GPIOJ, bitmap, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOJ, ~(bitmap| SEGDP_Pin), GPIO_PIN_RESET);
}

void showNumber(uint32_t numberToPrint, uint32_t delay){
  // Handle the case where numberToPrint is 0 separately
  if (numberToPrint == 0) {
      showDigit(0);
      return;
  }

  bool first = true;
  if(numberToPrint < 10){
    delay = 0;
    first = false;
  } 

  // Temporary storage for digits
  uint8_t digits[10]; // Maximum number of digits in a 32-bit number is 10
  int numDigits = 0;

  // Extract digits from numberToPrint
  while (numberToPrint > 0) {
      digits[numDigits++] = numberToPrint % 10;
      numberToPrint /= 10;
  }
  // Print digits in reverse order
  for (int i = numDigits - 1; i >= 0; i--) {
      if(first){
        HAL_GPIO_WritePin(GPIOJ, SEGDP_Pin, GPIO_PIN_SET);
        first = false;
      }else{
        HAL_GPIO_WritePin(GPIOJ, SEGDP_Pin, GPIO_PIN_RESET);
      }
      showDigit(digits[i]);
      HAL_Delay(delay);
  }
}

enum error_msg{
  UNKNOWN_STATE = 0,
};
// error codes: 
// error 4 = unknown behavior/unknown state
void error(enum error_msg errorcode){
#define A SEGA_Pin
#define B SEGB_Pin
#define C SEGC_Pin
#define D SEGD_Pin
#define E SEGE_Pin
#define F SEGF_Pin
#define G SEGG_Pin
  while (1)
  {
    // 
        HAL_GPIO_WritePin(GPIOJ, A | F | G | E | D, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOJ, C | B, GPIO_PIN_RESET);
        HAL_Delay(1000);
        showNumber(errorcode, 1000U);
        HAL_Delay(1000);

  }
  
}

void showLetter(char letter){
#define A SEGA_Pin
#define B SEGB_Pin
#define C SEGC_Pin
#define D SEGD_Pin
#define E SEGE_Pin
#define F SEGF_Pin
#define G SEGG_Pin
    if (letter == 'a' || letter == 'A')
    {
        HAL_GPIO_WritePin(GPIOJ, A | B | C | E | F | G, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOJ, D, GPIO_PIN_RESET);
    } else if (letter == 'b' || letter == 'B')
    {
        HAL_GPIO_WritePin(GPIOJ, C | D | E | F | G, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOJ, A | B, GPIO_PIN_RESET);
    } else if (letter == 'c' || letter == 'C')
    {
        HAL_GPIO_WritePin(GPIOJ, A | D | E | F, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOJ, B | C | G, GPIO_PIN_RESET);
    } else if (letter == 'd' || letter == 'D')
    {
        HAL_GPIO_WritePin(GPIOJ, B | C | D | E | G, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOJ, A | F, GPIO_PIN_RESET);
    } else if (letter == 'e' || letter == 'E')
    {
        HAL_GPIO_WritePin(GPIOJ, A | F | G | E | D, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOJ, C | B, GPIO_PIN_RESET);
    } else if (letter == 'f' || letter == 'F')
    {
        HAL_GPIO_WritePin(GPIOJ, A | F | G | E, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOJ, C | B | D, GPIO_PIN_RESET);
    } else if (letter == 'g' || letter == 'G')
    {
        HAL_GPIO_WritePin(GPIOJ, A | B | C | D | F | G, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOJ, E, GPIO_PIN_RESET); 
    } else if (letter == 'h' || letter == 'H')
    {
        HAL_GPIO_WritePin(GPIOJ, B | C | G | F | E, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOJ, A | D, GPIO_PIN_RESET);
    } else if (letter == 'i' || letter == 'I')
    {
        HAL_GPIO_WritePin(GPIOJ, A | C, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOJ, B | D | E | F | G, GPIO_PIN_RESET);
    } else {
      error(1);
    }
    
}

void showString(const char* string, uint32_t delay){
  if(sizeof(string) < 1) return;
  char *string_pointer = &string[0];
  while (*string_pointer != '\0')
  {
    showLetter(*string_pointer);
    ++string_pointer;
    HAL_Delay(delay);
  }
  
}

bool isButtonPressed(uint16_t buttonID){
  if(!HAL_GPIO_ReadPin(GPIOJ, buttonID)){
      HAL_Delay(400);
      return true;
  }
  return false;
}

void blink(uint8_t dutyCycle, uint32_t cycleTime, uint16_t outPIN){
  int activeTime = ( dutyCycle * cycleTime ) / 100;

  HAL_GPIO_WritePin(GPIOJ, SEGDP_Pin, GPIO_PIN_SET);
  HAL_Delay(activeTime);

  HAL_GPIO_WritePin(GPIOJ, SEGDP_Pin, GPIO_PIN_RESET);
  HAL_Delay(cycleTime - activeTime);
}

void task_A(void){
  int cycleTime = 3000;
  int dutyCycle = 67;
  int activeTime = ( dutyCycle * cycleTime ) / 100;

  HAL_GPIO_WritePin(GPIOJ, SEGDP_Pin, GPIO_PIN_SET);
  HAL_Delay(activeTime);

  HAL_GPIO_WritePin(GPIOJ, SEGDP_Pin, GPIO_PIN_RESET);
  HAL_Delay(cycleTime - activeTime);
}

void task_B(void){
  uint8_t dutyCycle = 67;
  static uint8_t stateCounter = 0;
  static uint8_t state = 0;
  stateCounter++;
  if(stateCounter > 15){
    state++;
    if(state > 2) state = 0;
    stateCounter = 0;
  }
  
  #define LOW 1000
  #define MID 500
  #define HIGH 200
  uint16_t cycleTime = LOW;
  switch (state)
  {
  case 0:
    cycleTime = LOW;
    break;
  case 1:
    cycleTime = MID;
    break;
  case 2:
    cycleTime = HIGH;
    break;
  
  default:
    error(UNKNOWN_STATE);
    break;
  }
  blink(dutyCycle, cycleTime, SEGDP_Pin);
}

void task_C(void){
  static uint8_t stateCounter = 0;
  static uint8_t state = 0;
  stateCounter++;
  if(stateCounter > 15){
    state++;
    if(state > 2) state = 0;
    stateCounter = 0;
  }
  
  int cycleTime = 2000;
  uint8_t dutyCycle = 0;
  switch (state)
  {
  case 0:
    dutyCycle = 70;
    break;
  case 1:
    dutyCycle = 50;
    break;
  case 2:
    dutyCycle = 30;
    break;
  
  default:
    error(UNKNOWN_STATE);
    break;
  }
  blink(dutyCycle, cycleTime, SEGDP_Pin);
}

void task_D(void){
  static uint16_t cycleTime = 100;
  uint8_t dutyCycle = 50;
  if(isButtonPressed(BTN1_Pin))
  {
    if(cycleTime <= 900) cycleTime += 100;
  }
  if(isButtonPressed(BTN2_Pin))
  {
    if(cycleTime >= 200) cycleTime -= 100;
  }
  blink(dutyCycle, cycleTime, SEGDP_Pin);
}

void task_E(void){
  static uint16_t cycleTime = 100;
  static uint8_t stateNumber = 0;
  uint8_t dutyCycle = 50;
  if(isButtonPressed(BTN1_Pin) && cycleTime <= 1700)
  {
    cycleTime += 200;
    stateNumber++;
  }
  if(isButtonPressed(BTN2_Pin) && cycleTime >= 300)
  {
    cycleTime -= 200;
    stateNumber--;
  }
  showDigit(stateNumber);
  blink(dutyCycle, cycleTime, SEGDP_Pin);
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
  

  while (1)
  {
    task_A();
    //task_B();
    //task_C();
    //task_D();
    //task_E();
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
