/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "math.h"
#define PORT_RESET ((GPIO_TypeDef *) GPIOD)
#define PORT_ENABLE ((GPIO_TypeDef *) GPIOD)
#define PORT_LATCH ((GPIO_TypeDef *) GPIOD)
#define PIN_RESET  ((uint16_t) GPIO_PIN_9)
#define PIN_ENABLE  ((uint16_t) GPIO_PIN_11)
#define PIN_LATCH  ((uint16_t) GPIO_PIN_10)
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef enum {
  D0 = 0xFC0,
  D1 = 0x600,
  D2 = 0xDA0,
  D3 = 0xF20,
  D4 = 0x660,
  D5 = 0xB60,
  D6 = 0xBE0,
  D7 = 0xE00,
  D8 = 0xFE0,
  D9 = 0xF60,
  D10 = 0xC60,
} DIGITS;
typedef enum {
  K1 = 0x8,
  K2 = 0x4,
  K3 = 0x2,
  K4 = 0x1,  
} KATODS;
typedef enum {
  ON,
  OFF    
} state_t;
uint8_t sensivity_light = 4;
state_t mute_state = OFF;
state_t buzzer_state = OFF;
uint8_t buzzer_done = 0;
uint8_t timer_buzzer = 0;
uint8_t timer_flag = 0;
uint8_t pData[3];
int tim = 0;
int spi = 0;
uint8_t hour = 0;
uint16_t angle = 0;
uint16_t old_angle = 0;
DIGITS digits [11] = {D0,D1,D2,D3,D4,D5,D6,D7,D8,D9,D10};
KATODS digits_k [4] = {K1,K2,K3,K4};
int flag_zerro = 0;
uint8_t shift = 0;
char reset = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void reset_led()
{
  old_angle = 0;
  pData[2] &= 0;
  pData[1] &= 0;
  pData[0] &= 0;  
}
int pow_digit(uint8_t base,uint8_t degree)
{
  int pow = 1;
  for (int i = 1; i <= degree; i++)
    pow *= base;
  return pow;
}     
void shift_katode()
{  
  pData[0] |= digits_k[shift];
  shift ++;
  if (shift > 3)
  {
    angle = old_angle ;
    shift = 0;
    flag_zerro = 0;
  }
}
void send_Data(SPI_HandleTypeDef *hspi)
{ 
  HAL_GPIO_WritePin(PORT_ENABLE, PIN_ENABLE,GPIO_PIN_SET);
  HAL_SPI_Transmit(hspi, pData, 3, 1);
  HAL_GPIO_WritePin(PORT_LATCH, PIN_LATCH,GPIO_PIN_SET);
  HAL_GPIO_WritePin(PORT_LATCH, PIN_LATCH,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PORT_ENABLE, PIN_ENABLE,GPIO_PIN_RESET);
 }
void make_digits_reset(uint8_t right, uint8_t left)
{
  pData[1] &= left;
  pData[0] &= right;
}
void make_digits(uint8_t right, uint8_t left)
{
  pData[1] |= left;
  pData[0] |= right;  
}
void buzzer (uint8_t hour)
{
  if (buzzer_state == OFF && buzzer_done != 1)
  {
  buzzer_state = ON;
  timer_buzzer = hour;
  HAL_TIM_Base_Start_IT(&htim13);
  }
  else
  {
  __NOP();
  }
}
void hour_to_led (uint8_t hour, state_t mute)
{
  if (mute == OFF)
  {
    buzzer(hour);
  }
  uint8_t base_hour = 0x80; // 10000000 - 1hour
  uint8_t shift = 0;
  if (hour <= 8)
  {
    shift = base_hour >> hour-1;
    pData[2] = pData[2] | shift;
  }
  else if (8 < hour <= 12)
  {
    shift = base_hour >> hour - 9;
    pData[1] = pData[1] | shift;
  }
}

void angle_to_indicator (uint8_t shift)
{
  
  uint16_t base_digits = 0xF000;
  uint8_t digits_right = base_digits;
  uint8_t digits_left = base_digits >> 8;
  make_digits_reset (digits_right, digits_left);
  if (shift < 3 )
  {
    uint16_t digit_num[3];
    int pow = pow_digit(10,shift);
    digit_num[shift] = angle / (100 / pow);
    if (digit_num[shift] == 0 && shift == 0)
    {
      flag_zerro = 1;
      make_digits (0, 0);
    }
    else if(digit_num[shift] == 0 && flag_zerro == 1)
    {
      make_digits (0, 0);
    }
    else
    {
      flag_zerro = 0;
      angle -= digit_num[shift] * (100 / pow);
      digit_num[shift] = digits[digit_num[shift]];
      digits_right = digit_num[shift];
      digits_left = digit_num[shift] >> 8;
      make_digits (digits_right, digits_left);
    }
    
    
  }
  else if (shift == 3 && old_angle != 0)
  {
    digits_right = digits[10];
    digits_left = digits[10] >> 8;
    make_digits (digits_right, digits_left);
  }
}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_SPI3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();

  /* USER CODE BEGIN 2 */
HAL_GPIO_WritePin(PORT_RESET, PIN_RESET,GPIO_PIN_SET);
HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,GPIO_PIN_SET);
angle = old_angle ;
HAL_TIM_PWM_Start_IT(&htim12,TIM_CHANNEL_2);

HAL_TIM_Base_Start_IT(&htim6);
HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
HAL_ADC_Start_IT(&hadc2);
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* ADC2 init function */
void MX_ADC2_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
  hadc2.Init.Resolution = ADC_RESOLUTION12b;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc2);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

}

/* SPI3 init function */
void MX_SPI3_Init(void)
{

  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi3.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi3);

}

/* TIM6 init function */
void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 52500;
  HAL_TIM_Base_Init(&htim6);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig);

}

/* TIM7 init function */
void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 2600;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65115;
  HAL_TIM_Base_Init(&htim7);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig);

}

/* TIM12 init function */
void MX_TIM12_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim12);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 30758;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1);

  sConfigOC.Pulse = 0;
  HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_2);

}

/* TIM13 init function */
void MX_TIM13_Init(void)
{

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 70;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 50000;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim13);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOE_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 PD11 PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  //HAL_GPIO_WritePin(PORT_LATCH, PIN_LATCH,GPIO_PIN_SET);
 // HAL_GPIO_WritePin(PORT_LATCH, PIN_LATCH,GPIO_PIN_RESET);
 // HAL_GPIO_WritePin(PORT_ENABLE, PIN_ENABLE,GPIO_PIN_RESET);
//spi++;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ 
  if (htim == &htim6)
  {
    angle_to_indicator(shift);
    hour_to_led(hour, mute_state);
    
    shift_katode();
    send_Data(&hspi3);
    tim++; 
    if (tim == 1000 && reset == 1)
    {
      reset = 0;
      hour = 0;
      old_angle = 0;
      shift = 0;
      angle_to_indicator(shift);
      hour_to_led(hour, mute_state);
      tim = 0;
    }
    else if (tim == 1000)
    {
      tim = 0;
    }
  }
  if (htim == &htim7)
    {
      hour += 1;
      buzzer_done =0;     
      old_angle += 30;
      if (hour > 12)
      {
        reset_led();
        hour = 0;
        old_angle = 0;
      }
    }
  if (htim == &htim13)
  {
  switch (buzzer_state)
  {
    case OFF:
      break;
    case ON:
    if (timer_buzzer != 0 && timer_buzzer <= 12)
    {
      if(timer_flag == 1)
      {
        HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);
        timer_flag = 0;
      }
      else 
      {
        HAL_TIM_PWM_Stop(&htim12,TIM_CHANNEL_1); 
        timer_flag = 1;
        timer_buzzer -- ;
      }
    }
    else 
    {
      buzzer_state = OFF;
      buzzer_done = 1;
    }
    break;
  }
  }

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_11)
  {
    if (sensivity_light == 4)
    {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
    sensivity_light*=3; 
    }
    else
    {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
    sensivity_light/=3; 
    }
  }
  else if (GPIO_Pin == GPIO_PIN_8)
  {
    if (mute_state == OFF)
    {
    mute_state = ON;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    }
    else if(mute_state == ON)
    {
    mute_state = OFF;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    }
  }
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
uint32_t light_value;
light_value = HAL_ADC_GetValue(hadc);
uint32_t ligt_sensivity = light_value*sensivity_light;
htim12.Instance->CCR2 = ligt_sensivity;
}


/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
