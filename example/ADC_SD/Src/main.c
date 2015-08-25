/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 16/10/2014 19:48:29
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
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
#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h" /* defines SD_Driver as external */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

SPI_HandleTypeDef hspi1;

uint8_t SD_DriverNum;      /* FatFS SD part */
char SD_Path[4];           /* SD logical drive path */

/* USER CODE BEGIN 0 */
typedef enum {
    IDLE,
    INIT_RECORD,
    RECORD,
    RECORD_READY,
    READ_BUFFER,
    SEND_SPI
} state_t;

#define DATA_SIZE            0x8000
#define READ_DATA_SIZE       0x1000

#define CS      GPIO_PIN_0
#define UD      GPIO_PIN_1
#define DELAY   1



volatile unsigned short ADCConvValues[DATA_SIZE];
unsigned char ReadBuff[READ_DATA_SIZE];

state_t state = IDLE;
volatile uint8_t file_writing = 0;
volatile uint32_t bytes_recorded = 0; 
volatile uint32_t bytes_read = 0;

FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */

static void Error_Handler(void);

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);

int main(void)
{

  /* USER CODE BEGIN 1 */
  static uint32_t index = 0;
  UINT rbytes;
 
 
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* System interrupt init*/
  /* Sets the priority grouping field */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
  
  HAL_GPIO_WritePin(GPIOC, CS, GPIO_PIN_SET);
  
  HAL_Delay(DELAY);
  HAL_GPIO_WritePin(GPIOC, UD, GPIO_PIN_SET);
  HAL_Delay(DELAY);
  HAL_GPIO_WritePin(GPIOC, CS, GPIO_PIN_RESET);
  HAL_Delay(DELAY);   
  HAL_GPIO_WritePin(GPIOC, UD, GPIO_PIN_RESET);
  HAL_Delay(DELAY); 
  
  for (unsigned char i = 0; i < 64; i++)
  {
      HAL_GPIO_WritePin(GPIOC, UD, GPIO_PIN_SET);
      HAL_Delay(DELAY);
      HAL_GPIO_WritePin(GPIOC, UD, GPIO_PIN_RESET);
      HAL_Delay(DELAY);
  }
  HAL_GPIO_WritePin(GPIOC, CS, GPIO_PIN_SET);
  HAL_Delay(DELAY);
  HAL_GPIO_WritePin(GPIOC, UD, GPIO_PIN_RESET);
  HAL_Delay(DELAY);
  HAL_GPIO_WritePin(GPIOC, CS, GPIO_PIN_RESET);
  HAL_Delay(DELAY);
  
  for (unsigned char i = 0; i < 10; i++)
  {
      HAL_GPIO_WritePin(GPIOC, UD, GPIO_PIN_SET);
      HAL_Delay(DELAY);	
      HAL_GPIO_WritePin(GPIOC, UD, GPIO_PIN_RESET);
      HAL_Delay(DELAY);
  }
  HAL_GPIO_WritePin(GPIOC, CS, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /*## FatFS: Link the SD disk I/O driver ###############################*/

  /* USER CODE BEGIN 3 */

  if(FATFS_LinkDriver(&SD_Driver, SD_Path) == 0)
  {  
      /*##-2- Register the file system object to the FatFs module ##############*/
      if(f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0) != FR_OK)
      {
          /* FatFs Initialization Error */
          Error_Handler();
      }
      
  }

  
  SPI1->DR = 0;
  /* Infinite loop */
  while (1)
  {

      switch (state)
      {
      //===================================================
      case IDLE:
          if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)))
          { 
              state = INIT_RECORD;
          }
          break;
      //===================================================    
      case INIT_RECORD:
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
          if(f_open(&MyFile, "data.bin", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
          {
              Error_Handler();
          }
          HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCConvValues, DATA_SIZE);
          file_writing = 1;
          bytes_recorded = 0;
          state = RECORD;         

          break;
      //===================================================
      case RECORD:
          {
              if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
              {
                  
                  state = RECORD_READY;  
                  while (file_writing) {};
                  f_close(&MyFile);
              }
          }
          break;
          
      //===================================================
      case RECORD_READY:
          bytes_read = 0;
          if(f_open(&MyFile, "data.bin", FA_READ) != FR_OK)
          {
              Error_Handler();
          }
          else
          {
             state = READ_BUFFER; 
          }
          
          break;
          
      //===================================================
      case READ_BUFFER:
          if ( bytes_read < (bytes_recorded * 2))
          {
              if (f_read(&MyFile,(void *) ReadBuff, READ_DATA_SIZE, &rbytes) == FR_OK)
              {
                  if (rbytes == READ_DATA_SIZE)
                  {
                      index = 0;
                      
                      
                      SPI1->DR = ReadBuff[0];
                      state = SEND_SPI;
                  }
                  else
                  {
                      //error 
                  }
              }
              else
              {
                  //error
              }
          }
          else
          {
              //f_close(&MyFile);
              //state = RECORD_READY;
          }
          
          
          if ((HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0)))
          { 
              state = INIT_RECORD;
              f_close(&MyFile);
          }
          
          
          break;
          
      case SEND_SPI:       
          //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
          GPIOA->ODR |= GPIO_PIN_2;
          if ((SPI1->SR & SPI_SR_TXE)&&(SPI1->SR & SPI_SR_RXNE))
          {
              index++;
              SPI1->DR = ReadBuff[index];
              //SPI1->DR = (uint8_t)(index & 0xFF);
              
              //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
              GPIOA->ODR &= ~GPIO_PIN_2;
              
              if (index == (READ_DATA_SIZE - 1))
              {
                  state = READ_BUFFER;
                  bytes_read += READ_DATA_SIZE;
                  
              }
                       
          }
          
          break;
      //===================================================    
      }


      
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* SDIO init function */
void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  HAL_SD_Init(&hsd, &SDCardInfo);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  HAL_SPI_Init(&hspi1);
    /* Check if the SPI is already enabled */ 
  if((hspi1.Instance->CR1 &SPI_CR1_SPE) != SPI_CR1_SPE)
  {
      /* Enable SPI peripheral */
      __HAL_SPI_ENABLE(&hspi1);
  }

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* Sets the priority grouping field */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
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
  __GPIOD_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
    /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); 
    uint32_t byteswritten;
    
    
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   
    f_write(&MyFile, (const void *)(ADCConvValues), DATA_SIZE, (void *)&byteswritten);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); 
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    uint32_t byteswritten;
    
    
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);   
    f_write(&MyFile,(const char * )(&ADCConvValues[DATA_SIZE / 2]), DATA_SIZE, (void *)&byteswritten);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); 
    
    bytes_recorded += DATA_SIZE;
        
    if (state == RECORD_READY)
    {
        HAL_ADC_Stop_DMA(&hadc1); 
        file_writing = 0;
    }

}

static void Error_Handler(void)
{
  /* Turn LED3 on */
  while(1)
  {
  }
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
