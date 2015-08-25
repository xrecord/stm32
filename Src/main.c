/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 15/05/2015 16:18:52
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
#include "fatfs.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypedef SDCardInfo;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
#define DATA_SIZE       0x1008
typedef enum {
  IDLE,
  INIT_RECORD,
  RECORD,
  RECORD_READY,
  READ_BUFFER       
} state_t;
typedef enum {
  READY = 0x01,
  START,
  STOP,
  ERROR_SD,
  ERROR_SYNC,
  WRITING,
  RESET_POWER,
  OK,
  HARD_ERROR
} COMMAND;

//-----------------------------------//
FRESULT res; 
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
FIL MyFile_time;     /* File object */
uint8_t SD_DriverNum;
//char SD_Path[4]/*={'1',':','/','\0'}*/; /* SD card logical drive path */
DWORD size; 
char last[4] = {'0','0','1','\0'};
char readbuf[2] = {'0','\0'};
int count_power;
uint32_t byteswritten, bytesread;
uint16_t ADCConvValues[DATA_SIZE];
extern volatile uint16_t DataReceived;
char nameOfFile[];
char nameOfFile_send[];
char timeOfFile[];
int countData = 0;
//----------------------------------//

state_t state = IDLE;
volatile uint8_t file_writing = 0;

RTC_TimeTypeDef sTimeNow;
RTC_DateTypeDef sDateNow;
//---------------UART------------------//
uint8_t aTxBuffer[2]; 
uint8_t aRxBuffer[2];
HAL_StatusTypeDef res_uart;
//---------------UART_END------------------//
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);

/* USER CODE BEGIN PFP */
void hardfault_handler(void)
{
  aTxBuffer[0] = HARD_ERROR;
  int i;
  for (i=0; i<1000; i++)
  {
    HAL_UART_Transmit(&huart3, (uint8_t*)aTxBuffer, 1,250);
  }
  NVIC_SystemReset();
}
static void delay(void)
{
  for (int i=0; i < 24000000; i++)
      {
        __NOP();        
      }
}
static void send_ok (void)
{
  aTxBuffer[0] = OK;
  int i;
  for (i=0; i<1000; i++)
  {
    HAL_UART_Transmit(&huart3, (uint8_t*)aTxBuffer, 1,500);
  }
}
static void Error_HandlerSD(void)
{
  HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 1);
  while(1)
  {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
    aTxBuffer[0] = ERROR_SD;
    HAL_UART_Transmit(&huart3, (uint8_t*)aTxBuffer, 1,250);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    delay();
    HAL_UART_Transmit(&huart3, (uint8_t*)aTxBuffer, 1,250);
    delay();
    
  }
}
static void Error_HandlerADC(void)
{
  HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 1);
  while(1)
  {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    delay();
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
    delay();
  }
}
static void Error_HandlerSync(void)
{
  HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 1);
    while(1)
    {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
      while (1)
        aTxBuffer[0] = ERROR_SYNC;
      HAL_UART_Transmit(&huart3, (uint8_t*)aTxBuffer, 1,500);
      
    }
}
static void Error_Handler_UART(void)
{
__NOP();
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

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
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_FATFS_Init();

  /* USER CODE BEGIN 2 */
res = HAL_IWDG_Start(&hiwdg);
    /*##-2- Register the file system object to the FatFs module ##############*/
    if((res = f_mount(&SDFatFs, (TCHAR const*)SD_Path, 0)) != FR_OK)
    {
      /* FatFs Initialization Error */
      Error_HandlerSD();
    }    
    if(f_open(&MyFile, "INIT.TXT", FA_READ) != FR_OK)
    {
      if((res = f_open(&MyFile, "INIT.TXT", FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK)
      {
        /* 'INIT.TXT' file Open for write Error */
        Error_HandlerSD();
      }
      else 
      {  /*##-5- Write data to the text file ################################*/
      f_write(&MyFile, last, sizeof(last), (void *)&byteswritten);
      f_close(&MyFile);
      }
    }
    else
    {

      f_read(&MyFile, last, sizeof(last), (UINT*)&bytesread);
      f_close(&MyFile);
      if(f_open(&MyFile, "INIT.TXT", FA_WRITE) != FR_OK)
      {
        /* 'STM32.TXT' file Open for write Error */
        Error_HandlerSD();
      }
      else
      {
        count_power = atoi(last);
        count_power +=1;
        if (count_power < 10)
        {
          sprintf(last, "00%d", count_power);
        }
        else if (count_power < 100)
        {
        sprintf(last, "0%d", count_power);
        }
        else
        {
          sprintf(last, "%d", count_power);
        }
        f_write(&MyFile, last, sizeof(last), (void *)&byteswritten);
        f_close(&MyFile);
      }
    }
if(__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET)
 {
 state = INIT_RECORD ;
 __HAL_RCC_CLEAR_RESET_FLAGS();
 }

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
  if(res_uart = HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 1) != HAL_OK)
  {
    Error_Handler_UART();
  }
 
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_IWDG_Refresh(&hiwdg);
    switch (state)
      //----------------------------------------//
    {
    case IDLE:
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1)
      {
        state = INIT_RECORD;
        
      }
      break;
      //----------------------------------------//
    case INIT_RECORD:
      HAL_RTC_GetTime(&hrtc, &sTimeNow, RTC_FORMAT_BIN);
      HAL_RTC_GetDate(&hrtc, &sDateNow, RTC_FORMAT_BIN);
      char nameOfFile[] = {(last[0]),(last[1]),(last[2]),'_',((sTimeNow.Minutes/10)+48),((sTimeNow.Minutes%10)+48),'h','_',(sTimeNow.Minutes+48),'m','_',((sTimeNow.Seconds/10)+48),((sTimeNow.Seconds%10)+48),'s','\0'};
      if((res = f_open(&MyFile, nameOfFile, FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK)
      {
        Error_HandlerSD();
      } 
      char timeOfFile[] = {(last[0]),(last[1]),(last[2]),'_',(sTimeNow.Hours+48),'h','_',((sTimeNow.Minutes/10)+48),((sTimeNow.Minutes%10)+48),'m','_',((sTimeNow.Seconds/10)+48),((sTimeNow.Seconds%10)+48),'s','_','t','.','t','x','t','\0'};
      if((res = f_open(&MyFile_time, timeOfFile, FA_CREATE_ALWAYS | FA_WRITE)) != FR_OK)
      {
        Error_HandlerSD();
      } 

HAL_Delay(240);

      if(HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADCConvValues, DATA_SIZE) != HAL_OK) 
      {
        Error_HandlerADC();
      }
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
      state = RECORD;
      file_writing = 1;

      break;
  
      //----------------------------------------//
    case RECORD:
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1)
        {
          state = RECORD_READY;
        }
        if (state == RECORD_READY)
        {
          while (file_writing) {};
          f_close(&MyFile);
          f_close(&MyFile_time);
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

        }
 
      break;
}

  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

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
  sConfig.Rank = 3;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* IWDG init function */
void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 512;
  HAL_IWDG_Init(&hiwdg);

}

/* RTC init function */
void MX_RTC_Init(void)
{

  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  RTC_AlarmTypeDef sAlarm;

    /**Initialize RTC and set the Time and Date 
    */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  HAL_RTC_Init(&hrtc);

  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.SubSeconds = 0;
  sTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BIN);

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 0;
  HAL_RTC_SetDate(&hrtc, &sDate, FORMAT_BIN);

    /**Enable the Alarm A 
    */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  HAL_RTC_SetAlarm(&hrtc, &sAlarm, FORMAT_BIN);

}

/* SDIO init function */
void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;

}

/* USART3 init function */
void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);

}

/* USB_OTG_FS init function */
void MX_USB_OTG_FS_PCD_Init(void)
{

  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 7;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.ep0_mps = DEP0CTL_MPS_64;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  HAL_PCD_Init(&hpcd_USB_OTG_FS);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PA5   ------> SPI1_SCK
     PA6   ------> SPI1_MISO
     PA7   ------> SPI1_MOSI
     PB10   ------> I2S2_CK
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOE_CLK_ENABLE();
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PE2 PE3 PE4 PE5 
                           PE6 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 
                           PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15 
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 
                           PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


volatile short startConversion = 0;
volatile short startWrite = 0;

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  /*##-5- Write data to the text file ################################*/
  res = f_write(&MyFile, ADCConvValues, DATA_SIZE, (void *)&byteswritten);
  if((byteswritten == 0) || (res != FR_OK))
  {
    res = f_write(&MyFile, ADCConvValues, DATA_SIZE, (void *)&byteswritten);
    if((byteswritten == 0) || (res != FR_OK))
    {
      Error_HandlerSD();
    }
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /*##-5- Write data to the text file ################################*/
  res = f_write(&MyFile, (const char * )(&ADCConvValues[DATA_SIZE / 2]), DATA_SIZE, (void *)&byteswritten);
  countData++;
    if((byteswritten == 0) || (res != FR_OK))
    {
      /* 'Data.TXT' file Write or EOF Error */
      Error_HandlerSD();
    }
    if (countData == 16)
    {
      countData = 0;
      if((res = f_sync (&MyFile)) != FR_OK)
      {
          Error_HandlerSync();
      }
      
      HAL_RTC_GetTime(&hrtc, &sTimeNow, RTC_FORMAT_BIN);
      HAL_RTC_GetDate(&hrtc, &sDateNow, RTC_FORMAT_BIN);
      char timeOfFile[] = {(sTimeNow.Hours+48),':',((sTimeNow.Minutes/10)+48),((sTimeNow.Minutes%10)+48),':',((sTimeNow.Seconds/10)+48),((sTimeNow.Seconds%10)+48),'\n'};
      res = f_write(&MyFile_time, timeOfFile, sizeof(timeOfFile), (void *)&byteswritten);
      if((byteswritten == 0) || (res != FR_OK))
      {
        res = f_write(&MyFile, ADCConvValues, DATA_SIZE, (void *)&byteswritten);
        if((byteswritten == 0) || (res != FR_OK))
        {
          Error_HandlerSD();
        }
      }
      if((res = f_sync (&MyFile_time)) != FR_OK)
      {
          Error_HandlerSync();
      }
    }
    
  if (state == RECORD_READY)
  {
    state = IDLE;
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_Delay(240);
    file_writing = 0;
  }  
}


char string[12] = "Flyability\r\n";

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

  switch (aRxBuffer[0])
  {
  case START:
    state = INIT_RECORD;
    break;
  case STOP:
    state = RECORD_READY;
    break; 
  case RESET_POWER:
    NVIC_SystemReset();
    break; 
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
