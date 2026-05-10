/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <ssd1306.h>
#include <fonts.h>
#include <SX1278.h>
#include <time.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RTC_Addr 0x68 << 1 

#define RTC_Year 0x06
#define RTC_Month 0x05
#define RTC_Day 0x04
#define RTC_Hour 0x02
#define RTC_Minute 0x01
#define RTC_Second 0x00 

#define RTC_MAGIC_REG  0x07   // DS3231 spare register (alarm/control area)
#define RTC_MAGIC_VAL  0xAB   // arbitrary flag value

#define DO_1_Addr 0x61 << 1
#define RTD_1_Addr 0x66 << 1
#define DO_2_Addr 0x62 << 1
#define RTD_2_Addr 0x67 << 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */
I2C_HandleTypeDef *Atlas_I2C = &hi2c3;
I2C_HandleTypeDef *Oled_I2C = &hi2c1;
I2C_HandleTypeDef *RTC_I2C = &hi2c2;

SPI_HandleTypeDef *MicroSD_SPI = &hspi1;
SPI_HandleTypeDef *Lora_SPI = &hspi2;


TIM_HandleTypeDef *LED_Tim = &htim11;
TIM_HandleTypeDef *Live_Tim = &htim10;
TIM_HandleTypeDef *Measure_Tim = &htim9;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM11_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM9_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

// ===============================
// Timer Interrupt Handler
// ===============================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

// ===============================
// Lora Functions
// ===============================
uint8_t LoRa_Init();
uint8_t LoRa_Detect(SX1278_t *module);
uint8_t LoRa_TX(char message[50]);
uint8_t LoRa_TX_Continuous(char message[50]);
static void LoRa_ArmRX();
void LoRa_RX();
// ===============================
// RTC Functions
// ===============================
static uint8_t RTC_Detect();
static uint8_t bcdtodec(const uint8_t val);
static uint8_t dectobcd(const uint8_t val);
static uint32_t RTC_Get_Time(I2C_HandleTypeDef *hi2c);
static void RTC_Set_Time_Once(I2C_HandleTypeDef *hi2c);
static void RTC_Set_Time(I2C_HandleTypeDef *hi2c,char *Time);
static uint32_t get_unix_timestamp(uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec);

// ===============================
// OLED Functions
// ===============================
static void write_OLED(uint8_t pos_x, uint8_t pos_y, char text[18]);

// ===============================
// SD Card Functions
// ===============================
static uint8_t SD_Detect();
static uint8_t SD_Write();
static uint8_t SD_Read();

// ===============================
// Atlas scientific Functions
// ===============================
static uint8_t Atlas_Detect(uint16_t device_addr);
static void Atlas_Wake(uint16_t device_addr);
static void Atlas_Sleep(uint16_t device_addr);
static uint8_t Atlas_Read_Val(char *Output_buffer, uint16_t device_addr);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ===============================
// Lora Variables
// ===============================
SX1278_hw_t SX1278_hw;
SX1278_t SX1278;

char LoRa_RX_Buffer[50];
uint8_t LoRa_RX_Length;

char LoRa_TX_Buffer[100];
uint8_t LoRa_TX_Buffer_Length;

//uint8_t master;
//uint8_t ret;

//uint8_t message;
//uint8_t message_length;


// ===============================
// Atlas Variables
// ===============================
char Atlas_Buffer_1[17];
char Atlas_Buffer_2[17];
char Atlas_Buffer_3[17];
char Atlas_Buffer_4[17];

// ===============================
// General Variables
// ===============================
//uint8_t count = 0;
char mode_buffer[3];
uint8_t Mode_Changed = 0;
uint8_t Mode = 0;
uint8_t Buoy_ID = 0;
char connected_buffer[20];
static char SD_filename[40] = "";  // max 8.3 = "YYMMDD.CSV\0" = 11 chars

uint8_t PB1_Flag = 0;
uint8_t PB2_Flag = 0;
uint8_t PB3_Flag = 0;

// ====================================================================================================
// Flags
// ====================================================================================================

// ===============================
// Lora Flags
// ===============================
uint8_t LoRa_Connected = 0;
uint8_t LoRa_RX_Flag = 0;
uint8_t LoRa_RX_Mode = 0;
uint8_t LoRa_RX_Counter = 0;

// ===============================
// OLED Flags
// ===============================
uint8_t OLED_Connected = 0;

// ===============================
// RTC Flags
// ===============================
uint8_t RTC_Connected = 0;
uint8_t RTC_Update_Oled = 0;

// ===============================
// SD Flags
// ===============================
uint8_t SD_Connected = 0;

// ===============================
// Atlas Flags
// ===============================
uint8_t Atlas1_Connected = 0;
uint8_t Atlas2_Connected = 0;
uint8_t Atlas3_Connected = 0;
uint8_t Atlas4_Connected = 0;

uint8_t Atlas_Send_live = 0;
uint8_t Atlas_measure = 0;

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
  MX_I2C1_Init();
  MX_TIM11_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_SPI2_Init();
  MX_TIM10_Init();
  MX_TIM9_Init();
  MX_IWDG_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(1500);

  //initialize LoRa module
  SX1278_hw.dio0.port = Lora_IRQ_GPIO_Port;
  SX1278_hw.dio0.pin = Lora_IRQ_Pin;
  SX1278_hw.nss.port = Lora_CS_GPIO_Port;
  SX1278_hw.nss.pin = Lora_CS_Pin;
  SX1278_hw.reset.port = Lora_Reset_GPIO_Port;
  SX1278_hw.reset.pin = Lora_Reset_Pin;
  SX1278_hw.spi = Lora_SPI;

  SX1278.hw = &SX1278_hw;
  LoRa_Init();
  
  //Initilise OLED Screen
  OLED_Connected = !ssd1306_Init(Oled_I2C); // Initilise the Oled module
 
  //Initilise the RTC One time when the system is flashed with firmware
  RTC_Connected = RTC_Detect();
  //if (RTC_Connected) RTC_Set_Time_Once(RTC_I2C);

  

  Atlas1_Connected = Atlas_Detect(DO_1_Addr);
  Atlas2_Connected = Atlas_Detect(RTD_1_Addr);
  Atlas3_Connected = Atlas_Detect(DO_2_Addr);
  Atlas4_Connected = Atlas_Detect(RTD_2_Addr);

  //SD CARD CODE 

  
  SD_Detect();

  // if(SD_Connected)
  // {

  //   FATFS FatFs;
  //   FIL fil;
  //   FRESULT fres;
  //   BYTE readBuf[30];

  
  //   fres = f_mount(&FatFs, "", 1); //1=mount now  //Open the file system
  //   if (fres != FR_OK) {
  //     while(1);
  //   }

  //   fres = f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);

  //   strncpy((char*)readBuf,"a new file is made!",19);
  //   UINT bytesWrote;
  //   fres = f_write(&fil,readBuf,19,&bytesWrote);
    

  //   f_close(&fil);
    
  // }
  

  HAL_TIM_Base_Start_IT(LED_Tim); //Start the timer for the LED with interrupt mode
  HAL_TIM_Base_Start_IT(Live_Tim); 
  HAL_TIM_Base_Start_IT(Measure_Tim);

  uint32_t curr_time;
  //uint16_t Counter = 0;
  char time_buffer[18];

  HAL_Delay(1000);
  HAL_IWDG_Refresh(&hiwdg);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    SD_Detect();

// ====================================================================================================
// Update RTC OLED Display
// ====================================================================================================
    if (RTC_Connected&&RTC_Update_Oled)
    {
      curr_time = RTC_Get_Time(RTC_I2C);

      struct tm *tm_info;
      time_t raw_time = (time_t)curr_time;

      // 1. Break the timestamp down into the tm struct
      tm_info = localtime(&raw_time);
      strftime(time_buffer, sizeof(time_buffer), "%d/%m/%y %H:%M:%S", tm_info);
    }

    if (OLED_Connected&&RTC_Update_Oled)
    {
      write_OLED(0,0,time_buffer);

      snprintf(connected_buffer,sizeof(connected_buffer), "L:%u,R:%u,A:%u%u%u%u,S:%u",LoRa_Connected,RTC_Connected,Atlas1_Connected,Atlas2_Connected,Atlas3_Connected,Atlas4_Connected,SD_Connected);
      write_OLED(0,1,connected_buffer);

      write_OLED(0,2,Atlas_Buffer_1);
      write_OLED(7,2,Atlas_Buffer_2);
      write_OLED(0,3,Atlas_Buffer_3);
      write_OLED(7,3,Atlas_Buffer_4);


      RTC_Update_Oled = 0;

      if(PB1_Flag)
      {
        write_OLED(6,5,"1");
        PB1_Flag = 0;
      }
      else if(PB2_Flag)
      {
        write_OLED(6,5,"2");
        PB3_Flag = 0;
      }
      else if(PB3_Flag)
      {
        write_OLED(6,5,"3");
        PB3_Flag = 0;
      }
    }


// ====================================================================================================
// Display the current mode on the OLED if present.
// ====================================================================================================
    if (OLED_Connected)
    {
      switch (Mode) {
        case 1:
          snprintf(mode_buffer,sizeof(mode_buffer),"MR");
          Mode = 0;
          Mode_Changed = 1;
          break;
        case 2:
          snprintf(mode_buffer,sizeof(mode_buffer),"TX");
          Mode = 0;
          Mode_Changed = 1;
          break;
        case 3:
          snprintf(mode_buffer,sizeof(mode_buffer),"ST");
          Mode = 0;
          Mode_Changed = 1;
          break;
        default:
          snprintf(mode_buffer,sizeof(mode_buffer),"NA");
          Mode = 0;
      }

      if (Mode_Changed)
      {
        write_OLED(15,5,mode_buffer);
        Mode_Changed = 0;
      }
    }




    if(LoRa_RX_Flag&&OLED_Connected)
    {
      //clear the OLED screen to display the new RX
      write_OLED(0,4,"                  ");
      write_OLED(0,4,LoRa_RX_Buffer);
    }

    if(LoRa_RX_Flag)
    {
      //Check the rx buffer to see if command sent, and respond accordingly
      if(strcmp(LoRa_RX_Buffer,"CMD:BATTERY")==0)
      {
        LoRa_TX("TYPE:3,BUOY:0,BAT:3.99");
        Mode = 2;
      }
      else if (strcmp(LoRa_RX_Buffer,"CMD:DATADUMP")==0)
      {
        LoRa_TX_Continuous("TYPE:2,BUOY:0,CHAMBER:0,DO:9999,RTD:9999,TS:3999_01_01_12_30_10");
        LoRa_TX_Continuous("TYPE:2,BUOY:0,CHAMBER:1,DO:8888,RTD:8888,TS:3999_01_01_12_30_10");
        LoRa_TX("TYPE:4,BUOY:0,COUNT:2");
        Mode = 2;
      }
      else if (strncmp(LoRa_RX_Buffer,"CMD:SYNC",8)==0)
      {
        char time[19];
        memcpy(time,LoRa_RX_Buffer+12,19);
        RTC_Set_Time(RTC_I2C,time);
        LoRa_TX("Sync_ACK");
        Mode = 2;
      }
      LoRa_RX_Flag = 0;

    }

// ====================================================================================================
// Read the atlas sensors and send live readings
// ====================================================================================================
    if(Atlas_measure)
    {  
      if (Atlas1_Connected) Atlas_Wake(DO_1_Addr);
      if (Atlas2_Connected) Atlas_Wake(RTD_1_Addr);
      if (Atlas3_Connected) Atlas_Wake(DO_2_Addr);
      if (Atlas4_Connected) Atlas_Wake(RTD_2_Addr);

      HAL_Delay(600);

      if (Atlas1_Connected) Atlas_Read_Val(Atlas_Buffer_1,DO_1_Addr);
      if (Atlas2_Connected) Atlas_Read_Val(Atlas_Buffer_2,RTD_1_Addr);
      if (Atlas3_Connected) Atlas_Read_Val(Atlas_Buffer_3,DO_2_Addr);
      if (Atlas4_Connected) Atlas_Read_Val(Atlas_Buffer_4,RTD_2_Addr);

      if (Atlas1_Connected) Atlas_Sleep(DO_1_Addr);
      if (Atlas2_Connected) Atlas_Sleep(RTD_1_Addr);
      if (Atlas3_Connected) Atlas_Sleep(DO_2_Addr);
      if (Atlas4_Connected) Atlas_Sleep(RTD_2_Addr);

      //HAL_Delay(500);
      if (SD_Connected&&RTC_Connected)
      {
        HAL_IWDG_Refresh(&hiwdg);
        curr_time = RTC_Get_Time(RTC_I2C);
        if (SD_Write(curr_time))
        {
          if (OLED_Connected) write_OLED(0,5,"ST ");
        }
        else
        {
          if (OLED_Connected) write_OLED(0,5,"STF");
        }
      }

      if (OLED_Connected&&!SD_Connected) write_OLED(0,5,"STF");


      Atlas_measure = 0;
      Mode = 1;
    }
    
    if (Atlas_Send_live)
    {
      //  // 1. Get and convert the timestamp
      // curr_time = RTC_Get_Time(RTC_I2C);
      // time_t raw_time = (time_t)curr_time;
      // struct tm *tm_info = localtime(&raw_time);

      // // 2. Format the timestamp into its own time_buffer using strftime
      // char ts_buffer[20];
      // strftime(ts_buffer, sizeof(ts_buffer), "%y_%m_%d_%H_%M_%S", tm_info);

      // 3. Build and send the header packet with the timestamp
      snprintf(LoRa_TX_Buffer, sizeof(LoRa_TX_Buffer), "TYPE:1,BUOY:0,CHAMBER:0,DO:%s,RTD:%s", Atlas_Buffer_1,Atlas_Buffer_2);
      LoRa_TX_Continuous(LoRa_TX_Buffer);

      snprintf(LoRa_TX_Buffer, sizeof(LoRa_TX_Buffer), "TYPE:1,BUOY:0,CHAMBER:1,DO:%s,RTD:%s", Atlas_Buffer_3,Atlas_Buffer_4); // Convert object to C-string
      LoRa_TX(LoRa_TX_Buffer);
      Mode = 2;

      Atlas_Send_live=0;
    }

    //HAL_Delay(5000);

    HAL_IWDG_Refresh(&hiwdg);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 24999;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 4999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 24999;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 29999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 24999;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 1000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */


  /* USER CODE END TIM11_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_Board_GPIO_Port, LED_Board_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Lora_CS_GPIO_Port, Lora_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Lora_Reset_GPIO_Port, Lora_Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Board_Pin */
  GPIO_InitStruct.Pin = LED_Board_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_Board_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB_1_Pin PB_2_Pin */
  GPIO_InitStruct.Pin = PB_1_Pin|PB_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB_3_Pin */
  GPIO_InitStruct.Pin = PB_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PB_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CD_Pin */
  GPIO_InitStruct.Pin = SD_CD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SD_CD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Lora_CS_Pin */
  GPIO_InitStruct.Pin = Lora_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Lora_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Lora_IRQ_Pin */
  GPIO_InitStruct.Pin = Lora_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Lora_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Lora_Reset_Pin */
  GPIO_InitStruct.Pin = Lora_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Lora_Reset_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// ===============================================================================================
// Timer Interrupt Handler
// ===============================================================================================
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM11)
  {
    HAL_GPIO_TogglePin(LED_Board_GPIO_Port,LED_Board_Pin);
    RTC_Update_Oled = 1;
  }
  if (htim->Instance == TIM10)
  {
    Atlas_Send_live = 1;
  }
  if (htim->Instance == TIM9)
  {
    Atlas_measure = 1;
  }
}

// ===============================================================================================
// GPIO Interrupt Handler
// ===============================================================================================

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == Lora_IRQ_Pin) // Check which pin triggered the interrupt
  {
    LoRa_RX();
  }
  if(GPIO_Pin == PB_1_Pin)
  {
    static uint32_t last_press_time1 = 0;
    uint32_t current_time1 = HAL_GetTick(); 

    if ((current_time1 - last_press_time1) > 500) // 200ms debounce delay
    {
      HAL_GPIO_TogglePin(LED_Board_GPIO_Port,LED_Board_Pin);
      last_press_time1 = current_time1;
      PB1_Flag = 1;
    }
  }
  if(GPIO_Pin == PB_2_Pin)
  {
    static uint32_t last_press_time2 = 0;
    uint32_t current_time2 = HAL_GetTick(); 

    if ((current_time2 - last_press_time2) > 500) // 200ms debounce delay
    {
      HAL_GPIO_TogglePin(LED_Board_GPIO_Port,LED_Board_Pin);
      last_press_time2 = current_time2;
      PB2_Flag = 1;
    }
  }
  if(GPIO_Pin == PB_3_Pin)
  {
    static uint32_t last_press_time3 = 0;
    uint32_t current_time3 = HAL_GetTick(); 

    if ((current_time3 - last_press_time3) > 500) // 200ms debounce delay
    {
      HAL_GPIO_TogglePin(LED_Board_GPIO_Port,LED_Board_Pin);
      last_press_time3 = current_time3;
      PB3_Flag = 1;
    }
  }
}

// ===============================================================================================
// Lora Functions
// ===============================================================================================
uint8_t LoRa_Init()
{

  SX1278_hw_init(&SX1278_hw);

  // Step 2: perform a hard reset and wait for module to boot
  SX1278_hw_Reset(&SX1278_hw);   // pulls reset low then high, waits 100ms internally
  HAL_Delay(100);         
  if (LoRa_Detect(&SX1278))
  {
    SX1278_init(&SX1278, 433175000, SX1278_POWER_17DBM, SX1278_LORA_SF_10,
    SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 100);

    LoRa_ArmRX();

    LoRa_Connected = 1;
    return 1;
  }
  else
  {
    LoRa_Connected = 0;
    return 0;
  }
}

uint8_t LoRa_Detect(SX1278_t *module)
{
  uint8_t version = SX1278_SPIRead(module, REG_LR_VERSION);
  return (version == 0x12) ? 1 : 0;
}


uint8_t LoRa_TX(char *message)
{
  uint8_t msg_len = strlen(message);
  if (msg_len == 0 || msg_len > 100) return 0;

  if (SX1278_LoRaEntryTx(&SX1278, msg_len, 2000))
  {
    uint8_t result =  SX1278_LoRaTxPacket(&SX1278, (uint8_t*)message, msg_len, 2000);
    LoRa_ArmRX();
    return result;
  }
  return 0;
}

uint8_t LoRa_TX_Continuous(char *message)
{
  uint8_t msg_len = strlen(message);
  if (msg_len == 0 || msg_len > 100) return 0;

  if (SX1278_LoRaEntryTx(&SX1278, msg_len, 2000))
  {
    uint8_t result =  SX1278_LoRaTxPacket(&SX1278, (uint8_t*)message, msg_len, 2000);
    //LoRa_ArmRX();
    return result;
  }
  return 0;
}

// uint8_t LoRa_TX(char message[50])
// {
//   //message_length = sprintf(Lora_buffer, "Hello May, Chamber ID:0001, DO: 21.5, RTD: 31.0, DO: 22.5, RTD: 22.0. %d", message);
//   message_length = strlen(message);

//   if (message_length == 0 || message_length > 50)
//     return 0;  // guard against empty or oversized messages

//   if (LoRa_Detect(&SX1278))
//   {
//     ret = SX1278_LoRaEntryTx(&SX1278, message_length, 2000);
//     if (ret)
//     {
//       ret = SX1278_LoRaTxPacket(&SX1278, (uint8_t*) message, message_length, 2000);

//       SX1278_LoRaEntryRx(&SX1278, 100, 2000);

//       if(ret) return 1;
//       else return 0;
//     }
//     else return 0;
//   }
//   else
//   {
//     LoRa_Connected = 0;
//     return 0;
//   }
// }

// uint8_t LoRa_TX(char message[50])
// {
//   uint8_t msg_len = strlen(message);

//   if (msg_len == 0 || msg_len > 50)
//     return 0;

//   // Switch directly to TX mode without calling SX1278_LoRaEntryTx
//   // which calls SX1278_config (15ms sleep + full reconfigure) every time
//   SX1278_SPIWrite(&SX1278, REG_LR_PADAC, 0x87);              // TX power
//   SX1278_SPIWrite(&SX1278, LR_RegHopPeriod, 0x00);           // no FHSS
//   SX1278_SPIWrite(&SX1278, REG_LR_DIOMAPPING1, 0x41);        // DIO0 = TxDone
//   SX1278_clearLoRaIrq(&SX1278);
//   SX1278_SPIWrite(&SX1278, LR_RegIrqFlagsMask, 0xF7);        // unmask TxDone
//   SX1278_SPIWrite(&SX1278, LR_RegPayloadLength, msg_len);

//   // Point FIFO to TX base
//   uint8_t addr = SX1278_SPIRead(&SX1278, LR_RegFifoTxBaseAddr);
//   SX1278_SPIWrite(&SX1278, LR_RegFifoAddrPtr, addr);

//   // Write payload to FIFO
//   SX1278_SPIBurstWrite(&SX1278, 0x00, (uint8_t*)message, msg_len);

//   // Fire TX
//   SX1278_SPIWrite(&SX1278, LR_RegOpMode, 0x8b);

//   // Wait for TxDone on DIO0 with timeout
//   uint32_t timeout = 500;
//   while (!SX1278_hw_GetDIO0(SX1278.hw))
//   {
//     if (--timeout == 0)
//     {
//       SX1278_hw_Reset(SX1278.hw);
//       // Full reinit needed after reset
//       SX1278_init(&SX1278, 433175000, SX1278_POWER_11DBM, SX1278_LORA_SF_7,
//           SX1278_LORA_BW_125KHZ, SX1278_LORA_CR_4_5, SX1278_LORA_CRC_EN, 100);
//       LoRa_ArmRX();
//       return 0;
//     }
//     SX1278_hw_DelayMs(1);
//   }

//   SX1278_clearLoRaIrq(&SX1278);

//   // Re-arm RX directly — no reconfigure
//   LoRa_ArmRX();

//   return 1;
// }

// static void LoRa_ArmRX()
// {
//     SX1278_SPIWrite(&SX1278, REG_LR_PADAC, 0x84);
//     SX1278_SPIWrite(&SX1278, LR_RegHopPeriod, 0xFF);
//     SX1278_SPIWrite(&SX1278, REG_LR_DIOMAPPING1, 0x01);        // DIO0 = RxDone
//     SX1278_SPIWrite(&SX1278, LR_RegIrqFlagsMask, 0x3F);        // unmask RxDone
//     SX1278_clearLoRaIrq(&SX1278);
//     SX1278_SPIWrite(&SX1278, LR_RegPayloadLength, 100);
//     uint8_t addr = SX1278_SPIRead(&SX1278, LR_RegFifoRxBaseAddr);
//     SX1278_SPIWrite(&SX1278, LR_RegFifoAddrPtr, addr);
//     SX1278_SPIWrite(&SX1278, LR_RegOpMode, 0x8d);              // continuous RX
//     SX1278.status = RX;
// }

static void LoRa_ArmRX()
{
  SX1278_LoRaEntryRx(&SX1278, 100, 2000);
}

// void LoRa_RX()
// {
//   // Read packet out of the FIFO immediately
//   LoRa_RX_Length = SX1278_LoRaRxPacket(&SX1278);

//   if (LoRa_RX_Length > 0)
//   {

//     // Copy out of the module's internal time_buffer and null-terminate
//     if (LoRa_RX_Length >= sizeof(LoRa_RX_Buffer)) LoRa_RX_Length = sizeof(LoRa_RX_Buffer) - 1;
//     memcpy(LoRa_RX_Buffer, SX1278.rxBuffer, LoRa_RX_Length);
//     LoRa_RX_Buffer[LoRa_RX_Length] = '\0';  // always null-terminate
//     LoRa_RX_Flag = 1;  // signal main loop to process it
//     LoRa_RX_Counter++;
//   }

//   // Re-arm the receiver for the next packet
//   // Can't call SX1278_LoRaEntryRx here (it uses HAL_Delay)
//   // Instead just reset the FIFO pointer and clear IRQ
//   uint8_t addr = SX1278_SPIRead(&SX1278, LR_RegFifoRxBaseAddr);
//   SX1278_SPIWrite(&SX1278, LR_RegFifoAddrPtr, addr);
//   SX1278_clearLoRaIrq(&SX1278);

//   LoRa_ArmRX();
// }

void LoRa_RX()
{
    uint8_t len = SX1278_LoRaRxPacket(&SX1278);
    if (len > 0)
    {
        if (len >= sizeof(LoRa_RX_Buffer))
            len = sizeof(LoRa_RX_Buffer) - 1;
        memcpy(LoRa_RX_Buffer, SX1278.rxBuffer, len);
        LoRa_RX_Buffer[len] = '\0';
        LoRa_RX_Length = len;
        LoRa_RX_Flag = 1;
        //LoRa_RX_Counter++;
    }
    SX1278_clearLoRaIrq(&SX1278);
}

// ===============================================================================================
// RTC Funtions
// ===============================================================================================
//function to convert binary coded decimal to normal decimal value.
static uint8_t RTC_Detect()
{
  if (HAL_I2C_IsDeviceReady(RTC_I2C, RTC_Addr, 3, 100) == HAL_OK) {
    return 1;
  } else {
    return 0;
  }
}

static uint8_t bcdtodec(const uint8_t val)
{
  return ((val / 16 * 10) + (val % 16));
}

static uint8_t decimalbcd(uint8_t val)
{
    return ((val / 10) << 4) | (val % 10);
}

static uint32_t RTC_Get_Time(I2C_HandleTypeDef *hi2c)
{
    uint8_t Year, Month, Day, Hour, Minute, Second;

    HAL_I2C_Mem_Read(hi2c, RTC_Addr, RTC_Year,   1, &Year,   1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(hi2c, RTC_Addr, RTC_Month,  1, &Month,  1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(hi2c, RTC_Addr, RTC_Day,    1, &Day,    1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(hi2c, RTC_Addr, RTC_Hour,   1, &Hour,   1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(hi2c, RTC_Addr, RTC_Minute, 1, &Minute, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(hi2c, RTC_Addr, RTC_Second, 1, &Second, 1, HAL_MAX_DELAY);

    // Mask status bits BEFORE BCD conversion
    Year   = bcdtodec(Year);
    Month  = bcdtodec(Month  & 0x1F); // mask century bit
    Day    = bcdtodec(Day    & 0x3F);
    Hour   = bcdtodec(Hour   & 0x3F); // mask 12/24hr bit
    Minute = bcdtodec(Minute & 0x7F);
    Second = bcdtodec(Second & 0x7F);

    return get_unix_timestamp(2000 + Year, Month, Day, Hour, Minute, Second);
}

static void RTC_Set_Time_Once(I2C_HandleTypeDef *hi2c)
{
    // Check if time has already been set
    uint8_t magic = 0;
    HAL_I2C_Mem_Read(hi2c, RTC_Addr, RTC_MAGIC_REG, 1, &magic, 1, HAL_MAX_DELAY);
    
    if (magic == RTC_MAGIC_VAL)
        return; // Already set, skip

    // Parse compile-time strings into numbers
    // __DATE__ is "Mon DD YYYY" e.g. "Apr 22 2026"
    // __TIME__ is "HH:MM:SS"   e.g. "14:30:00"
    char date[] = __DATE__;
    char time[] = __TIME__;

    uint8_t day    = ((date[4] == ' ') ? 0 : (date[4] - '0')) * 10 + (date[5] - '0');
    uint16_t year  = (date[7]-'0')*1000 + (date[8]-'0')*100 + (date[9]-'0')*10 + (date[10]-'0');
    uint8_t hour   = (time[0]-'0')*10 + (time[1]-'0');
    uint8_t minute = (time[3]-'0')*10 + (time[4]-'0');
    uint8_t second = (time[6]-'0')*10 + (time[7]-'0');

    // Parse month string
    const char *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    uint8_t month = 1;
    for (int i = 0; i < 12; i++) {
        if (strncmp(&months[i*3], date, 3) == 0) {
            month = i + 1;
            break;
        }
    }

    // Write to DS3231 registers in BCD
    uint8_t reg_val;

    reg_val = decimalbcd(second);
    HAL_I2C_Mem_Write(hi2c, RTC_Addr, RTC_Second, 1, &reg_val, 1, HAL_MAX_DELAY);

    reg_val = decimalbcd(minute);
    HAL_I2C_Mem_Write(hi2c, RTC_Addr, RTC_Minute, 1, &reg_val, 1, HAL_MAX_DELAY);

    reg_val = decimalbcd(hour);
    HAL_I2C_Mem_Write(hi2c, RTC_Addr, RTC_Hour, 1, &reg_val, 1, HAL_MAX_DELAY);

    reg_val = decimalbcd(day);
    HAL_I2C_Mem_Write(hi2c, RTC_Addr, RTC_Day, 1, &reg_val, 1, HAL_MAX_DELAY);

    reg_val = decimalbcd(month);
    HAL_I2C_Mem_Write(hi2c, RTC_Addr, RTC_Month, 1, &reg_val, 1, HAL_MAX_DELAY);

    reg_val = decimalbcd(year % 100); // DS3231 only stores last 2 digits
    HAL_I2C_Mem_Write(hi2c, RTC_Addr, RTC_Year, 1, &reg_val, 1, HAL_MAX_DELAY);

    // Write magic flag so we never set time again
    reg_val = RTC_MAGIC_VAL;
    HAL_I2C_Mem_Write(hi2c, RTC_Addr, RTC_MAGIC_REG, 1, &reg_val, 1, HAL_MAX_DELAY);
}

static void RTC_Set_Time(I2C_HandleTypeDef *hi2c,char *Time)
{
    // __DATE__ is "YYYY:MM:DD:HH:MM:SS"

    uint16_t year = (Time[0]-'0')*1000 + (Time[1]-'0')*100 + (Time[2]-'0')*10 + (Time[3]-'0');
    uint8_t month = (Time[5]-'0')*10 + (Time[6]-'0');
    uint8_t day = (Time[8]-'0')*10 + (Time[9]-'0');
    uint8_t hour = (Time[11]-'0')*10 + (Time[12]-'0');
    uint8_t minute = (Time[14]-'0')*10 + (Time[15]-'0');
    uint8_t second = (Time[17]-'0')*10 + (Time[18]-'0');

    // Write to DS3231 registers in BCD
    uint8_t reg_val;

    reg_val = decimalbcd(second);
    HAL_I2C_Mem_Write(hi2c, RTC_Addr, RTC_Second, 1, &reg_val, 1, HAL_MAX_DELAY);

    reg_val = decimalbcd(minute);
    HAL_I2C_Mem_Write(hi2c, RTC_Addr, RTC_Minute, 1, &reg_val, 1, HAL_MAX_DELAY);

    reg_val = decimalbcd(hour);
    HAL_I2C_Mem_Write(hi2c, RTC_Addr, RTC_Hour, 1, &reg_val, 1, HAL_MAX_DELAY);

    reg_val = decimalbcd(day);
    HAL_I2C_Mem_Write(hi2c, RTC_Addr, RTC_Day, 1, &reg_val, 1, HAL_MAX_DELAY);

    reg_val = decimalbcd(month);
    HAL_I2C_Mem_Write(hi2c, RTC_Addr, RTC_Month, 1, &reg_val, 1, HAL_MAX_DELAY);

    reg_val = decimalbcd(year % 100); // DS3231 only stores last 2 digits
    HAL_I2C_Mem_Write(hi2c, RTC_Addr, RTC_Year, 1, &reg_val, 1, HAL_MAX_DELAY);

    // Write magic flag so we never set time again
    reg_val = RTC_MAGIC_VAL;
    HAL_I2C_Mem_Write(hi2c, RTC_Addr, RTC_MAGIC_REG, 1, &reg_val, 1, HAL_MAX_DELAY);
}

static uint32_t get_unix_timestamp(uint16_t year, uint8_t month, uint8_t day, 
                           uint8_t hour, uint8_t min, uint8_t sec) 
{
    struct tm t;
    time_t t_of_day;

    // Standard C struct tm requirements:
    t.tm_year = year - 1900;  // Year since 1900
    t.tm_mon = month - 1;     // Month, 0 - 11
    t.tm_mday = day;          // Day of the month
    t.tm_hour = hour;
    t.tm_min = min;
    t.tm_sec = sec;
    t.tm_isdst = -1;          // Is Daylight Savings Time on? -1 = let the library decide

    t_of_day = mktime(&t);

    return (uint32_t)t_of_day;
}

// ===============================================================================================
// OLED Functions
// ===============================================================================================
/**
  * @brief OLED Write Function
  * @param uint8_t X_Position
  * @param uint8_t Y_Position
  * @param char text[18]
  * @retval None
  */
static void write_OLED(uint8_t pos_x, uint8_t pos_y, char text[18])
{
  ssd1306_SetCursor(pos_x*7, pos_y*10);
  ssd1306_WriteString(text, Font_7x10, White); //Write some text to the Oled module

  ssd1306_UpdateScreen(Oled_I2C);
}

// ===============================================================================================
// SD Functions
// ===============================================================================================

static uint8_t SD_Detect()
{
  SD_Connected = HAL_GPIO_ReadPin(SD_CD_GPIO_Port,SD_CD_Pin);
  return SD_Connected;
}

static uint8_t SD_Write(uint32_t timestamp)
{
    SD_Detect();
    if (!SD_Connected) return 0;

    FATFS   FatFs;
    FIL     fil;
    FRESULT fres;
    UINT    bytesWrote;
    char    row[60];

    time_t raw = (time_t)timestamp;
    struct tm *tm_info = localtime(&raw);

    // Generate the filename on the very first write of this session


    strftime(SD_filename, sizeof(SD_filename), "%y-%m-%d-%H-%M-%S-READING.CSV", tm_info);

    //if (OLED_Connected) write_OLED(0,5,"flnnm");

    // Human-readable timestamp for inside the CSV rows
    char ts[20];
    strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", tm_info);

    //if (OLED_Connected) write_OLED(0,5,"timok ");

    fres = f_mount(&FatFs, "", 1);
    if (fres != FR_OK) return 0;
    //if (OLED_Connected) write_OLED(0,5,"mntok");

    fres = f_open(&fil, SD_filename, FA_WRITE | FA_CREATE_ALWAYS);
    if (fres != FR_OK) { f_mount(NULL, "", 0); return 0; }

    //if (OLED_Connected) write_OLED(0,5,"opnok");

    if (f_size(&fil) == 0) {
        char *header = "Date,Sensor,Value\n";
        f_write(&fil, header, strlen(header), &bytesWrote);
    }

    //f_lseek(&fil, f_size(&fil));

    snprintf(row, sizeof(row), "%s,DO_1,%s\n",  ts, Atlas_Buffer_1);
    f_write(&fil, row, strlen(row), &bytesWrote);

    snprintf(row, sizeof(row), "%s,RTD_1,%s\n", ts, Atlas_Buffer_2);
    f_write(&fil, row, strlen(row), &bytesWrote);

    snprintf(row, sizeof(row), "%s,DO_2,%s\n",  ts, Atlas_Buffer_3);
    f_write(&fil, row, strlen(row), &bytesWrote);

    snprintf(row, sizeof(row), "%s,RTD_2,%s\n", ts, Atlas_Buffer_4);
    f_write(&fil, row, strlen(row), &bytesWrote);

    f_close(&fil);
    //f_unmount("");
    f_mount(NULL, "", 0);

    //Mode = 3;
    //if (OLED_Connected) write_OLED(0,5,"ST");
    return 1;
}

// ===============================================================================================
// Atlas Functions
// ===============================================================================================
static uint8_t Atlas_Detect(uint16_t device_addr)
{
    return (HAL_I2C_IsDeviceReady(Atlas_I2C, device_addr, 1, 10) == HAL_OK) ? 1 : 0;
}

static void Atlas_Sleep(uint16_t device_addr)
{
    char cmd[] = "Sleep";
    HAL_I2C_Master_Transmit(Atlas_I2C, device_addr, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
    // Datasheet explicitly says: do NOT try to read a response after this
}

static void Atlas_Wake(uint16_t device_addr)
{
    // Send "R" — this wakes the device AND queues a reading in one shot.
    // The device needs ~600ms to wake + take the reading before you read back.
    char cmd[] = "R";
    HAL_I2C_Master_Transmit(Atlas_I2C, device_addr, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY);
}

static uint8_t Atlas_Read_Val(char *Output_buffer, uint16_t device_addr)
{
    char tx_cmd[] = "R";
    HAL_I2C_Master_Transmit(Atlas_I2C, device_addr, (uint8_t*)tx_cmd, strlen(tx_cmd), HAL_MAX_DELAY);

    HAL_Delay(600);

    uint8_t raw[17];  // 1 byte response code + up to 16 bytes data
    uint8_t retries = 0;

    do {
        HAL_I2C_Master_Receive(Atlas_I2C, device_addr, raw, sizeof(raw), 100);

        if (raw[0] != 254) break;  // 254 = still processing, anything else = done

        retries++;
        HAL_Delay(20);
    } while (retries < 10);

    // raw[0] is the response code, raw[1...] is the ASCII string
    if (raw[0] == 1) {
        memcpy(Output_buffer, (char*)&raw[1], sizeof(raw) - 1);
        Output_buffer[sizeof(raw) - 1] = '\0';
        return 1;  // success
    }

    return 0;  // error
}

  // Holds the current session's filename, set on first write


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
#ifdef USE_FULL_ASSERT
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
