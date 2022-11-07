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
STM32L4A6 loRa water detection node
PCB: L4A6_WL_LoRa_V3
Version: V1.4
IWDG: 8s
Alarm: 10s
standby mode :6HR
  ******************************************************************************
Update Note:
20210629----V1.1:
Wake-up pin trigger from low level to High level.
IWDG reload delay 50ms at alarm mode.
20211015----V1.2:
main.c, stm32l4xx_it.c : Add EEPROM emulation function.
main.c, stm32l4xx_hal_msp.c : Add PVD function, <2.0V to always standby, delay 50ms enable PVD to wait power on.
main.c, LoRa_App_slave.c : Add low voltage monitor function, <2.8V alarm low battery, <2.5V to always standby.
20211015----V1.3:
Change HCLK to 16MHz, for improve Uart receive speed.
main.c, modify low voltage protection from <2.5V to <2.35V to always standby.
LoRa_App_slave.c, Update to V2.6.
 *Improve Tx receive 1st and 2nd response process, for first connect to UMC LoRa Gateway ADR messenger.
 *Add Voltage Heavy load detect.
20220401----V1.4:
main.c, When active ,PVD save Voltage valve to trigger standby mode after software reset.
main.c, When Voltage < 2.35V, add lora chip sleep function.
LoRa_App_slave.c, Update to V2.7.
 *Change Lora chip sleep time to 24hr,avid wake-up early than mcu standby time.
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//TODO--Wayne20210528
PWRST_t PWRST;
USART_LoRa USARTLoRa;
ADC1_t ADC_1;
char NodeStatus;
//test
uint16_t SendAlarmCount;
uint16_t SendAlarmTMRCount;
//_Bool WaterDetectflag;
_Bool SendAlarmflag;
//TODO for IWDG Freeze By Standby Mode
FLASH_OBProgramInitTypeDef pOBInit;

//TODO:for CubeEEPROM
uint32_t Index = 1;
__IO uint32_t ErasingOnGoing = 0;
uint32_t a_VarDataTab[NB_OF_VARIABLES] = {0};
uint32_t VarValue = 0;
uint8_t itt, itt2, itt3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */
////TODO:for CubeEEPROM--20211005 by Wayne
//static void Error_Handler(void);

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
  //TODO: EEPROM by Wayne 20211005
  EE_Status ee_status = EE_OK;
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_IWDG_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  //TODO: EEPROM by Wayne 20211005
   /* Unlock the Flash Program Erase controller */
   HAL_FLASH_Unlock();

 #if defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined (STM32L4S9xx)
   /* Clear OPTVERR bit and PEMPTY flag if set*/
   if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_OPTVERR) != RESET)
   {
     __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
   }

   if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PEMPTY) != RESET)
   {
     __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PEMPTY);
   }
 #endif /* defined (STM32L4R5xx) || defined (STM32L4R7xx) || defined (STM32L4R9xx) || defined (STM32L4S5xx) || defined (STM32L4S7xx) || defined (STM32L4S9xx) */

 /* Set EEPROM emulation firmware to erase all potentially incompletely erased
      pages if the system came from an asynchronous reset. Conditional erase is
      safe to use if all Flash operations where completed before the system reset */
   //flag test
 //  uint8_t itt, itt2, itt3;
 //  itt=__HAL_PWR_GET_FLAG(PWR_FLAG_SB);
 //  itt2= __HAL_PWR_GET_FLAG(PWR_FLAG_WU);
 //  itt3=__HAL_RTC_WAKEUPTIMER_GET_IT_SOURCE (&hrtc, RTC_FLAG_WUTF);
 //  if(itt == RESET)
   if(__HAL_PWR_GET_FLAG(PWR_FLAG_SB) == RESET)
   {
     /* System reset comes from a power-on reset: Forced Erase */
     /* Initialize EEPROM emulation driver (mandatory) */
     ee_status = EE_Init(EE_FORCED_ERASE);
     if(ee_status != EE_OK) {Error_Handler();}
// 	  __NOP();
   }
   else
   {
 //    /* Clear the Standby flag */
 //    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
 //
 //    /* Check and Clear the Wakeup flag */
 //    if (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF) != RESET)
 //    {
 //      __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);
 //    }
     /* System reset comes from a STANDBY wakeup: Conditional Erase*/
     /* Initialize EEPROM emulation driver (mandatory) */
     ee_status = EE_Init(EE_CONDITIONAL_ERASE);
     if(ee_status != EE_OK) {Error_Handler();}

     /*EEPROM Read*/
     ee_status = EEPROM_Emu_read();
   }

//TODO--Wayne20210531
  RUN_PWR_Mode_Init(&hrtc);
  LoRa_status_init(&huart1);
  //TODO for IWDG Freeze By Standby Mode
  IWDGFreezeBySTDBY();

  /*
  setting Device power mode:
  PWRST.PowerMode =
  			 	 ---RunMode
  			 	 ---StandbyMode
  */
  PWRST.PowerMode =StandbyMode;
  NodeStatus = DataAcquire;
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	switch(NodeStatus)
	  {
	  	case DataAcquire:
	  		//battery light loading voltage
	  		ADC1_Conv(&hadc1);
	  		ADC_1.BatVLightload=ADC_1.BatVTemp;
	  		//ADC LED status
	  		HAL_GPIO_WritePin(ADC_LED_Port, ADC_LED , GPIO_PIN_SET);
	  		HAL_Delay(10);
	  		HAL_GPIO_WritePin(ADC_LED_Port, ADC_LED , GPIO_PIN_RESET);
	  		//battery minimum Voltage
	  		if(ADC_1.BatVHeavyload!=0)
	  			ADC_1.BatV=min(ADC_1.BatVLightload, ADC_1.BatVHeavyload);
	  		else
	  			ADC_1.BatV=ADC_1.BatVLightload;
                
            //Battery BatVHeavyload Voltage > 3V, send BatVLightload battery Voltage
	  		if(ADC_1.BatV > 3)
	  		{
	  		   ADC_1.BatStatus = 0;
	  		   ADC_1.BatV=ADC_1.BatVLightload;

	  		}   
                
	  		//Battery Voltage < 2.8V send low battery alarm
	  		if(ADC_1.BatV >= 2.8)
	  			ADC_1.BatStatus = 0;
	  	    else
	  	    	ADC_1.BatStatus = 1;
            
	  		//Battery Voltage < 2.35V MCU always standby
	  		if(ADC_1.BatV < 2.35)
	  		{
	  			//TODO:Lora chip enter sleep
	  			USARTLoRa.Status=EnterSleepMode;
	  			LoRa_USART(&huart1, &hiwdg, &hadc1);

	  			/* Wait for any cleanup to complete before entering standby/shutdown mode */
	  			while (ErasingOnGoing == 1) { }

	  			/* Request to enter STANDBY mode  */
	  			HAL_PWR_EnterSTANDBYMode();
	  		}
	  		//detect wake-up pin for sensor status
	  		if(HAL_GPIO_ReadPin(WKUP_GPIO_Port, WKUP_Pin) == GPIO_PIN_SET) //high level by water detected
	  		{
	  			NodeStatus = TransmitWithAlarm;
	  			SendAlarmCount++;
	  		}
	  		else//High level by water not detected
	  		{
	  			NodeStatus = TransmitWithPWRST;
	  			SendAlarmCount = 0;
	  		}
	  			break;

	  	case TransmitWithPWRST:
	  		if(PWRST.PowerMode == StandbyMode)
			{
    			//TODO:Lora send data
	  			LoRa_USART(&huart1, &hiwdg, &hadc1);

    			//IWDG_Refresh
    			HAL_IWDG_Refresh(&hiwdg);

    			//TODO: Write EEPROM 20211005 by Wayne
				ee_status = EEPROM_Emu_write();

    			//TODO:MCU Standby Entry
    			EnterStandbyPWR_Mode(&hrtc);

    			/* This code should never go beyond this point. Reset on Standby wakeup */
    			Error_Handler();
    		}

    		else
    		{
    			USARTLoRa.ResStatus = LoRa_OK;
    			//TODO:Lora send data
    			LoRa_USART(&huart1, &hiwdg, &hadc1);

    			//TODO: Write EEPROM 20211005 by Wayne
				ee_status = EEPROM_Emu_write();

    			//IWDG_Refresh
    			HAL_IWDG_Refresh(&hiwdg);

    			NodeStatus = DataAcquire;

    		}
	  		  break;

	  	case TransmitWithAlarm:
	  		if(SendAlarmflag == true)
	  		{
	  			//IWDG_Refresh
	  			HAL_IWDG_Refresh(&hiwdg);

				USARTLoRa.ResStatus = LoRa_OK;
				//TODO:Lora send data
				LoRa_USART(&huart1, &hiwdg, &hadc1);

				//TODO: Write EEPROM 20211005 by Wayne
				ee_status = EEPROM_Emu_write();

				SendAlarmflag= false;

				NodeStatus = DataAcquire;

				//detect wake-up pin for sensor status
				if(HAL_GPIO_ReadPin(WKUP_GPIO_Port, WKUP_Pin) == GPIO_PIN_RESET) //low level by water no detected
				{
					SendAlarmCount = 0;
					//IWDG_Refresh
					HAL_IWDG_Refresh(&hiwdg);

					HAL_Delay(50);

					USARTLoRa.ResStatus = LoRa_OK;
					//TODO:Lora send data
					LoRa_USART(&huart1, &hiwdg, &hadc1);

					//TODO: Write EEPROM 20211005 by Wayne
					ee_status = EEPROM_Emu_write();


					//TODO:MCU Standby Entry
					EnterAlarmStnadby_Mode(&hrtc);
				}

	  		}
	  		else
	  		{
	  			HAL_Delay(50);
	  			//IWDG_Refresh
	  			HAL_IWDG_Refresh(&hiwdg);
	  		}
	  		break;
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSE;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV8;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_CRC);

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  LL_CRC_SetInputDataReverseMode(CRC, LL_CRC_INDATA_REVERSE_NONE);
  LL_CRC_SetOutputDataReverseMode(CRC, LL_CRC_OUTDATA_REVERSE_NONE);
  LL_CRC_SetPolynomialCoef(CRC, LL_CRC_DEFAULT_CRC32_POLY);
  LL_CRC_SetPolynomialSize(CRC, LL_CRC_POLYLENGTH_32B);
  LL_CRC_SetInitialData(CRC, LL_CRC_DEFAULT_CRC_INITVALUE);
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim2.Init.Prescaler = 1600-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10-1;
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//TODO-Wayne20210528
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart1);
//UNUSED(&lphuart1);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file
   */

  USARTLoRa.RevStrCount++; //all string receive count
  if(USARTLoRa.Rbuffer==0x0A)//when receive 0x0a("\n" Line feeds string),it is a effective command
  {
	   USARTLoRa.RevStrEndCount++;//0x0a("\n" Line feeds string) count
	  if( USARTLoRa.RevStrEndCount==1	&&	USARTLoRa.RevStrCount > 5)
		  USARTLoRa.ResetRevflag = 1;
  }

  if(!USARTLoRa.ResetRevflag)//if LoRa not receive reset message
  {
  	  USARTLoRa.RevData[USARTLoRa.RxCount++]= USARTLoRa.Rbuffer;
  	  if(USARTLoRa.RevData[0]!=0x0A)
  	  {
		USARTLoRa.RxCount= 0;
//		USARTLoRa.RevData[0]=0x0A;
  	  }
  	  if(USARTLoRa.RevStrEndCount == 2)
  	  {
  		  USARTLoRa.Revflag = 1;
  		  USARTLoRa.RxCount= 0;
  		  USARTLoRa.RevStrEndCount = 0;
  		  USARTLoRa.RevStrCount = 0;
//  		  int8_t c;
//  		  char RxHead_Check[] = "\n\r>> mac rx";
//  		  for(c=0;c<=10;c++)
//  		  {
//  			USARTLoRa.RxHead[c]=USARTLoRa.RevData[c];
//  		  }
//  		  if(strcmp(USARTLoRa.RxHead, RxHead_Check ) == 0)
//  			USARTLoRa.LoRaRxflag = 1;
  	  }
    }
    else//reset message receive
    {
    	if(USARTLoRa.RevStrEndCount > 6)
    	{
    		USARTLoRa.RevData[USARTLoRa.RxCount++]= USARTLoRa.Rbuffer;
    		if(USARTLoRa.RevStrEndCount == 9)
    		{
    			USARTLoRa.Revflag = 1;
    			USARTLoRa.RxCount= 0;
    			USARTLoRa.RevStrEndCount = 0;
    			USARTLoRa.RevStrCount = 0;
    		}
    	}

    }

  if(USARTLoRa.RxCount>=70)//if don't receive "\n" Line feeds string full over to 64 byte, clear data
  {
	  USARTLoRa.RxCount=0;
  	  memset( USARTLoRa.RevData, 0, strlen((const char*)USARTLoRa.RevData) ); //clear Receive data
  }
  while(HAL_UART_Receive_IT(&huart1,&USARTLoRa.Rbuffer,1)!=HAL_OK);
//  while(HAL_UART_Receive_IT(&lphuart1,&USARTLoRa.Rbuffer,1)!=HAL_OK);
}

//TODO for IWDG Freeze By Standby Mode
void IWDGFreezeBySTDBY(void)
{

	 HAL_FLASH_Unlock();

	 __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR); //clear FLASH Option validity error flag

	 HAL_FLASH_OB_Unlock();



	 HAL_FLASHEx_OBGetConfig(&pOBInit); //Get Option bit setting


	 //setup IWDG Freeze in standby mode
	 pOBInit.OptionType = OPTIONBYTE_USER;

	 pOBInit.USERType = OB_USER_IWDG_STDBY;

	 pOBInit.USERConfig = OB_IWDG_STDBY_FREEZE;

	 HAL_FLASHEx_OBProgram(&pOBInit);



	 HAL_FLASH_OB_Lock();

	 HAL_FLASH_Lock();
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @param  htim: TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == htim2.Instance)//timer2 interrupt per 1ms for trigger
    {
    	if(USARTLoRa.CrashTimerflag == 1)
    		USARTLoRa.CrashTimerCount++;
        if(USARTLoRa.TxRevTimerflag == 1)
    	    USARTLoRa.TxRevTimerCount++;
    	//Water detect timer
    	if(NodeStatus == TransmitWithAlarm)
    	{
    		SendAlarmTMRCount++;
    		if(SendAlarmTMRCount == 10000)//per 10s for trigger
    		{
        		SendAlarmflag = true ;
        		SendAlarmTMRCount = 0;
    		}
    	}

    }
}

/**
  * @brief PWR PVD interrupt callback
  * @retval None
  */

void HAL_PWR_PVDCallback(void)
{

	__NOP();
	//IWDG_Refresh
	HAL_IWDG_Refresh(&hiwdg);

	/* Request LoRa chip to enter Sleep mode  */
	//Hardware Reset
	HAL_GPIO_WritePin(RstPinGroup, RstPin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RstPinGroup, RstPin, GPIO_PIN_SET);
	//IWDG_Refresh
//	HAL_IWDG_Refresh(&hiwdg);
	uint32_t Nopi;
	for(Nopi=0;Nopi<4000000;Nopi++)
		__NOP();
	LoRaCommand(&huart1, Sleep24h);
    
    //PVD active, voltage <2V, set Heavyload = 2V to trigger sleep after reset.
	ADC_1.BatVHeavyload = 2;
	//TODO: Write EEPROM 20211005 by Wayne
	EE_Status ee_status;
	ee_status = EEPROM_Emu_write();
    
	/* Wait for any cleanup to complete before entering standby/shutdown mode */
	while (ErasingOnGoing == 1) { }

	/* Request to enter STANDBY mode  */
	HAL_PWR_EnterSTANDBYMode();
 }

/**
  * @brief  FLASH end of operation interrupt callback.
  * @param  ReturnValue: The value saved in this parameter depends on the ongoing procedure
  *                  Mass Erase: Bank number which has been requested to erase
  *                  Page Erase: Page which has been erased
  *                    (if 0xFFFFFFFF, it means that all the selected pages have been erased)
  *                  Program: Address which was selected for data program
  * @retval None
  */
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
  /* Call CleanUp callback when all requested pages have been erased */
  if (ReturnValue == 0xFFFFFFFF)
  {
    EE_EndOfCleanup_UserCallback();
  }
}

/**
  * @brief  Clean Up end of operation interrupt callback.
  * @param  None
  * @retval None
  */
void EE_EndOfCleanup_UserCallback(void)
{
  ErasingOnGoing = 0;
}

//TODO for EEPROM emulation Read
EE_Status EEPROM_Emu_read(void)
{
	EE_Status ee_status;
	/* Read all the variables */
     for (Index = 1; Index < NB_OF_VARIABLES+1; Index++)
     {
       ee_status = EE_ReadVariable32bits(Index, &VarValue);
       switch(Index)
       {
       	case 1:
       		ADC_1.BatVHeavyload = (float)VarValue/100;
       		break;

       }
 //    if (VarValue != a_VarDataTab[Index-1]) {Error_Handler();}
     if (ee_status != EE_OK) {Error_Handler();}
// 	  __NOP();
	return ee_status;
     }
}

//TODO for EEPROM emulation Write
EE_Status EEPROM_Emu_write(void)
{
	EE_Status ee_status;
	/* Unlock the Flash Program Erase controller */
    HAL_FLASH_Unlock();
    /* Store values of all variables in EEPROM, ascending order */
    for (Index = 1; Index < NB_OF_VARIABLES+1; Index++)
    {
    	/* Wait any cleanup is completed before accessing flash again */
    	while (ErasingOnGoing == 1) { }

		switch(Index)
		{
			case 1:
//				VarValue = 1.22233*100;
				VarValue = ADC_1.BatVHeavyload*100;
			break;
		}
		ee_status = EE_WriteVariable32bits(Index, VarValue);
		ee_status|= EE_ReadVariable32bits(Index, &a_VarDataTab[Index-1]);
    	if (VarValue != a_VarDataTab[Index-1]) {Error_Handler();}

    	/* Start cleanup IT mode, if cleanup is needed */
    	if ((ee_status & EE_STATUSMASK_CLEANUP) == EE_STATUSMASK_CLEANUP) {ErasingOnGoing = 1;ee_status|= EE_CleanUp_IT();}
    	if ((ee_status & EE_STATUSMASK_ERROR) == EE_STATUSMASK_ERROR) {Error_Handler();}
    }

    /* Lock the Flash Program Erase controller */
    HAL_FLASH_Lock();
    /* Wait for any cleanup to complete before entering standby/shutdown mode */
    while (ErasingOnGoing == 1) { }
    return ee_status;

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
