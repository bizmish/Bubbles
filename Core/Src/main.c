/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED_Fonts.h"
#include "OLED.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_AVERAGE_SIZE 16
#define ADC_BUF_SIZE 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
char trans_str[64] = {0,};
uint16_t adc1Array [ADC_BUF_SIZE];
uint16_t adc2Array [ADC_BUF_SIZE];
uint16_t i,j,k;
int index1 = 0;
int index2 = 0;
uint32_t adcFlag = 0;
uint32_t deltaTimePeaks = 0;
uint32_t measureReady = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char* ftoa(double data, char *str, int precise)
{
    char sss[16];
    int sign = (data < 0);
    if (sign) data=0-data;
    uint32_t pow=1;
    int p=precise;
    while (p--) pow*=10;
    data*=pow;
    uint32_t DataInt = (uint32_t)(data+0.5);
    char *ps = sss;
    p=precise;
    while (p--) {*ps++ = (DataInt % 10)+'0'; DataInt/=10;}
    if (precise>0) *ps++='.';
    if (DataInt==0) *ps++ = '0';
    while (DataInt) {*ps++ = (DataInt % 10)+'0'; DataInt/=10;}
    if (sign) *ps++='-';
    ps--;
    while (ps>=sss) {*str++ = *ps--;}
    *str=0;
    return str;
}

/*    Оцифровка с частотой 16 kSPS/s   */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    static uint32_t subindex = 0;
    static int32_t adc1Summator = 0;
    static int32_t adc2Summator = 0;
    static int32_t adc1NullValue = INT32_MAX;
    static int32_t adc2NullValue = INT32_MAX;
    static uint32_t adc1NullSumm = 0;
    static uint32_t adc2NullSumm = 0;
    static uint32_t adcNullCount = 0;
    static uint32_t adc1NullPorog = 0;
    static uint32_t adc2NullPorog = 0;
    static uint32_t adc1FindingPeak = 0;
    static uint32_t adc2FindingPeak = 0;
    static uint32_t adc1PeakMin = 0;
    static uint32_t adc2PeakMin = 0;
    static uint32_t adc1PeakTime = 0;
    static uint32_t adc2PeakTime = 0;
    static uint32_t firstPeakFound = 0;
    static uint32_t adc1FlagDelay = 0;
    static uint32_t adc2FlagDelay = 0;
    static uint32_t adc1Flag = 0;
    static uint32_t adc2Flag = 0;
    
    if(hadc->Instance == ADC1) //check if the interrupt comes from ACD1
    {
        adc1Summator += HAL_ADC_GetValue(&hadc1);
        adc2Summator += HAL_ADC_GetValue(&hadc2);
        if (++subindex >= 16) { 
            subindex = 0;
            // Измерение нуля
            adc1NullSumm += adc1Summator;
            adc2NullSumm += adc2Summator;
            if (++adcNullCount >= 200) {  // 200 мс ровного сигнала считаем за нуль
                if (adc1NullSumm >5000) { // Нуль не должен быть слишком маленьким
                    adc1NullValue = adc1NullSumm/adcNullCount;
                    adc1NullPorog = adc1NullValue * 9 / 10;
                }
                if (adc2NullSumm >5000) { // Нуль не должен быть слишком маленьким
                    adc2NullValue = adc2NullSumm/adcNullCount;
                    adc2NullPorog = adc2NullValue * 9 / 10;
                }
                adcNullCount = 0;
                adc1NullSumm = 0;
                adc2NullSumm = 0;
            }
          
            if (adc1NullValue < INT32_MAX) {  // Нуль известен
                if (abs(adc1Summator-adc1NullValue) > 160) {  // Сорвалось измерение нуля
                    adcNullCount = 0;
                    adc1NullSumm = 0;
                    adc2NullSumm = 0;
                }
                if (adc1Flag == 0) {
                    adc1Array[index1] = adc1Summator * 60 / adc1NullValue;     // Осциллограмма ADC1
                    if (++index1 >= ADC_BUF_SIZE) index1 = 0;
                }
                if (adc2Flag == 0) {
                    adc2Array[index2] = adc2Summator * 60 / adc2NullValue - 3; // Осциллограмма ADC2
                    if (++index2 >= ADC_BUF_SIZE) index2 = 0;
                }

                if (!firstPeakFound) {
                    // Ожидание пузыря через первый датчик
                    if (adc1FindingPeak) {
                        if (adc1Summator < adc1PeakMin) {
                            adc1PeakMin = adc1Summator;
//                            adc1PeakTime = HAL_GetTick();
                        }
                        if (adc1Summator > adc1NullPorog) {
                            adc1FindingPeak = 0;
                            firstPeakFound++;
                        }
                    } else if (adc1Summator <= adc1NullPorog) {
                        adc1FindingPeak = 1;
                        adc1PeakTime = HAL_GetTick();
                        adc1PeakMin = adc1Summator;
                        if (adc1FlagDelay == 0) adc1FlagDelay = 110;
                    }
                } else {
                    // Ожидание пузыря через второй датчик
                    if (adc2FindingPeak) {
                        if (adc2Summator < adc2PeakMin) {
                            adc2PeakMin = adc2Summator;
//                            adc2PeakTime = HAL_GetTick();
                        }
                        if (adc2Summator > adc2NullPorog) {
                            adc2FindingPeak = 0;
                            firstPeakFound = 0;
                            deltaTimePeaks = adc2PeakTime - adc1PeakTime;
                            measureReady = 1;
                        }
                    } else if (adc2Summator <= adc2NullPorog) {
                        adc2FindingPeak = 1;
                        adc2PeakTime = HAL_GetTick();
                        adc2PeakMin = adc2Summator;
                        if (adc2FlagDelay == 0) adc2FlagDelay = 60;
                    }
                }

            }
            
            adc1Summator = 0;
            adc2Summator = 0;

            if ((adc1FlagDelay) && (--adc1FlagDelay == 0)) 
                adc1Flag = 1;
            if (adc2Flag) { 
                adc2Flag--;
                adc1Flag = adc2Flag;
            }
            if ((adc2FlagDelay) && (--adc2FlagDelay == 0)) adc2Flag = 1000;
            
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
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  char sss[40];
  
  OLED_Init(&hi2c2);  FontSet(Segoe_UI_Eng_12);  OLED_Clear(0);  OLED_DrawStr("start", 80, 0, RIGHT);  OLED_UpdateScreen();
  /* --------------------------------------------- */
  for (i=0; i<ADC_BUF_SIZE; i++) {adc1Array [i] = 0; adc2Array [i] = 0;}         // зачистка
  HAL_ADCEx_Calibration_Start(&hadc1);             // калибровка ацп    
  HAL_Delay(100);
  HAL_ADCEx_Calibration_Start(&hadc2);             // калибровка ацп    
  HAL_Delay(100);
  HAL_ADC_Start_IT(&hadc1);                        // запускаем преобразование сигнала АЦП1
  HAL_ADC_Start_IT(&hadc2);                        // запускаем преобразование сигнала АЦП2
  HAL_TIM_Base_Start   (&htim3);

  HAL_GPIO_WritePin(led_GPIO_Port,led_Pin,GPIO_PIN_SET);
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//    if (adcFlag == 0)
//    {
      OLED_Clear(0);
      int i81 = index1;
      int i82 = index2;
      for (int i8 = 0; i8 < ADC_BUF_SIZE; i8++)
      {
          if(++i81 >= ADC_BUF_SIZE) i81 = 0;
          if(++i82 >= ADC_BUF_SIZE) i82 = 0;
          OLED_DrawPixel(i8, 63 - adc1Array[i81]);
          OLED_DrawPixel(i8, 63 - adc2Array[i82]);
      }
      if (1) {//(measureReady) {
          measureReady = 0;
          char *sssend = ftoa(deltaTimePeaks, sss, 0);
          *sssend++ = ' ';
          *sssend++ = 'm';
          *sssend++ = 's';
          *sssend = 0;
          FontSet(Segoe_UI_Eng_12);                      // шрифт
          OLED_DrawStr(sss, 20, 40, LEFT);
          if (deltaTimePeaks>0) {
              float value = 20*60 / (float)deltaTimePeaks; // V=20мл, мл/мс -> л/мин
              char *sssend = ftoa(value, sss, 2);
              *sssend++ = ' ';
              *sssend++ = 's';
              *sssend++ = 'l';
              *sssend++ = 'p';
              *sssend++ = 'm';
              *sssend = 0;
              OLED_DrawStr(sss, 20, 52, LEFT);
          }
      }
      OLED_UpdateScreen();
      //HAL_GPIO_TogglePin(led_GPIO_Port,led_Pin);
//    }

    //HAL_ADC_PollForConversion(&hadc1, 100); // ожидаем окончания преобразования
    //adc = HAL_ADC_GetValue(&hadc1); // читаем полученное значение в переменную adc
    //HAL_ADC_Stop(&hadc1); // останавливаем АЦП (не обязательно)
    
    
    //snprintf(trans_str, 10, "%d", adc);
    //OLED_Clear(0);
    //OLED_DrawStr(trans_str, 2, 2, RIGHT);
    //OLED_UpdateScreen();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_DUALMODE_REGSIMULT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  hi2c2.Init.ClockSpeed = 400000;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 35;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 124;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led_Pin */
  GPIO_InitStruct.Pin = led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
