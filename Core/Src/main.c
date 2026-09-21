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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// #include <stdio.h>
#include "MyUART.h"
#include "AS5600.h"
#include "arm_math.h"
#include "FOC.h"
#include <stdint.h>

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

// float a = 3.14f;

float Uq = 0, Ud = 0;
float U[3] = {0,0,0};   //三相电压值存储
// float Angle = 0;

int16_t iabc_raw[3] = {0,0,0};

float ialpha_beta[2] = {0,0};

float idq[2] = {0,0};

float idq_ref[2] = {0.0f,0.100f};

// --标志位--

uint8_t i_curb_flag = 0;    //电流环控制标志位


// 初始化所需值

uint32_t ia_zero = 0;
uint32_t ib_zero = 0;

uint8_t i_count = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {

        if (i_curb_flag == 1) //电流偏置量获取
        {
            ia_zero += HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
            ib_zero += HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
            i_count ++;
            if (i_count >= 100)
            {
                ia_zero /= 100;
                ib_zero /= 100;
                i_curb_flag = 2;
            }
        }

        if (i_curb_flag == 2)   // 电流环控制
        {
            iabc_raw[0] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1)-ia_zero;
            iabc_raw[1] = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2)-ib_zero;
            iabc_raw[2] = -iabc_raw[0] - iabc_raw[1];

            Get_Electrical_Angle();

            Get_idiq(iabc_raw, ialpha_beta, idq);

            Current_PI_Control(idq_ref, idq);
            // Set_PhaseVoltage(Uq, Ud);
        }
    }
}

void Get_i_zero()
{
    Delay_ms(500);
    Set_PhaseVoltage(0, 0);
    Delay_ms(500);
    i_curb_flag = 1;
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */


    AS5600_Init();

	MyFOC_Init();
	
	Get_ZeroElAngle();  //获取零角度
	
    HAL_ADCEx_InjectedStart_IT(&hadc1); // 开启ADC1的注入通道

    Get_i_zero();       //获取电流偏置

    HAL_TIM_Base_Start_IT(&htim2);  // 开启定时器2

	// Uq = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	// Get_PhaseVoltage(U);
	// Angle = Get_Angle();
    // printf("%f,%f,%f,%f\r\n",U[0],U[1],U[2],Angle);
    
    // Angle = AS5600_GetAngleData() / 4096.0 * 2*PI;
    // printf("%f\r\n",Angle);

    // printf("%d,%d\r\n",AS5600_S.as5600Status,AS5600_S.MagStatus);

    //----ADCtest----

    // HAL_ADC_Start(&hadc1);
    // HAL_ADC_PollForConversion(&hadc1, 100);
    // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADC_Value, 2);

    // uint8_t TIM3C4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
    // uint8_t TIM3C3 = 0;
    // if (TIM3C4 == 1)
    // {
    //     TIM3C3 ++;
    // }
    
    // printf("%d,%d,%d\r\n",iabc_raw[0],iabc_raw[1],iabc_raw[2]);

    // printf("%d,%d\r\n",TIM3C4,TIM3C3);

    // uint32_t ADC_Value = HAL_ADC_GetValue(&hadc1);

    
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 42;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

uint16_t T2Count1 = 0;
uint16_t FOC_Time = 1;

//定时器中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		T2Count1++;
		if (T2Count1 >= FOC_Time)
		{
			T2Count1 = 0;
			// Set_PhaseVoltage(Uq, Ud, Get_Electrical_Angle());
            // Get_PhaseVoltage(U);
            // printf("%f,%f,%f\r\n",U[0],U[1],U[2]);

            // printf("%d,%d,%d\r\n",iabc_raw[0],iabc_raw[1],iabc_raw[2]);

            printf("%f,%f\r\n",idq[0],idq[1]);

            // printf("%f,%f\r\n",ialpha_beta[0],ialpha_beta[1]);

            // printf("%d,%d\r\n",ia_zero,ib_zero);
		}
	}
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
