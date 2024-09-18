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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include <math.h>
#include <stdio.h>

typedef struct {
    float32_t q;
    float32_t r;
    float32_t x;
    float32_t p;
    float32_t k;
} kalman_state;


/* ----------------------------------------------------------------------
* Defines each of the tests performed
* ------------------------------------------------------------------- */
#define MAX_BLOCKSIZE   5
#define DELTA           (0.000001f)


/* ----------------------------------------------------------------------
** Test input data of srcB for blockSize 32
** ------------------------------------------------------------------- */

/* ----------------------------------------------------------------------
* Declare Global variables
* ------------------------------------------------------------------- */
float32_t testOutput;  /* Final ouput */

arm_status status;       /* Status of the example */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))



float updateC(kalman_state* kstate, float measurement) {
	kstate->p = kstate->p + kstate->q;
	kstate->k = kstate->p / ( kstate->p +  kstate->r );
	kstate->x = kstate->x + kstate->k * (measurement - kstate->x);
	kstate->p = (1 - kstate->k) * kstate->p;
	return kstate->x;
}

float updateCMSIS(kalman_state* kstate, float measurement) {

    float32_t temp1, temp2;

	arm_add_f32(&kstate->p, &kstate->q, &kstate->p, 1); // p = p + q
	arm_add_f32(&kstate->p, &kstate->r, &temp1, 1); // temp1 = p + r
	kstate->k = kstate->p / temp1;  // k = p / (p + r)
	arm_sub_f32(&measurement, &kstate->x, &temp1, 1); // temp1 = (measurement - x)
	arm_mult_f32(&kstate->k, &temp1, &temp2, 1); // temp2 = k * (measurement - x)
	arm_add_f32(&kstate->x, &temp2, &kstate->x, 1); // x = x + k * (measurement - x)
	arm_sub_f32(1.0f, &kstate->x, &temp1, 1); // temp1 = (1 - k)
	arm_mult_f32(&temp1, &kstate->p, &kstate->p, 1); // p = (1 - k) * p

    return kstate->x;
}

void ComputeDifferenceArraysC(float* InputArray1, float* InputArray2, float* ResultArray, int Length) {
    for (int i = 0; i < Length; i++) {
        ResultArray[i] = InputArray1[i] - InputArray2[i];
    }
}

void ComputeAverageAndStandardDeviationArrayC(float* InputArray, float* Average, float* StandardDeviation, int Length) {
    float sum = 0.0, squaredSum = 0.0;

    for (int i = 0; i < Length; i++) {
        sum += InputArray[i];
        squaredSum += InputArray[i] * InputArray[i];
    }

    *Average = sum / Length;
    *StandardDeviation = sqrt((squaredSum / Length) - (*Average) * (*Average));
}

void ComputeCorrelationArraysC(float* InputArray1, float* InputArray2, float* Correlation, int Length) {
    float squaredSum1 = 0.0, squaredSum2 = 0.0, sum12 = 0.0, sum1 = 0.0, sum2 = 0.0;

    for (int i = 0; i < Length; i++) {
        squaredSum1 += InputArray1[i] * InputArray1[i];
        squaredSum2 += InputArray2[i] * InputArray2[i];
        sum12 += InputArray1[i] * InputArray2[i];
        sum1 += InputArray1[i];
        sum2 += InputArray2[i];
    }

    float denominator = sqrt(((Length * squaredSum1) - (sum1 * sum1)) * ((Length * squaredSum2) - (sum2 * sum2)));

    if (denominator == 0) {
        return;
    }

    float numerator = (Length * sum12) - (sum1 * sum2);

    *Correlation = numerator / denominator;
}

void ComputeConvolutionArraysC(float* InputArray1, float* InputArray2, float* ResultArray, int Length) {
    for (int i = 0; i < Length; i++) {
        ResultArray[i] = 0.0;
        for (int j = 0; j < Length; j++) {
            if (i - j >= 0) {
                ResultArray[i] += InputArray1[j] * InputArray2[i - j];
            }
        }
    }
}


void ComputeDifferenceArraysCMSIS(float* InputArray1, float* InputArray2, float* ResultArray, int Length) {

	arm_sub_f32(InputArray1, InputArray2, ResultArray, Length);

}

float32_t ComputeStandardDeviationArrayCMSIS(float* InputArray, int Length) {
	float32_t std;
	arm_std_f32(InputArray, Length, &std);

	return std;

}


float32_t ComputeCorrelationArraysCMSIS(float* InputArray1, float* InputArray2, float* Correlation, int Length) {
	//correlation
	float32_t corr;
	arm_correlate_f32(InputArray1, Length, InputArray2, Length, &corr);

	return corr;

}

void ComputeConvolutionArraysCMSIS(float* InputArray1, float* InputArray2, float* ResultArray, int Length) {
	//convolution
		arm_conv_f32(InputArray1, Length, InputArray2, Length, ResultArray); //convolution
}


int KalmanfilterARM(float* InputArray, float* OutputArray, kalman_state* kstate, int Length) {
	for (int i=0; i<Length; i++) {
		if (isnan(kstate->x) > 0)
			return 1;

		kalman(kstate, InputArray[i]); // change to kalman for .s function
		OutputArray[i]= kstate->x;
	}

	return 0;
}

int KalmanfilterCMSIS(float* InputArray, float* OutputArray, kalman_state* kstate, int Length) {
	for (int i=0; i<Length; i++) {
		if (isnan(kstate->x) > 0)
			return 1;
		OutputArray[i] = updateCMSIS(kstate, InputArray[i]); // change to kalman for .s function
	}

	return 0;

}

int KalmanfilterC(float* InputArray, float* OutputArray, kalman_state* kstate, int Length) {
	for (int i=0; i<Length; i++) {
		if (isnan(kstate->x) > 0)
			return 1;
		OutputArray[i] = updateC(kstate, InputArray[i]); // change to kalman for .s function
	}

	return 0;

}


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  /* USER CODE BEGIN 2 */

  // float updateC(kalman_state* kstate, float measurement)

  // int KalmanfilterC(float* InputArray, float* OutputArray, kalman_state* kstate, int Length)

  kalman_state ks = {0.1f, 0.1f, 5.0f, 0.1f, 0.0f};

  float InputArray[5] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
  float OutputArray[5];
  int Length = 5;

  /* for (int i=0; i<5; i++) {
	  updateC(&ks, i);

  }

  */

  KalmanfilterCMSIS(InputArray, OutputArray, &ks, Length);









  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
