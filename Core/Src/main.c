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
#include <cmsis_gcc.h>
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


//-------------------------- KALMAN STATE UPDATE -----------------------------//


float updateC(kalman_state* kstate, float measurement) {
	float temp_p = kstate->p + kstate->q;
	float temp_k = temp_p / (temp_p + kstate->r);
	float temp_x = kstate->x + temp_k * (measurement - kstate->x);
	float temp_p_updated = (1 - temp_k) * temp_p;

	// Step 2: Update kstate based on a condition


	// check for error
	uint32_t err =  __get_FPSCR();
	err = err & 0x0000000F;
	__set_FPSCR(0);

	if (err > 0) {
		return -1;
	}

	kstate->p = temp_p;
	kstate->k = temp_k;
	kstate->x = temp_x;
	kstate->p = temp_p_updated;

	return kstate->x;
}

float updateCMSIS(kalman_state* kstate, float measurement) {

    float32_t temp1, temp2, temp_p, temp_k, temp_x;
    float32_t k = 1.0f;

	arm_add_f32(&kstate->p, &kstate->q, &temp_p, 1); // p = p + q
	arm_add_f32(&temp_p, &kstate->r, &temp1, 1); // temp1 = p + r
	temp_k = temp_p / temp1;  // k = p / (p + r)
	arm_sub_f32(&measurement, &kstate->x, &temp1, 1); // temp1 = (measurement - x)
	arm_mult_f32(&temp_k, &temp1, &temp2, 1); // temp2 = k * (measurement - x)
	arm_add_f32(&kstate->x, &temp2, &temp_x, 1); // x = x + k * (measurement - x)
	arm_sub_f32(&k, &temp_k, &temp1, 1); // temp1 = (1 - k)
	arm_mult_f32(&temp1, &temp_p, &temp_p, 1); // p = (1 - k) * p

	/* check for error
	uint32_t err =  __get_FPSCR();
	err = err & 0x0000000F;
	__set_FPSCR(0);

	if (err > 0) {
		return -1;
	}**/

	kstate->p = temp_p;
	kstate->k = temp_k;
	kstate->x = temp_x;
	kstate->p = temp_p;



    return kstate->x;
}


//---------------------------- KALMAN FILTERS -------------------------------//


int KalmanfilterARM(float* InputArray, float* OutputArray, kalman_state* kstate, int Length) {
	for (int i=0; i<Length; i++) {
		if (isnan(kstate->x) > 0)
			return 1;

		OutputArray[i] = kalman(kstate, InputArray[i]); // change to kalman for .s function

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

//------------------------- ANALYSIS FUNCTIONS ----------------------------//

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


void ComputeConvolutionArraysC(float* InputArray1, float* InputArray2, float* ResultArray, int Length) {
	for (int i = 0; i < (2*Length - 1); i++) {
		ResultArray[i] = 0.0f;
        for (int j = 0; j < Length; j++) {
        	if (i-j >= 0 && i-j < Length)
        		ResultArray[i] += InputArray1[j] * InputArray2[i-j];
        }
    }
}

void ComputeCorrelationArraysC(float* InputArray1, float* InputArray2, float* CorrArray, int Length) {
	for (int i=0; i<(2*Length -1); i++) { //based on the documentation we have to intialize to 0
			CorrArray[i] = 0.0f;
	}

	float interArray[Length];

	for (int j = Length; j>=0; j--) {
		interArray[Length - j] = InputArray2[j-1];

	}

	/** In the arm CMSIS documentation it states that correlation is just the
	 *  input array convolved with the other array but reversed so that it is
	 *  done by calling the convolution function but first reversing the array
	 *  and placing it into an intermediate array
	 **/

	ComputeConvolutionArraysC(InputArray1, interArray, CorrArray, Length);
}

void ComputeDifferenceArraysCMSIS(float* InputArray1, float* InputArray2, float* ResultArray, int Length) {
	arm_sub_f32(InputArray1, InputArray2, ResultArray, Length);
}

void ComputeAverageAndStandardDeviationArrayCMSIS(float* InputArray, float* Average, float* StandardDeviation, int Length) {
	arm_mean_f32(InputArray, Length, Average);
	arm_std_f32(InputArray, Length, StandardDeviation);
}


void ComputeCorrelationArraysCMSIS(float* InputArray1, float* InputArray2, float* CorrArray, int Length) {
	for (int i=0; i<(2*Length -1); i++) { //based on the documentation we have to intialize to 0
		CorrArray[i] = 0.0f;
	}
	arm_correlate_f32(InputArray1, Length, InputArray2, Length, CorrArray);
}

void ComputeConvolutionArraysCMSIS(float* InputArray1, float* InputArray2, float* ResultArray, int Length) {
	arm_conv_f32(InputArray1, Length, InputArray2, Length, ResultArray);
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
  // Kalman filter states
    kalman_state ksC = {0.1f, 0.1f, 5.0f, 0.1f, 0.0f};
    kalman_state ksCMSIS = {0.1f, 0.1f, 5.0f, 0.1f, 0.0f};
    kalman_state ksARM = {0.1f, 0.1f, 5.0f, 0.1f, 0.0f};


    // Input and output arrays
    float InputArray[] = {10.4915760032, 10.1349974709, 9.53992591829, 9.60311878706,
            10.4858891793, 10.1104642352, 9.51066931906, 9.75755656493,
            9.82154078273, 10.2906541933, 10.4861328671, 9.57321181356,
            9.70882714139, 10.4359069357, 9.70644021369, 10.2709894039,
            10.0823149505, 10.2954563443, 9.57130449017, 9.66832136479,
            10.4521677502, 10.4287240667, 10.1833650752, 10.0066049721,
            10.3279461634, 10.4767210803, 10.3790964606, 10.1937408814,
            10.0318963522, 10.4939180917, 10.2381858895, 9.59703103024,
            9.62757986516, 10.1816981174, 9.65703773168, 10.3905666599,
            10.0941977598, 9.93515274393, 9.71017053437, 10.0303874259,
            10.0173504397, 9.69022731474, 9.73902896102, 9.52524419732,
            10.3270730526, 9.54695650657, 10.3573960542, 9.88773266876,
            10.1685038683, 10.1683694089, 9.88406620159, 10.3290065898,
            10.2547227265, 10.4733422906, 10.0133952458, 10.4205693583,
            9.71335255372, 9.89061396699, 10.1652744131, 10.2580948608,
            10.3465431058, 9.98446410493, 9.79376005657, 10.202518901,
            9.83867150985, 9.89532986869, 10.2885062658, 9.97748768804,
            10.0403923759, 10.1538911808, 9.78303667556, 9.72420149909,
            9.59117495073, 10.1716116012, 10.2015818969, 9.90650056596,
            10.3251329834, 10.4550120431, 10.4925749165, 10.1548177178,
            9.60547133785, 10.4644672766, 10.2326496615, 10.2279703226,
            10.3535284606, 10.2437410625, 10.3851531317, 9.90784804928,
            9.98208344925, 9.52778805729, 9.69323876912, 9.92987312087,
            9.73938925207, 9.60543743477, 9.79600805462, 10.4950988486,
            10.2814361401, 9.7985283333, 9.6287888922, 10.4491538991,
            9.5799256668};

    int Length = sizeof(InputArray)/sizeof(float);
    float OutputArrayC[Length], OutputArrayCMSIS[Length], OutputArrayASM[Length];

    // Run each implementation of the Kalman filter
    KalmanfilterC(InputArray, OutputArrayC, &ksC, Length);
    KalmanfilterCMSIS(InputArray, OutputArrayCMSIS, &ksCMSIS, Length);
    KalmanfilterARM(InputArray, OutputArrayASM, &ksARM, Length);

    // Arrays and float for comparing the outputs between implementations
    float Difference_C[Length], Difference_CMSIS[Length];
    float Correlation_C[(2 * Length - 1)], Correlation_CMSIS[(2 * Length - 1)];
    float Convolution_C[(2 * Length - 1)], Convolution_CMSIS[(2 * Length - 1)];
    float StdDev_C, Avg_C;
    float StdDev_CMSIS, Avg_CMSIS;

    //-------- ANALYSES -------//

    // Compute output differences
    ComputeDifferenceArraysC(InputArray, OutputArrayC, Difference_C, Length);
    ComputeDifferenceArraysCMSIS(InputArray, OutputArrayCMSIS, Difference_CMSIS, Length);

    // Compute average and standard deviation for each of these difference arrays
    ComputeAverageAndStandardDeviationArrayC(Difference_C, &Avg_C, &StdDev_C, Length);
    ComputeAverageAndStandardDeviationArrayCMSIS(Difference_CMSIS, &Avg_CMSIS, &StdDev_CMSIS, Length);

    // Correlation between outputs
    ComputeCorrelationArraysC(InputArray, OutputArrayC, Correlation_C, Length);
    ComputeCorrelationArraysCMSIS(InputArray, OutputArrayCMSIS, Correlation_CMSIS, Length);

    // Convolution between outputs
    ComputeConvolutionArraysC(InputArray, OutputArrayC, Convolution_C, Length);
    ComputeConvolutionArraysCMSIS(InputArray, OutputArrayCMSIS, Convolution_CMSIS, Length);
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
