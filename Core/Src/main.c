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
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "buffer.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
    E_1_2 = 0,
    E_2_3 = 1,
    E_3_4 = 2,
    E_4_5 = 3,
    E_5_6 = 4,
    E_6_7 = 5,
    E_7_8 = 6,
    E_8_9 = 7,
    E_9_10 = 8,
    E_10_11 = 9,
    E_11_12 = 10,
    E_12_13 = 11,
    E_13_14 = 12,
    E_14_15 = 13,
    E_15_16 = 14,
    E_16_1 = 15,
	NUM_ELECTRODE_PAIRS
} Electrode_t;

typedef struct {
	uint16_t mux;
	dbleBuf_t* adc1;
	dbleBuf_t* adc2;
	dbleBuf_t* adc3;
	dbleBuf_t* adc4;
	dbleBuf_t* adc5;
}buffers_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_SIZE 256 //The size of the adc buffer that values are read into

#define FREQ_CURRENT 1000
#define LUT_SIZE 256

#define TIMER2_PRESCALAR 1
#define TIMER2_FREQ 5e6
#define USB_PAYLOAD 32

#define DAC_FREQ 1e6
#define MAIN_FREQ 1
#define USB_FREQ 1e3
#define CLKFREQ 96e6
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

void send_all_buffer(buffers_t* buff);
void set_mux(uint8_t index);
void send_adc_buffer(int32_t *array, size_t size, uint8_t mux, uint8_t adc);
void write_dac(uint16_t value);
void init_lut(void);
void updateCurrent(Electrode_t electrodes);
void init_all_buffers(buffers_t* buff);
void USB_Send(char* message);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t sinewave[LUT_SIZE];
uint32_t sine_idx;
bool all_readings_done = false;

buffers_t* buffers;

uint64_t gl_ticks = 0;
bool is_main = 0;
bool is_dac = 0;
bool is_usb = 0;

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_ADC5_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USB_Device_Init();
  /* USER CODE BEGIN 2 */

  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK)
  	  Error_Handler();
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK)
  	  Error_Handler();
  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED) != HAL_OK)
  	  Error_Handler();
  if (HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED) != HAL_OK)
  	  Error_Handler();
  if (HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED) != HAL_OK)
  	  Error_Handler();

  if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
  	  Error_Handler();
  if (HAL_ADC_Start_IT(&hadc2) != HAL_OK)
  	  Error_Handler();
  if (HAL_ADC_Start_IT(&hadc3) != HAL_OK)
  	  Error_Handler();
  if (HAL_ADC_Start_IT(&hadc4) != HAL_OK)
  	  Error_Handler();
  if (HAL_ADC_Start_IT(&hadc5) != HAL_OK)
  	  Error_Handler();

  uint32_t period = (uint32_t)(CLKFREQ / (TIMER2_PRESCALAR * TIMER2_FREQ) - 1);
  TIM2->ARR = period;
  TIM2->PSC = TIMER2_PRESCALAR;

  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
	  Error_Handler();
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  	  Error_Handler();

  init_lut();
  init_all_buffers(buffers);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (is_main) {

		  is_main = 0;
	  }
	  if (is_dac) {
		  write_dac(sinewave[sine_idx]);
		  sine_idx++;
		  is_dac = 0;
	  }
	  if (is_usb) {
		  send_all_buffer(buffers);
		  is_usb = 0;
	  }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);
}

/* USER CODE BEGIN 4 */

//---------- ADC Callback ----------
/*
 * Reads all ADC values into the appropriate channel
 * Struct. Passes if all channels have filled there
 * buffers.
 *
 * Args:
 * 		hadc: handle type for the adc, defines which
 * 			adc is being used
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
    	uint32_t adc1_val = HAL_ADC_GetValue(hadc);
    	writeDbleBuf (buffers->adc1, adc1_val);
    }
    if (hadc->Instance == ADC2) {
    	uint32_t adc2_val = HAL_ADC_GetValue(hadc);
    	writeDbleBuf (buffers->adc2, adc2_val);
    }
    if (hadc->Instance == ADC3) {
    	uint32_t adc3_val = HAL_ADC_GetValue(hadc);
    	writeDbleBuf (buffers->adc3, adc3_val);
    }
    if (hadc->Instance == ADC4) {
    	uint32_t adc4_val = HAL_ADC_GetValue(hadc);
    	writeDbleBuf (buffers->adc4, adc4_val);
    }
    if (hadc->Instance == ADC5) {
    	uint32_t adc5_val = HAL_ADC_GetValue(hadc);
    	writeDbleBuf (buffers->adc5, adc5_val);
    }
}

//---------- DAC Callback ----------
/*
 * Calls when the timer reaches its set period.
 * Will write a new value to the DAC port according
 * to a sinewave LUT.
 *
 * Args:
 * 		htim: handle type for the timer to define
 * 			which timer has triggered the callback.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM2) {
		gl_ticks++;
	    if (gl_ticks % (uint64_t)(TIMER2_FREQ / MAIN_FREQ) == 1) {
			is_main = 1; //1Hz
		}
	    if (gl_ticks % (uint64_t)(TIMER2_FREQ / DAC_FREQ) == 1) {
			is_dac = 1; //1Hz
		}
	    if (gl_ticks % (uint64_t)(TIMER2_FREQ / USB_FREQ) == 1) {
	    	is_usb = 1;
	    }
        if (gl_ticks >= TIMER2_FREQ) {
            gl_ticks = 0;
        }
	}

}

void init_all_buffers(buffers_t* buff) {
	initDbleBuf(buff->adc1, ADC_SIZE);
	initDbleBuf(buff->adc2, ADC_SIZE);
	initDbleBuf(buff->adc3, ADC_SIZE);
	initDbleBuf(buff->adc4, ADC_SIZE);
	initDbleBuf(buff->adc5, ADC_SIZE);
	buff->mux = 0;
}

//----------Buffer Send ----------
/*
 * Sends the currently active buffer in the channel struct
 * to a host PC.
 *
 * Args:
 * 		channel: the channel with a full buffer
 * 			that needs to be sent.
 */
void send_adc_buffer(int32_t *array, size_t size, uint8_t mux, uint8_t adc) {
	char buffer[USB_PAYLOAD + 1];
	for (size_t i = 0; i < size; i++) {
		uint16_t value = array[i];
		snprintf (buffer, sizeof (buffer), "A%uM%u %u\n",adc, mux, value);
		USB_Send(buffer);
	}
}

void send_all_buffer(buffers_t* buff) {
	int32_t *array;
	readDbleBuf (buff->adc1, array);
	send_adc_buffer(array, ADC_SIZE, buff->mux, 1);
	readDbleBuf (buff->adc2, array);
	send_adc_buffer(array, ADC_SIZE, buff->mux, 2);
	readDbleBuf (buff->adc3, array);
	send_adc_buffer(array, ADC_SIZE, buff->mux, 3);
	readDbleBuf (buff->adc4, array);
	send_adc_buffer(array, ADC_SIZE, buff->mux, 4);
	readDbleBuf (buff->adc5, array);
	send_adc_buffer(array, ADC_SIZE, buff->mux, 5);
}


//---------- Voltage MUX ----------
/*
 * Sets the voltage multiplexers to one of four possible
 * combinations. This gives 16 different adc readings each
 * between 2 electrodes.
 *
 * Args:
 * 		index: The configuration that is being set, only four
 * 			combinations exist to get all readings.
 */
void set_mux(uint8_t index) {
	switch (index) {
		case 0:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
			break;
		default:
			Error_Handler();
	}
}


//---------- DAC write ----------
/*
 * Writes a 14 bit binary value to the parallel interface
 * on the external DAC. This is used to generate a sinewave.
 *
 * Args:
 * 		value: The number that is being written
 * 			to the DAC
 */
void write_dac(uint16_t value){
	sine_idx = 0;
	if (value >= 16384) {
		value = 16384;
	}
	GPIOC->ODR = value;
}


//---------- LUT setup ----------
/*
 * Sets up a look up table of a sinewave with a DC offset
 * and a max 14 bit amplitude. THe frequency is adjusted according
 * to the update rate of the DAC.
 *
 */
void init_lut(void) {
	uint32_t amplitude = 8192;
	for (int i = 0; i < LUT_SIZE; i++) {
		uint32_t new_freq = FREQ_CURRENT / (DAC_FREQ / LUT_SIZE);
		uint16_t sine = sin((i * 2.0 * M_PI * new_freq) / LUT_SIZE);
		sinewave[i] = (uint16_t)(amplitude * sine + amplitude);
	}
}



//---------- USB send string ----------
/*
 * Sends a string to the USB com port on a host PC.
 *
 * Args:
 * 		message: the string that is being sent
 */
void USB_Send(char* message) {
	CDC_Transmit_FS((uint8_t*)message, strlen(message));
}



void updateCurrent(Electrode_t electrodes) {
	switch (electrodes) {//must be a better way to do this, kinda sucks
		case E_1_2: //IN = 0001 IP = 0000
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
			break;
		case E_2_3: //IN = 0010 IP = 0001
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_13, GPIO_PIN_SET);
			break;
		case E_3_4: //IN = 0011 IP = 0010
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case E_4_5: //IN = 0100 IP = 0011
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_SET);
			break;
		case E_5_6: //IN = 0101 IP = 0100
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_11 , GPIO_PIN_SET);
			break;
		case E_6_7: //IN = 0110 IP = 0101
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_11|GPIO_PIN_13 , GPIO_PIN_SET);
			break;
		case E_7_8: //IN = 0111 IP = 0110
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case E_8_9: //IN = 1000 IP = 0111
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_SET);
			break;
		case E_9_10: //IN = 1001 IP = 1000
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_10, GPIO_PIN_SET);
			break;
		case E_10_11: //IN = 1010 IP = 1001
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_13 , GPIO_PIN_SET);
			break;
		case E_11_12: //IN = 1011 IP = 1010
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_11|GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case E_12_13: //IN = 1100 IP = 1011
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_11, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_SET);
			break;
		case E_13_14: //IN = 1101 IP = 1100
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11 , GPIO_PIN_SET);
			break;
		case E_14_15: //IN = 1110 IP = 1101
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5|GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_13, GPIO_PIN_SET);
			break;
		case E_15_16: //IN = 1111 IP = 1110
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);
			break;
		case E_16_1: //IN = 0000 IP = 1111
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_SET);
			break;
		default:
			Error_Handler();

			break;
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
