/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Embedded software for use in an EIT open source development
  * 				board. Production files can be found at:
  * 				https://data.mendeley.com/preview/6dz82jxzvh?a=f4632e7c-eceb-4229-b38e-013b2df32c65
  *
  * 				Workflow of the program is to read adc values into a double buffer
  * 				based on a timer interupt. These values are sent to a host PC for
  * 				processing. Multiplexing occurs after every succesfully sent buffer.
  * 				Current stimulation is done by communication with an external AD9707
  * 				DAC via parallel communicaiton.
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
#include "cordic.h"
#include "dma.h"
#include "hrtim.h"
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
#include "usbd_cdc_if.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 128
//#define BUFFER_SIZE 1024
#define MEAS_SIZE 256
#define LUT_SIZE 100 //Size of the stimulation Look Up Table

#define TIMER2_PRESCALAR 1 //Prescalar for TIM2
#define TIMER2_FREQ 1e5 //Desired Frequency for TIM2


#define DAC_FREQ 1e4 //Desired Frequency of the Stimulation waveform
#define SAMPLING_FREQUENCY 1e5


#define MAIN_FREQ 2 //Update frequency of the Main program
#define VOLTAGE_FREQ 20
//#define VOLTAGE_FREQ 10


#define CLKFREQ 170e6 //SYSCKL Frequency
#define HRTIM_MUL 32 //HRTIM multiplier
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

void updateVoltage(uint16_t index);
void init_lut(void);
void updateCurrent(uint16_t electrodes);
void USB_Send(char* message);
void WriteBits(GPIO_TypeDef* GPIOx, uint16_t pinMask, uint8_t value);
uint16_t calcMag(uint16_t * array);
void checkStim(void);
void setGain(uint16_t adc, uint8_t gain);
void calcMagnitude(void);
void sendMagnitude(void);
void sendBuffers(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t sinewave[LUT_SIZE];
uint32_t sine_idx;

uint8_t current_mux = 0;
uint8_t voltage_mux = 0;

//buffers_t buffers;

volatile uint64_t gl_ticks = 0;
volatile bool is_main = 0;
volatile bool is_dac = 0;
volatile bool is_usb = 0;
volatile bool is_voltage_mux = 0;
volatile bool is_current_mux = 0;


uint16_t adc1Buff[BUFFER_SIZE] = {0};
uint16_t adc2Buff[BUFFER_SIZE] = {0};
uint16_t adc3Buff[BUFFER_SIZE] = {0};
uint16_t adc4Buff[BUFFER_SIZE] = {0};
uint16_t adc5Buff[BUFFER_SIZE] = {0};
uint16_t magnitude[MEAS_SIZE] = {0};


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
  MX_USB_Device_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_ADC5_Init();
  MX_TIM2_Init();
  MX_HRTIM1_Init();
  MX_CORDIC_Init();
  /* USER CODE BEGIN 2 */



  //---------- ADC Calibration ----------
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

  //---------- ADC Start ----------
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1Buff, BUFFER_SIZE);
  HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2Buff, BUFFER_SIZE);
  HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3Buff, BUFFER_SIZE);
  HAL_ADC_Start_DMA(&hadc4, (uint32_t*)adc4Buff, BUFFER_SIZE);
  HAL_ADC_Start_DMA(&hadc5, (uint32_t*)adc5Buff, BUFFER_SIZE);

  //---------- Timer Period Set ----------
  uint32_t period = (uint32_t)(CLKFREQ / (2*TIMER2_PRESCALAR * TIMER2_FREQ) - 1);
  TIM2->ARR = period;
  TIM2->PSC = TIMER2_PRESCALAR;


  uint32_t updateFreq = (LUT_SIZE*DAC_FREQ);
  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].PERxR = HRTIM_MUL*(CLKFREQ/updateFreq);

  HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].PERxR = HRTIM_MUL*(CLKFREQ/SAMPLING_FREQUENCY);


  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  	  Error_Handler();


  if (HAL_HRTIM_SimpleBaseStart(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B) != HAL_OK)
    	  Error_Handler();

  HAL_Delay(400);

  //---------- LUT Initialisation ----------
  init_lut();
  HAL_Delay(100);
  HAL_HRTIM_SimpleBaseStart_DMA(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, (uint32_t)&sinewave[0], (uint32_t)&GPIOC->ODR, LUT_SIZE);



  //---------- MUX Enable ----------
  updateCurrent(0);
  updateVoltage(0);
//  setGain(1,0b0011);
//  setGain(2,0b0011);
//  setGain(3,0b0011);
//  setGain(4,0b0011);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET);






  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	if (is_main) { // Toggles a status LED
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
		is_main = 0;
	}
	if (is_voltage_mux) { //Updates Voltage MUX and triggers a MUX change
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
//		calcMagnitude();
		sendBuffers();
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);

		voltage_mux++;
		if (voltage_mux > 3) {
			voltage_mux = 0;
			current_mux++;
			if (current_mux > 15) {
				  current_mux = 0;
				  uint16_t marker = 0xAA00;
				  CDC_Transmit_FS((uint8_t*)&marker, 2);

//				  sendMagnitude();

				}
		}
		updateVoltage(voltage_mux);
		updateCurrent(current_mux);
		checkStim();
		is_voltage_mux = 0;
	  }

    /* USER CODE END WHILE */
  }
    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV12;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
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
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_2);
}

/* USER CODE BEGIN 4 */




//---------- TIM2 Callback ----------
/*
 * Calls when the timer reaches its set period.
 * Currently used for basic timer scheduling of tasks
 * in the main function.
 *
 * Args:
 * 		htim: Handle type for the timer to define
 * 			which timer has triggered the callback.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM2) {
		gl_ticks++;

		if (gl_ticks % (uint64_t)(TIMER2_FREQ / MAIN_FREQ) == 1) {
			is_main = 1;
		}
	    if (gl_ticks % (uint64_t)(TIMER2_FREQ / VOLTAGE_FREQ) == 1) {
	    	is_voltage_mux = 1;
	    }
	}

}


void sendMagnitude(void) {
	uint8_t* data = (uint8_t*)magnitude;
	CDC_Transmit_FS(data, 512);
}

void sendBuffers(void) {
	uint8_t idx_A1 = (current_mux) *16 + 4 * (1 - 1) + voltage_mux;
	uint8_t idx_A2 = (current_mux) *16 + 4 * (2 - 1) + voltage_mux;
	uint8_t idx_A3 = (current_mux) *16 + 4 * (3 - 1) + voltage_mux;
	uint8_t idx_A4 = (current_mux) *16 + 4 * (4 - 1) + voltage_mux;

	__disable_irq();

	uint8_t* data1 = (uint8_t*)adc1Buff;
	data1[0] = idx_A1;


	uint8_t* data2 = (uint8_t*)adc2Buff;
	data2[0] = idx_A2;


	uint8_t* data3 = (uint8_t*)adc3Buff;
	data3[0] = idx_A3;


	uint8_t* data4 = (uint8_t*)adc4Buff;
	data4[0] = idx_A4;
	__enable_irq();


	CDC_Transmit_FS(data1, BUFFER_SIZE*2);
	CDC_Transmit_FS(data2, BUFFER_SIZE*2);
	CDC_Transmit_FS(data3, BUFFER_SIZE*2);
	CDC_Transmit_FS(data4, BUFFER_SIZE*2);
}

void calcMagnitude(void) {
	int firstElec_A1 = 4 * (1 - 1) + voltage_mux + 1;
	int firstElec_A2 = 4 * (2 - 1) + voltage_mux + 1;
	int firstElec_A3 = 4 * (3 - 1) + voltage_mux + 1;
	int firstElec_A4 = 4 * (4 - 1) + voltage_mux + 1;

	int firstStim = current_mux + 1;

	uint8_t idx_A1 = (firstStim - 1) *16 + firstElec_A1 - 1;
	uint8_t idx_A2 = (firstStim - 1) *16 + firstElec_A2 - 1;
	uint8_t idx_A3 = (firstStim - 1) *16 + firstElec_A3 - 1;
	uint8_t idx_A4 = (firstStim - 1) *16 + firstElec_A4 - 1;

	magnitude[idx_A1] = calcMag(adc1Buff);
	magnitude[idx_A2] = calcMag(adc2Buff);
	magnitude[idx_A3] = calcMag(adc3Buff);
	magnitude[idx_A4] = calcMag(adc4Buff);


}


uint16_t calcMag(uint16_t * array) {
	q15_t max, min;
	uint32_t idxMax, idxMin;
	arm_max_q15((q15_t*)array, BUFFER_SIZE, &max, &idxMax);
	arm_min_q15((q15_t*)array, BUFFER_SIZE, &min, &idxMin);
	return max-min;
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
void updateVoltage(uint16_t index) {
	uint16_t mask = 0xC000; //1100 0000 0000 0000
	WriteBits(GPIOB, mask, index);
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
	if (value >= 16383) {
		value = 16383;
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
	uint32_t amp = 819;
	uint32_t max = 16383;

	for (int i = 0; i < LUT_SIZE; i++) {
		float sine = sinf(i * (2.0 * M_PI / LUT_SIZE));
		sinewave[i] = (uint16_t)(amp * sine + max * 0.5);
//		sinewave[i] = (uint16_t)(max * 0.5 * ((amp / max) * sine + 1.0));
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


//---------- Write Port ----------
/*
 * Helper function for setting a multiplexer
 * value on a port without changing other states.
 *
 * Args:
 * 		GPIOx: Port that is being updated eg GPIOE
 * 		pinMask: The binary value of the pins that are
 * 			to be changed.
 * 		value: The binary number that is being written
 * 			to the pins defined in pinMask.
 */
void WriteBits(GPIO_TypeDef* GPIOx, uint16_t pinMask, uint8_t value) {
    // Clear the bits at the positions of pinMask
    GPIOx->ODR &= ~pinMask;

    // Set the new value at the positions of pinMask
    GPIOx->ODR |= (value & 0x0F) << __builtin_ctz(pinMask);
}

void setGain(uint16_t adc, uint8_t gain){
	if (adc == 1) {
		WriteBits(GPIOD, 0b0000001100000000, gain); //0000 0011 0000 0000
	}
	if (adc == 2) {
		WriteBits(GPIOD, 0b0000110000000000, gain); //0000 1100 0000 0000
	}
	if (adc == 3) {
		WriteBits(GPIOD, 0b0011000000000000, gain); //0011 0000 0000 0000
	}
	if (adc == 4) {
		WriteBits(GPIOD, 0b1100000000000000, gain); //1100 0000 0000 0000
	}
}

void checkStim(void) {
	for (size_t A = 1; A<5; A++) {
		int firstElec = 4 * (A - 1) + voltage_mux + 1;
		int secondElec = (firstElec < 16) ? (firstElec + 1) : 1;

		int firstStim = current_mux + 1;
		int secondStim = (firstStim < 16) ? (firstStim + 1) : 1;
		if (firstElec != firstStim && firstElec != secondStim && secondElec != firstStim) {
			setGain(A,0b0011);
		} else {
			setGain(A,0b0000);
		}
	}

}

//---------- Current Multiplexing ----------
/*
 * Sets the multiplexers for the positive and
 * negative current stimulation. A value of 0-15 is
 * passed to it defining what electrodes are written to.
 *
 * Args:
 * 		electrodes: The configuration that is being set.
 */
void updateCurrent(uint16_t electrodes) {
	uint16_t maskP = 0x003C; //0000 0000 0011 1100
	uint16_t maskN = 0x3C00; //0011 1100 0000 0000
	uint8_t pinsP = (uint8_t)electrodes; //1,2
	uint8_t pinsN = pinsP < 15 ? pinsP + 1 : 0;
	WriteBits(GPIOE, maskP, pinsP);
	WriteBits(GPIOE, maskN, pinsN);
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
