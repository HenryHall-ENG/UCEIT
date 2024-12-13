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
#define BUFFER_SIZE 25
#define MEAS_SIZE 256
#define LUT_SIZE 100 //Size of the stimulation Look Up Table
#define USB_PAYLOAD 8 //Size of the USB Buffer

#define TIMER2_PRESCALAR 1 //Prescalar for TIM2
#define TIMER2_FREQ 1e5 //Desired Frequency for TIM2


#define DAC_FREQ 1e4 //Desired Frequency of the Stimulation waveform
#define SAMPLING_FREQUENCY 1e5


#define MAIN_FREQ 2 //Update frequency of the Main program
#define USB_FREQ 2000 //Frequency of the USB communication

#define CURRENT_FREQ 500
#define VOLTAGE_FREQ 50


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

void send_all_buffer(void);
void updateVoltage(uint16_t index);
void send_adc_buffer(uint16_t *array, size_t size, uint8_t adc);
void write_dac(uint16_t value);
void init_lut(void);
void updateCurrent(uint16_t electrodes);
void USB_Send(char* message);
void WriteBits(GPIO_TypeDef* GPIOx, uint16_t pinMask, uint8_t value);
uint16_t calcMag(uint16_t * array);
void clearAllBuffers(void);
void checkStim(void);
void setGain(uint16_t adc, uint8_t gain);
void calcMagnitude(void);
void sendMagnitude(uint16_t *array);

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


uint16_t adc1Buff[BUFFER_SIZE];
uint16_t adc2Buff[BUFFER_SIZE];
uint16_t adc3Buff[BUFFER_SIZE];
uint16_t adc4Buff[BUFFER_SIZE];
uint16_t adc5Buff[BUFFER_SIZE];
uint16_t magnitude[MEAS_SIZE] = {0};

float32_t omega;
float32_t cosine;
float32_t sine;
float32_t coeff;
float32_t q1, q2, q0;

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



  omega = 2.0f * PI * DAC_FREQ / SAMPLING_FREQUENCY;
  cosine = arm_cos_f32(omega);
  sine = arm_sin_f32(omega);
  coeff = 2.0f * cosine;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if (is_main) { // Toggles a status LED
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_2);
		is_main = 0;
	}
	if (is_usb) { //Sends all data and triggers a MUX change
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
//		send_all_buffer();
//				clearAllBuffers();
		is_usb = 0;
	  }
	if (is_voltage_mux) { //Updates Voltage MUX and triggers a MUX change
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
		calcMagnitude();
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_1);
		voltage_mux++;
		if (voltage_mux > 3) {
			voltage_mux = 0;
			current_mux++;
			if (current_mux > 15) {
				  current_mux = 0;
				  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0);
				  sendMagnitude(magnitude);
				  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0);
				}
		}
		updateVoltage(voltage_mux);
		updateCurrent(current_mux);
		checkStim();
		is_voltage_mux = 0;
	  }
//	  if (is_current_mux) { //Updates Current MUX
////		  send_all_buffer();
////		  current_mux++;
////		  if (current_mux > 15) {
////			  current_mux = 0;
////		  }
////
////		  updateCurrent(current_mux);
////		  checkStim();
////		  clearAllBuffers();
////		  is_current_mux = 0;
//	  }

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



//---------- ADC Callback ----------
/*
 * Reads all ADC values into the appropriate double buffer.
 *
 * Args:
 * 		hadc: Handle type for the adc, defines which
 * 			adc is being used
 */
//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
////	char buffer[USB_PAYLOAD + 1];
//    if (hadc->Instance == ADC1) {
////    	HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_0);
//    	uint32_t adc1_val = HAL_ADC_GetValue(hadc);
//    	writeDbleBuf (&buffers.adc1, adc1_val);
//    }
//    if (hadc->Instance == ADC2) {
//    	uint32_t adc2_val = HAL_ADC_GetValue(hadc);
//    	writeDbleBuf (&buffers.adc2, adc2_val);
//    }
//    if (hadc->Instance == ADC3) {
//    	uint32_t adc3_val = HAL_ADC_GetValue(hadc);
//    	writeDbleBuf (&buffers.adc3, adc3_val);
//    }
//    if (hadc->Instance == ADC4) {
//    	uint32_t adc4_val = HAL_ADC_GetValue(hadc);
//    	writeDbleBuf (&buffers.adc4, adc4_val);
//    }
//    if (hadc->Instance == ADC5) {
//    	uint32_t adc5_val = HAL_ADC_GetValue(hadc);
//    	writeDbleBuf (&buffers.adc5, adc5_val);
//    }
//}

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
//	    if (gl_ticks % (uint64_t)(TIMER2_FREQ / USB_FREQ) == 1) {
//	    	is_usb = 1;
//	    }
	    if (gl_ticks % (uint64_t)(TIMER2_FREQ / VOLTAGE_FREQ) == 1) {
	    	is_voltage_mux = 1;
	    }
//	    if (gl_ticks % (uint64_t)(TIMER2_FREQ / CURRENT_FREQ) == 1) {
//	    	is_current_mux = 1;
//	    }
//        if (gl_ticks >= TIMER2_FREQ) {
//            gl_ticks = 0;
//        }
	}

}

//---------- HRTIM Callback ----------
/*
 * Calls when the timer reaches its set period.
 * This triggers a new value to be sent to the DAC
 * from the predefined LUT.
 *
 * Args:
 * 		hhrtim: Handle type for the timer to define
 * 			which timer has triggered the callback.
 */
//void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef * hhrtim,
//                                              uint32_t TimerIdx)
//{
//	if (hhrtim->Instance == HRTIM1) {
//
////		write_dac(sinewave[sine_idx]);
////		sine_idx++;
////		if (sine_idx > (LUT_SIZE-1)) {
////			sine_idx=0;
////		}
//	}
//}

void clearAllBuffers(void) {
	memset(adc1Buff, 0, BUFFER_SIZE * sizeof(uint16_t));
	memset(adc2Buff, 0, BUFFER_SIZE * sizeof(uint16_t));
	memset(adc3Buff, 0, BUFFER_SIZE * sizeof(uint16_t));
	memset(adc4Buff, 0, BUFFER_SIZE * sizeof(uint16_t));
	memset(adc5Buff, 0, BUFFER_SIZE * sizeof(uint16_t));
}


//----------Buffer Send ----------
/*
 * Sends an array to the host PC with a
 * unique ID based on multiplexing state
 * and ADC used.
 *
 * Args:
 * 		array: Data to be sent to the host PC
 * 		size: Length of the array being sent
 * 		mux: Multiplexing value for the voltage
 * 			reading system
* 		adc: The ADC unit that the buffer is associated with.
 */
//void sendMagnitude(uint16_t *array) {
//	char buffer[USB_PAYLOAD];
//
//	for (size_t i = 0; i < MEAS_SIZE; i++) {
//		snprintf(buffer, sizeof(buffer), "%u\r\n", array[i]);
//		CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
//	}
//	snprintf(buffer, sizeof(buffer), "X\r\n");
//	CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
//}

//void sendMagnitude(uint16_t *array) {
//    char buffer[2048];  // Large enough buffer to hold entire transmission
//    char *ptr = buffer;
//    size_t remaining = sizeof(buffer);
//
//    // Convert entire array to string
//    for (size_t i = 0; i < MEAS_SIZE; i++) {
//        int written = snprintf(ptr, remaining, "%u\r\n", array[i]);
//        ptr += written;
//        remaining -= written;
//    }
//
//    // Append end marker
//    snprintf(ptr, remaining, "X\r\n");
//
//    // Send entire buffer in one transmission
//    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer));
//}

void sendMagnitude(uint16_t *array) {
	char buffer[2048];  // Large enough buffer to hold entire transmission
	char *ptr = buffer;
	size_t remaining = sizeof(buffer);

	// Convert entire array to string
	for (size_t i = 0; i < MEAS_SIZE; i++) {
		int written = snprintf(ptr, remaining, "%u\r\n", array[i]);
		ptr += written;
		remaining -= written;
	}

	// Append end marker
	snprintf(ptr, remaining, "X\r\n");



//    for (size_t total_sent = 0; total_sent <= 2048; total_sent += 64) {
//    	CDC_Transmit_FS((uint8_t*)buffer + total_sent, 64);
//	}
    CDC_Transmit_FS((uint8_t*)buffer, strlen(buffer)); // Send only the used portion
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

//	q1 = 0.0f;
//	q2 = 0.0f;
//
//	// Apply Goertzel filter using CMSIS
//	for (size_t i = 0; i < BUFFER_SIZE; i++) {
//		q0 = coeff * q1 - q2 + (float32_t)(array[i] - 2048);
//		q2 = q1;
//		q1 = q0;
//	}
//
//	// Compute real and imaginary components
//	float32_t real = q1 - q2 * cosine;
//	float32_t imag = q2 * sine;
//
//	// Compute magnitude using CMSIS
//	float32_t magnitude;
//	arm_sqrt_f32(real * real + imag * imag, &magnitude);
//
//	return (uint16_t)magnitude;
}

//---------- Data Send ----------
/*
 *Reads all the data from the non-active
 *window in the buffer and calls to the send
 *data function.
 *
 * Args:
 * 		buff: Buffer structure that is being
 * 			sent.
 */
void send_all_buffer() {
	uint8_t buffer[12];

//	uint16_t value1 = calcMag(adc1Buff,BUFFER_SIZE);
//	uint16_t value2 = calcMag(adc1Buff,BUFFER_SIZE);
//	uint16_t value3 = calcMag(adc1Buff,BUFFER_SIZE);
//	uint16_t value4 = calcMag(adc1Buff,BUFFER_SIZE);
//
//    buffer[0] = (uint8_t)(value1 >> 8);
//    buffer[1] = (uint8_t)(value1 & 0xFF);
//    buffer[2] = (uint8_t)(value2 >> 8);
//    buffer[3] = (uint8_t)(value2 & 0xFF);
//    buffer[4] = (uint8_t)(value3 >> 8);
//    buffer[5] = (uint8_t)(value3 & 0xFF);
//    buffer[6] = (uint8_t)(value4 >> 8);
//    buffer[7] = (uint8_t)(value4 & 0xFF);
//    buffer[8] = (uint8_t)(current_mux >> 8);
//    buffer[9] = (uint8_t)(current_mux & 0xFF);
//    buffer[10] = (uint8_t)(voltage_mux >> 8);
//    buffer[11] = (uint8_t)(voltage_mux & 0xFF);
//    CDC_Transmit_FS(buffer, 12);

//	send_adc_buffer(adc1Buff, BUFFER_SIZE, 1);
//	send_adc_buffer(adc2Buff, BUFFER_SIZE, 2);
//	send_adc_buffer(adc3Buff, BUFFER_SIZE, 3);
//	send_adc_buffer(adc4Buff, BUFFER_SIZE, 4);
//	send_adc_buffer(adc5Buff, BUFFER_SIZE, 5);
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
