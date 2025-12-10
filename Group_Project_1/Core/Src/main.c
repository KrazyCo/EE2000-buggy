/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
#include "task.h"
#include "line_following.h"
#include <stdio.h>
#include <string.h>
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for lineFollowing */
osThreadId_t lineFollowingHandle;
const osThreadAttr_t lineFollowing_attributes = { .name = "lineFollowing",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityHigh, };
/* Definitions for buttonPoll */
osThreadId_t buttonPollHandle;
const osThreadAttr_t buttonPoll_attributes =
		{ .name = "buttonPoll", .stack_size = 256 * 4, .priority =
				(osPriority_t) osPriorityBelowNormal, };
/* Definitions for ultrasonicLoop */
osThreadId_t ultrasonicLoopHandle;
const osThreadAttr_t ultrasonicLoop_attributes = { .name = "ultrasonicLoop",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for sponsors */
osThreadId_t sponsorsHandle;
const osThreadAttr_t sponsors_attributes = { .name = "sponsors", .stack_size =
		128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* USER CODE BEGIN PV */

// State flags
uint8_t show_logo = 1;
uint8_t moving_forward = 0;
bool object_in_path = false;

// Button debounce
#define DEBOUNCE_MS 200
uint32_t btn1_last = 0;
uint32_t btn2_last = 0;
uint32_t btn3_last = 0;
uint32_t btn4_last = 0;

#define LOGO_LED_GPIO_PORT   GPIOB
#define LOGO_LED_PIN         GPIO_PIN_6   // D10

void blink_logo_led(uint8_t times);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
void lineFollowingTask(void *argument);
void buttonPollTask(void *argument);
void ultrasonicTask(void *argument);
void sponsorsTask(void *argument);

/* USER CODE BEGIN PFP */
void delay(uint16_t time);
void HCSR04_Read(void);

void display_menu(void);
void display_logo(void);
void display_lap_times(void);
void blink_logo_led(uint8_t times);
void poll_buttons(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay(uint16_t time) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while (__HAL_TIM_GET_COUNTER(&htim1) < time)
		;
}

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;
uint16_t Distance = 0;
uint16_t Last_Valid_Distance = 0;
uint8_t sponsor_index = 0;
bool is_sponsor_loop = false;
int sponsor_loop_index = 0;
bool lap_display = false;

#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA

// Let's write the callback function

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // if the interrupt source is channel1
			{
		if (Is_First_Captured == 0) // if the first value is not captured
				{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured == 1)   // if the first is already captured
				{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1) {
				Difference = IC_Val2 - IC_Val1;
			}

			else if (IC_Val1 > IC_Val2) {
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * 0.034 / 2;

//			if (newDistance <= 50) {
//				// Accept the reading
//				Distance = newDistance;
//				Last_Valid_Distance = newDistance;
//			} else {
//				// Ignore bad values, keep last good one
//				Distance = Last_Valid_Distance;
//			}

			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}

void HCSR04_Read(void) {
	taskENTER_CRITICAL(); // disable interrupts for timing
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET); // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET); // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
	taskEXIT_CRITICAL(); // enable interrupts
}

char string[20];
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	SSD1306_Init();
	display_menu();
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of lineFollowing */
	lineFollowingHandle = osThreadNew(lineFollowingTask, NULL,
			&lineFollowing_attributes);

	/* creation of buttonPoll */
	buttonPollHandle = osThreadNew(buttonPollTask, NULL,
			&buttonPoll_attributes);

	/* creation of ultrasonicLoop */
	ultrasonicLoopHandle = osThreadNew(ultrasonicTask, NULL,
			&ultrasonicLoop_attributes);

	/* creation of sponsors */
	sponsorsHandle = osThreadNew(sponsorsTask, NULL, &sponsors_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2
			| RCC_PERIPHCLK_I2C1;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x0010020A;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 47;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 47;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 999;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | Trig_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
	Motor_Output_Pin | Motor_OutputB5_Pin | LED_D10_PB6_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Btn1_A0_Pin Btn2_A1_Pin Btn3_A2_Pin */
	GPIO_InitStruct.Pin = Btn1_A0_Pin | Btn2_A1_Pin | Btn3_A2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin Trig_Pin */
	GPIO_InitStruct.Pin = LD2_Pin | Trig_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : Btn4_A3_Pin */
	GPIO_InitStruct.Pin = Btn4_A3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(Btn4_A3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Middle_sensor_Pin Left_sensor_Pin */
	GPIO_InitStruct.Pin = Middle_sensor_Pin | Left_sensor_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : Right_sensor_Pin */
	GPIO_InitStruct.Pin = Right_sensor_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Right_sensor_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Motor_Output_Pin Motor_OutputB5_Pin LED_D10_PB6_Pin */
	GPIO_InitStruct.Pin = Motor_Output_Pin | Motor_OutputB5_Pin
			| LED_D10_PB6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void display_menu(void) {
	SSD1306_Clear();

	SSD1306_GotoXY(5, 0);
	SSD1306_Puts("B1: Sponsors", &Font_7x10, 1);

	SSD1306_GotoXY(5, 12);
	SSD1306_Puts("B2: Lap Times", &Font_7x10, 1);

	SSD1306_GotoXY(5, 24);
	SSD1306_Puts("B3: Line Following", &Font_7x10, 1);

	SSD1306_GotoXY(5, 36);
	SSD1306_Puts("B4: Main Menu", &Font_7x10, 1);

	SSD1306_UpdateScreen();
}

const char *sponsors[3] = { "DMC Motors", "Libyan Plutonium",
		"BiffCo Enterprise" };

// DMC logo
void draw_dmc_logo(void) {
	// Circle
	SSD1306_DrawCircle(64, 24, 14, SSD1306_COLOR_WHITE);

	// Inverted DMC text (white block, black text)
	SSD1306_DrawFilledRectangle(45, 18, 38, 12, SSD1306_COLOR_WHITE);

	SSD1306_GotoXY(54, 20);
	SSD1306_Puts("DMC", &Font_7x10, SSD1306_COLOR_BLACK);
}

// Flux Capacitor Graphic
void draw_flux_capacitor(void) {
	int cx = 64;
	int cy = 22;
	int radius = 5;

	int topX = cx;
	int topY = cy - 14;

	int leftX = cx - 18;
	int leftY = cy + 10;

	int rightX = cx + 18;
	int rightY = cy + 10;

	SSD1306_DrawCircle(topX, topY, radius, SSD1306_COLOR_WHITE);
	SSD1306_DrawCircle(leftX, leftY, radius, SSD1306_COLOR_WHITE);
	SSD1306_DrawCircle(rightX, rightY, radius, SSD1306_COLOR_WHITE);

	SSD1306_DrawLine(topX, topY + radius, cx, cy, SSD1306_COLOR_WHITE);
	SSD1306_DrawLine(leftX + radius, leftY - radius, cx, cy,
			SSD1306_COLOR_WHITE);
	SSD1306_DrawLine(rightX - radius, rightY - radius, cx, cy,
			SSD1306_COLOR_WHITE);
}

//BiffCo logo
void draw_biffco_logo(void) {

	// Outer thick rectangle (smaller height: 6 → 30)
	SSD1306_DrawRectangle(34, 6, 60, 30, SSD1306_COLOR_WHITE);
	SSD1306_DrawRectangle(36, 8, 56, 26, SSD1306_COLOR_WHITE);

	// Diagonal slash (adjusted to fit inside shorter box)
	SSD1306_DrawLine(34, 6, 94, 30, SSD1306_COLOR_WHITE);
	SSD1306_DrawLine(36, 8, 92, 28, SSD1306_COLOR_WHITE);

	// Big center B (moved upward so it stays inside box)
	SSD1306_GotoXY(58, 12);
	SSD1306_Puts("B", &Font_11x18, SSD1306_COLOR_WHITE);
}

void display_logo(void) {
	SSD1306_Clear();

	// --- Draw correct logo based on sponsor ---
	if (sponsor_index == 0) {
		draw_dmc_logo();
	} else if (sponsor_index == 1) {
		draw_flux_capacitor();
	} else if (sponsor_index == 2) {
		draw_biffco_logo();
	}

	SSD1306_GotoXY(10, 40);
	SSD1306_Puts("Sponsored by:", &Font_7x10, SSD1306_COLOR_WHITE);

	// Sponsor name (bottom)
	SSD1306_GotoXY(10, 52);
	SSD1306_Puts(sponsors[sponsor_index], &Font_7x10, SSD1306_COLOR_WHITE);

	SSD1306_UpdateScreen();
}

void display_lap_times(void) {
	SSD1306_Clear();
	char buf[32];

	// ---------- LAP 1 ----------
	snprintf(buf, sizeof(buf), "Lap1: 12.3 sec");     // <-- change here
	SSD1306_GotoXY(5, 0);
	SSD1306_Puts(buf, &Font_7x10, 1);

	snprintf(buf, sizeof(buf), "Avg Speed:0.25m/s");  // <-- and here
	SSD1306_GotoXY(5, 10);
	SSD1306_Puts(buf, &Font_7x10, 1);

	// ---------- LAP 2 ----------
	snprintf(buf, sizeof(buf), "Lap2: 15.8 sec");     // <-- change here
	SSD1306_GotoXY(5, 20);
	SSD1306_Puts(buf, &Font_7x10, 1);

	snprintf(buf, sizeof(buf), "Avg Speed:0.30m/s");  // <-- and here
	SSD1306_GotoXY(5, 30);
	SSD1306_Puts(buf, &Font_7x10, 1);

	// ---------- LAP 3 ----------
	snprintf(buf, sizeof(buf), "Lap3: 9.6 sec");      // <-- change here
	SSD1306_GotoXY(5, 40);
	SSD1306_Puts(buf, &Font_7x10, 1);

	snprintf(buf, sizeof(buf), "Avg Speed:0.40m/s");  // <-- and here
	SSD1306_GotoXY(5, 50);
	SSD1306_Puts(buf, &Font_7x10, 1);

	SSD1306_UpdateScreen();
}

void display_lap_finished(void) {
    SSD1306_Clear();

    SSD1306_GotoXY(5, 4);
    SSD1306_Puts("88 MPH REACHED!", &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_GotoXY(5, 26);
    SSD1306_Puts("1.21 GIGAWATTS", &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_GotoXY(20, 38);
    SSD1306_Puts("ACHIEVED", &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_GotoXY(22, 52);
    SSD1306_Puts("LAP FINISHED!", &Font_7x10, SSD1306_COLOR_WHITE);

    SSD1306_UpdateScreen();
}

void blink_logo_led(uint8_t times) {
	for (uint8_t i = 0; i < times; i++) {
		HAL_GPIO_WritePin(LOGO_LED_GPIO_PORT, LOGO_LED_PIN, GPIO_PIN_SET);
		osDelay(500); // LED ON  500 ms

		HAL_GPIO_WritePin(LOGO_LED_GPIO_PORT, LOGO_LED_PIN, GPIO_PIN_RESET);
		osDelay(250); // LED OFF 250 ms
	}
}

void poll_buttons(void) {
	static uint8_t btn1_prev = 0;
	static uint8_t btn2_prev = 0;
	static uint8_t btn3_prev = 0;
	static uint8_t btn4_prev = 0;
	uint32_t t = HAL_GetTick();

	uint8_t btn1_now = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
	uint8_t btn2_now = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
	uint8_t btn3_now = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	uint8_t btn4_now = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);

	// Button 1: Display company logo
	if (btn1_now == GPIO_PIN_SET && btn1_prev == 0
			&& (t - btn1_last > DEBOUNCE_MS)) {

		is_sponsor_loop = false; // we dont want to loop through
		// move to next sponsor
		sponsor_index = (sponsor_index + 1) % 3;
		display_logo();

		blink_logo_led(2);  // shorter blink for sponsor switch
		btn1_last = t;
	}

	btn1_prev = btn1_now;

	// Button 2: Display lap times
	if (btn2_now == GPIO_PIN_SET && btn2_prev == 0
			&& (t - btn2_last > DEBOUNCE_MS)) {
		is_sponsor_loop = false;
		display_lap_times();
		btn2_last = t;

	}
	btn2_prev = btn2_now;

	// Button 3: Display logo and move forward
	if (btn3_now == GPIO_PIN_SET && btn3_prev == 0
			&& (t - btn3_last > DEBOUNCE_MS)) {
		if (moving_forward) {
			moving_forward = 0;
		} else {
			moving_forward = 1;
		}
//		moving_forward = moving_forward==1 ? 0 : 1;

		is_sponsor_loop = true;

		btn3_last = t;
	}
	btn3_prev = btn3_now;

	// Button 4: Return to main menu
	if (btn4_now == GPIO_PIN_SET && btn4_prev == 0
			&& (t - btn4_last > DEBOUNCE_MS)) {
		is_sponsor_loop = false;
		display_menu();   // redraw the selection screen
		btn4_last = t;
	}
	btn4_prev = btn4_now;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_lineFollowingTask */
/**
 * @brief  Function implementing the lineFollowing thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_lineFollowingTask */
void lineFollowingTask(void *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		line_following_loop(&htim3);
		osDelay(10);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_buttonPollTask */
/**
 * @brief Function implementing the buttonPoll thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_buttonPollTask */
void buttonPollTask(void *argument) {
	/* USER CODE BEGIN buttonPollTask */
	/* Infinite loop */
	for (;;) {
		poll_buttons();
		osDelay(50);
	}
	/* USER CODE END buttonPollTask */
}

/* USER CODE BEGIN Header_ultrasonicTask */
/**
 * @brief Function implementing the ultrasonicLoop thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ultrasonicTask */
void ultrasonicTask(void *argument) {
	/* USER CODE BEGIN ultrasonicTask */
	/* Infinite loop */
	for (;;) {
		HCSR04_Read();
		osDelay(200);
		if (Distance < 15) {
			object_in_path = true;
		} else {
			object_in_path = false;
		}
	}
	/* USER CODE END ultrasonicTask */
}

/* USER CODE BEGIN Header_sponsorsTask */
/**
 * @brief Function implementing the sponsors thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_sponsorsTask */
void sponsorsTask(void *argument) {
	/* USER CODE BEGIN sponsorsTask */
	/* Infinite loop */
	for (;;) {
		if (lap_display) {

		} else if (is_sponsor_loop) {
			if (sponsor_loop_index == 0) {
				display_logo();
				// move to next sponsor
				sponsor_index = (sponsor_index + 1) % 3;
			}
			if (sponsor_loop_index >= 50) {
				sponsor_loop_index = 0;
			} else {
				sponsor_loop_index += 1;
			}
		}
		osDelay(100);
	}
	/* USER CODE END sponsorsTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
