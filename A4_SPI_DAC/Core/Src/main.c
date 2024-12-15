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
#define UPPER_MASK 0x3000
#define CALIBRATION 1.0275
// header
void DAC_write(uint16_t);
static void Keypad_TurnOnCols(void);
static void Keypad_TurnOffCols(void);
void Keypad_Init(void);
int8_t Keypad_GetButton(void);
int8_t Keypad_CalculateButton(int8_t, int8_t);
void DAC_init(void);
uint16_t DAC_volt_conv(uint16_t);

void SystemClock_Config(void);

static void Keypad_TurnOnCols(void) {
	GPIOC->ODR |= GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12;
}

static void Keypad_TurnOffCols(void) {
	GPIOC->ODR &= ~(GPIO_ODR_OD10 | GPIO_ODR_OD11 | GPIO_ODR_OD12);
}

// Cols: PC10-12, Rows: PB13, 14, 15, 1
void Keypad_Init(void) {
	  // Turn on clock for GPIOB and GPIOC
	  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);

	  // Configure cols (PC10-12) for output
	  GPIOC->MODER &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11 | GPIO_MODER_MODE12);
	  GPIOC->MODER |= (1 << GPIO_MODER_MODE10_Pos) | (1 << GPIO_MODER_MODE11_Pos) | (1 << GPIO_MODER_MODE12_Pos);
	  GPIOC->OTYPER &= ~(GPIO_OTYPER_OT10 | GPIO_OTYPER_OT11 | GPIO_OTYPER_OT12);
	  GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEED10 | GPIO_OSPEEDR_OSPEED11 | GPIO_OSPEEDR_OSPEED12);
	  GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPD10 | GPIO_PUPDR_PUPD11 | GPIO_PUPDR_PUPD12);

	  // Configure rows (PB13-15,1) for input
	  GPIOB->MODER &= ~(GPIO_MODER_MODE13 | GPIO_MODER_MODE14 | GPIO_MODER_MODE15 | GPIO_MODER_MODE1);

	  // Use pull down resistors on rows.
	  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD13 | GPIO_PUPDR_PUPD14 | GPIO_PUPDR_PUPD15 | GPIO_PUPDR_PUPD1);
	  GPIOB->PUPDR |= (GPIO_PUPDR_PUPD13_1 | GPIO_PUPDR_PUPD14_1 | GPIO_PUPDR_PUPD15_1 | GPIO_PUPDR_PUPD1_1);

	  Keypad_TurnOnCols();
}

int8_t Keypad_GetButton(void) {
	// Return early if no buttons are pressed
	if (!(GPIOB->IDR & (GPIO_IDR_ID13 | GPIO_IDR_ID14 | GPIO_IDR_ID15 | GPIO_IDR_ID1))) {
		return -1;
	}
	for (int8_t col = 0; col <= 2; col++) {
		// Turn on only the specific column being tested
		Keypad_TurnOffCols();
		GPIOC->ODR |= 1 << (GPIO_ODR_OD10_Pos + col);
		// Check if any buttons are pressed in the column
		int8_t row = -1;
		uint32_t row_pins = GPIOB->IDR;
		if (row_pins & GPIO_IDR_ID13) row = 0;
		if (row_pins & GPIO_IDR_ID14) row = 1;
		if (row_pins & GPIO_IDR_ID15) row = 2;
		if (row_pins & GPIO_IDR_ID1) row = 3;
		// If none are pressed, keep going
		if (row == -1) {
			continue;
		}
		// Turn on all columns before returning
		Keypad_TurnOnCols();
		return Keypad_CalculateButton(row, col);
	}
	// Turn on all columns before returning
	Keypad_TurnOnCols();
	return -1;
}


int8_t Keypad_CalculateButton(int8_t row, int8_t col) {
	if (row <= 2) {
		return row * 3 + col + 1;
	}
	// Special cases for bottom row
	if (col == 0) return 10;
	if (col == 1) return 0;
	if (col == 2) return 11;
	return -1;
}

void DAC_init(void){
	// Write proper GPIO registers: Configure GPIO for MOSI, MISO and SCK pins
	RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN);

	//SPI clock
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE7);
	GPIOA->MODER |= (2 << GPIO_MODER_MODE4_Pos | 2 << GPIO_MODER_MODE5_Pos
					| 2 << GPIO_MODER_MODE7_Pos);
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 | GPIO_OTYPER_OT7);
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5 | GPIO_OSPEEDR_OSPEED7);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD7);

	// configure PA4/NSS, PA5/SCLK, PA7/MOSI for SPI1/AF5
	GPIOA->AFR[0] &= ~((0xF << GPIO_AFRL_AFSEL4_Pos)|(0xF << GPIO_AFRL_AFSEL5_Pos)| (0xF << GPIO_AFRL_AFSEL7_Pos)); // clear
	GPIOA->AFR[0] |= ((0x5 << GPIO_AFRL_AFSEL4_Pos)| (0x5 << GPIO_AFRL_AFSEL5_Pos)| (0x5 << GPIO_AFRL_AFSEL7_Pos)); // set


	// baud rate of Fpclk/2
	SPI1->CR1 &= ~SPI_CR1_BR;
	//clock in 00 mode
	SPI1->CR1 &= ~(SPI_CR1_CPOL | SPI_CR1_CPHA);
	// simplex mode, send data only
	SPI1->CR1 &= ~SPI_CR1_RXONLY;

	SPI1->CR1 |= SPI_CR1_BIDIOE;

	// MSB First
	SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);
	// Hardware manage the CS
	SPI1->CR1 &= ~(SPI_CR1_SSM);
	// Controller configuration
	SPI1->CR1 |= (SPI_CR1_MSTR);


	// 16 Bit data transfer
	SPI1->CR2 |= SPI_CR2_DS;
	// Disable hardware NSS management
	SPI1->CR2 |= SPI_CR2_SSOE;
	SPI1->CR2 |= SPI_CR2_NSSP_Msk;


	//disable interrupts
	SPI1->CR2 &= ~(SPI_CR2_TXEIE | SPI_CR2_RXNEIE);
	// FRXTH enabled since DS is 12
	SPI1->CR2 &= ~SPI_CR2_FRXTH;
	// enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;
}

uint16_t DAC_volt_conv(uint16_t num) {
	// calculate voltage in bits
	num *= (4096/330); //* CALIBRATION;

	if (num >= 4095) {
			num = 4095;
	}
	// clear upper nibble and set it
	num &= ~(0xF000);
	num |= UPPER_MASK;
	return num;
}

void DAC_write(uint16_t num) {

	SPI1->DR = num;
	// waiting till done transmitting w/ polling
	while (!(SPI1->SR & SPI_SR_TXE));
	// wait till not busy
	while (SPI1->SR & SPI_SR_BSY);
	// set CS back high to not send data
	//GPIOA->ODR |= GPIO_ODR_OD4;
}

void dumb_wait(void) {
	uint32_t total = 355340;
	while (total) {
		total--;
	}
	return;
}



int main(void)
{

  HAL_Init();
  DAC_init();
  Keypad_Init();
  SystemClock_Config();


  while (1)
  {

//	SPI1->DR = 0x137F;
//	while (!(SPI1->SR & SPI_SR_TXE));

	  static uint16_t count = 0;
	  static uint16_t user_val = 0;
	  static uint16_t num_conv = 0;

	  while (count < 3) {
		  int8_t button = Keypad_GetButton();
		  if (button != -1) {
			  count++;
			  user_val = user_val*10 + button;
			  dumb_wait();
		  }
		  DAC_write(num_conv);
	  }
	  if (count == 3) {
		  num_conv = DAC_volt_conv(user_val);
		  DAC_write(num_conv);
		  count = 0;
		  user_val = 0;
	  }
	  dumb_wait();

  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
