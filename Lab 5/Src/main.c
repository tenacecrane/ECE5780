/**
 *
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"
void _Error_Handler(char *file, int line);

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  SystemClock_Config();

  // Enable GPIOB and GPOIC
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

  // Enable I2C Clock
  RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

  // Enable LED's 6-9
  // Clear the bits for PC6-9
  GPIOC->MODER &= ~(3 << 12);
  GPIOC->MODER &= ~(3 << 14);
  GPIOC->MODER &= ~(3 << 16);
  GPIOC->MODER &= ~(3 << 18);
  // Setting PC6-9 to General-Purpose Output Mode
  GPIOC->MODER |= (1 << 12);
  GPIOC->MODER |= (1 << 14);
  GPIOC->MODER |= (1 << 16);
  GPIOC->MODER |= (1 << 18);
  // Setting PC6-9 to Push-Pull Output Type
  GPIOC->OTYPER &= ~(1 << 6);
  GPIOC->OTYPER &= ~(1 << 7);
  GPIOC->OTYPER &= ~(1 << 8);
  GPIOC->OTYPER &= ~(1 << 9);
  // Set PC6-9 to Low Speed
  GPIOC->OSPEEDR &= ~(0 << 12);
  GPIOC->OSPEEDR &= ~(0 << 14);
  GPIOC->OSPEEDR &= ~(0 << 16);
  GPIOC->OSPEEDR &= ~(0 << 18);
  // Set the pull-up/pull-down resistors to no pull-up/pull-down since the bits are 00
  GPIOC->PUPDR &= ~(3 << 12);
  GPIOC->PUPDR &= ~(3 << 14);
  GPIOC->PUPDR &= ~(3 << 16);
  GPIOC->PUPDR &= ~(3 << 18);

  // Set PB11 to AF1 (I2C2_SDA), open-drain mode
  GPIOB->MODER |= (1 << 23);
  GPIOB->OTYPER |= (1 << 11);
  GPIOB->AFR[1] |= (1 << 12);
  GPIOB->AFR[1] &= ~(1 << 13) | ~(1 << 14) | ~(1 << 15);

  // Set PB13 to AF5, open-drain mode
  GPIOB->MODER &= ~(1 << 26);
  GPIOB->MODER |= (1 << 27);
  GPIOB->OTYPER |= (1 << 13);
  GPIOB->AFR[1] &= ~(1 << 21) | ~(1 << 23);
  GPIOB->AFR[1] |= (1 << 20) | (1 << 22);

  // Set PB14 to output mode, push-pull, and set high
  GPIOB->MODER |= (1 << 28);
  GPIOB->MODER &= ~(1 << 29);
  GPIOB->OTYPER &= ~(1 << 14);
  GPIOB->ODR |= (1 << 14);

  // Set PC0 to output mode, push-pull, and set high
  GPIOC->MODER &= ~(1 << 1);
  GPIOC->MODER |= (1 << 0);
  GPIOC->OTYPER &= ~(1 << 0);
  GPIOC->ODR |= (1 << 0);

  // Set the I2C2 clock to 100 kHz
  I2C2->TIMINGR = (0x1 << 28) | (0x13 << 0) | (0xF << 8) | (0x2 << 16) | (0x4 << 20);

  // Enable I2C2
  I2C2->CR1 |= I2C_CR1_PE;

  // Set PC6 (red) to high to signal I2C start
  GPIOC->ODR |= (1 << 6);

  // Set I2C2_CR2 SADD to 0x69 (slave address set)
  I2C2->CR2 |= (1 << 16) | (0x69 << 1);

  // Set RD_WRN to WRN
  I2C2->CR2 &= ~I2C_CR2_RD_WRN;

  // Set the START bit
  I2C2->CR2 |= (1 << 13);

  while (1)
  { // Check for TXIS flag (bit 1)
    if ((I2C2->ISR & (1 << 1)))
    {
      // Slave did not respond to the address frame
      // Just continue
      // Set PC6 (red) to low to signal I2C stop
      GPIOC->ODR &= ~(1 << 6);
      break;
    }
    // Check for NACKF flag
    else if ((I2C2->ISR & (1 << 4)))
    {

      // Wiring or configuration error
    }
  }
  // Send the WHO_AM_I register address
  I2C2->TXDR = 0x0F;

  // Wait for the TC flag to be set
  while (!(I2C2->ISR & (1 << 6)))
    ;

  // Reset CR2
  // Set CR2 to 0
  I2C2->CR2 = 0;

  // Set I2C2_CR2 SADD to 0011010010 (0x69) (slave address set)
  I2C2->CR2 |= (1 << 16) | (0x69 << 1);

  // Set the RD_WRN bit to 1 (read)
  I2C2->CR2 |= I2C_CR2_RD_WRN;

  // Set the START bit
  I2C2->CR2 |= (1 << 13);

  // Wait until the RXNE or NACKF flag is set
  while (1)
  {
    // If NACKF flag is set
    if (I2C2->ISR & (1 << 4))
    {
      // Slave did not respond to the address frame
      // Just continue
      // Set PC6 (red) to low to signal I2C stop
      GPIOC->ODR &= ~(1 << 6);
    }
    // If RXNE flag is set
    else if (I2C2->ISR & (1 << 2))
    {
      break;
    }
  }
  // Set PC7 to high to confirm that the WHO_AM_I register is being read correctly
  if (I2C2->RXDR == 0xD3)
  {
    GPIOC->ODR |= (1 << 7);
    // Send the stop bit
    I2C2->CR2 |= (1 << 14);
  }
  while (1);
  
}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure the Systick interrupt time
   */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
   */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
