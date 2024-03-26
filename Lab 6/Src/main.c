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
#include <stm32f072xb.h>
void _Error_Handler(char * file, int line);

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
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

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

  // Configure PC0 to analog mode, no pull-up/down resistors
  GPIOC->MODER |= (3 << 0);
  GPIOC->PUPDR &= ~(3 << 0);

  // Enable the ADC clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

  // Set the ADC to 8-bit
  ADC1->CFGR1 &= ~(3 << 3);
  ADC1->CFGR1 |= (2 << 3);

  // Set continuous conversion mode
  ADC1->CFGR1 |= (1 << 13);

  // Hardware trigger disabled
  ADC1->CFGR1 &= ~(1 << 0);

  // PC0 is input channel ADC_IN10
  ADC1->CHSELR |= (1 << 10);

  // Begin the calibration
  ADC1->ISR |= (1 << 31); // Set the ADRDY bit to 1
  ADC1->CR |= (1 << 0); // Set the ADEN bit to 1

  // Wait for calibration to finish
  while ((ADC1->ISR & (1 << 0)) == 0) {
    // Wait for the ADRDY bit to be 1
  }

  // Start the ADC
  ADC1->CR |= (1 << 2);

  while(1){
    // Read the values of the ADC and turn on and off LEDs based on the value
    // Using a 100k potentiometer hooked up to PC0, operating in 8-bit mode
    // gives us a value between 0-255.

    if(ADC1->DR < 64){
      GPIOC->ODR |= (1 << 6);
      GPIOC->ODR &= ~(1 << 7);
      GPIOC->ODR &= ~(1 << 8);
      GPIOC->ODR &= ~(1 << 9);
    }
    else if(ADC1->DR < 128){
      GPIOC->ODR |= (1 << 6);
      GPIOC->ODR |= (1 << 7);
      GPIOC->ODR &= ~(1 << 8);
      GPIOC->ODR &= ~(1 << 9);
    }
    else if(ADC1->DR < 192){
      GPIOC->ODR |= (1 << 6);
      GPIOC->ODR |= (1 << 7);
      GPIOC->ODR |= (1 << 8);
      GPIOC->ODR &= ~(1 << 9);
    }
    else{
      GPIOC->ODR |= (1 << 6);
      GPIOC->ODR |= (1 << 7);
      GPIOC->ODR |= (1 << 8);
      GPIOC->ODR |= (1 << 9);
    }
  }










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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

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
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
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
