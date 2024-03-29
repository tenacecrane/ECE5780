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

  // Enable TIM3, PC6 (Red LED), PC7 (Blue LED)
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  // Set PC6 and PC7 to alternate function mode
  GPIOC->MODER &= ~(3 << 12);
  GPIOC->MODER |= (2 << 12);
  GPIOC->MODER &= ~(3 << 14);
  GPIOC->MODER |= (2 << 14);

  // Set PC6 and PC7 to low speed
  GPIOC->OSPEEDR &= ~(3 << 12);
  GPIOC->OSPEEDR &= ~(3 << 14);

  // Set PC6 and PC7 to no pull-up, no pull-down
  GPIOC->PUPDR &= ~(3 << 12);
  GPIOC->PUPDR &= ~(3 << 14);

  // Target Frequency = Clock Frequency / PSC * ARR
  // Target Frequency = 800Hz
  // Clock Frequency = 8MHz

  // Set TIM3 PSC to 999 to get 8kHz
  TIM3->PSC = 999;

  // Set TIM3 ARR to 10 to get 800Hz
  TIM3->ARR = 10;

  // Set CC1S and CC2S to 00 to set the channels as output
  TIM3->CCMR1 &= ~(3 << 0);
  TIM3->CCMR1 &= ~(3 << 8);

  // Set OC1M to 111 to set to PWM mode 2
  TIM3->CCMR1 |= (7 << 4);
  // Set OC2M to 110 to set to PWM mode 1
  TIM3->CCMR1 |= (6 << 12);

  // Set OC1PE and OC2PE to 1 to enable preload
  TIM3->CCMR1 |= (1 << 3);
  TIM3->CCMR1 |= (1 << 11);

  // Set the output enable bits for channel 1 and 2 in the CCER register
  TIM3->CCER |= TIM_CCER_CC1E;
  TIM3->CCER |= TIM_CCER_CC2E;

  // Set the capture/compare registers (CCRx) for both channels to 20% of ARR value.
  TIM3->CCR1 = 2;
  TIM3->CCR2 = 2;

  // Set PC6 and PC7 to AF0 to use TIM3 for PWM output
  GPIOC->AFR[0] &= ~(0xF << 24); // Clear AFRL6
  GPIOC->AFR[0] |= (0 << 24); // Set AFRL6 to AF0
  GPIOC->AFR[0] &= ~(0xF << 28); // Clear AFRL7
  GPIOC->AFR[0] |= (0 << 28); // Set AFRL7 to AF0

  // Enable the counter
  TIM3->CR1 |= TIM_CR1_CEN;

  while(1) {

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
