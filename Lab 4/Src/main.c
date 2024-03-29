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

int charCounter = 0;
char ledColor;

void send_char(char c)
{
  // Use an empty while loop which exits once the flag is set.
  while (!(USART3->ISR & USART_ISR_TXE))
  {
  };
  // Write the character into the transmit data register
  USART3->TDR = c;
  return;
}

// Loop through an array of characters, send each one, exit when the null character is reached.
void transmit_string(char str[])
{
  for (int i = 0; str[i] != '\0'; i++)
  {
    send_char(str[i]);
  }
  return;
}

// Blank USART3 interrupt handler
void USART3_4_IRQHandler(void)
{
  // Check if the interrupt was triggered by a received character
  if (USART3->ISR & USART_ISR_RXNE)
  {
    char c = USART3->RDR;
    transmit_string("LED?");
    if (charCounter == 0)
      switch (c)
      {
      case 'r':
        charCounter++;
        ledColor = 'r';
        break;
      case 'b':
        charCounter++;
        ledColor = 'b';
        break;
      case 'o':
        charCounter++;
        ledColor = 'o';
        break;
      case 'g':
        charCounter++;
        ledColor = 'g';
        break;
      default:
        transmit_string("Invalid character\n");
      }
    else
    {
      transmit_string("CMD?");
      charCounter = 0;
      switch (c)
      {
      // Turn off the LED
      case '0':
        if (ledColor == 'r')
          GPIOC->ODR &= ~(1 << 6);
        else if (ledColor == 'b')
          GPIOC->ODR &= ~(1 << 7);
        else if (ledColor == 'o')
          GPIOC->ODR &= ~(1 << 8);
        else if (ledColor == 'g')
          GPIOC->ODR &= ~(1 << 9);
        break;
      // Turn on the LED
      case '1':
        if (ledColor == 'r')
          GPIOC->ODR |= (1 << 6);
        else if (ledColor == 'b')
          GPIOC->ODR |= (1 << 7);
        else if (ledColor == 'o')
          GPIOC->ODR |= (1 << 8);
        else if (ledColor == 'g')
          GPIOC->ODR |= (1 << 9);
        break;
      // Toggle the LED
      case '2':
        if (ledColor == 'r')
          GPIOC->ODR ^= (1 << 6);
        else if (ledColor == 'b')
          GPIOC->ODR ^= (1 << 7);
        else if (ledColor == 'o')
          GPIOC->ODR ^= (1 << 8);
        else if (ledColor == 'g')
          GPIOC->ODR ^= (1 << 9);
        break;
      default:
        transmit_string("Invalid character\n");
      }
      ledColor = ' ';
    }
  }


  return;
}

int main(void)
{
  SystemClock_Config();

  // Enable GPIOB clock
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

  // Enable GPIOC clock
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

  // Set PB10 and PB11 to alternate function mode.
  GPIOB->MODER &= ~(3 << 20);
  GPIOB->MODER |= 2 << 20;
  GPIOB->MODER &= ~(3 << 22);
  GPIOB->MODER |= 2 << 22;

  // Set PB10 and PB11 in AFRH AF4 mode (PB10 -> TX, PB11 -> RX) (USART3)
  GPIOB->AFR[1] &= ~(0xF << 8);
  GPIOB->AFR[1] |= 4 << 8;
  GPIOB->AFR[1] &= ~(0xF << 12);
  GPIOB->AFR[1] |= 4 << 12;

  // Enable the red PC6, PC7, PC8, and PC9 to general purpose output mode
  GPIOC->MODER &= ~(3 << 12);
  GPIOC->MODER |= 1 << 12;
  GPIOC->MODER &= ~(3 << 14);
  GPIOC->MODER |= 1 << 14;
  GPIOC->MODER &= ~(3 << 16);
  GPIOC->MODER |= 1 << 16;
  GPIOC->MODER &= ~(3 << 18);
  GPIOC->MODER |= 1 << 18;

  // Set PC6, PC7, PC8, and PC9 to push-pull
  GPIOC->OTYPER &= ~(1 << 6);
  GPIOC->OTYPER &= ~(1 << 7);
  GPIOC->OTYPER &= ~(1 << 8);
  GPIOC->OTYPER &= ~(1 << 9);

  // Set PC6, PC7, PC8, and PC9 to low speed
  GPIOC->OSPEEDR &= ~(3 << 12);
  GPIOC->OSPEEDR &= ~(3 << 14);
  GPIOC->OSPEEDR &= ~(3 << 16);
  GPIOC->OSPEEDR &= ~(3 << 18);

  // Set PC6, PC7, PC8, and PC9 to no pull-up, no pull-down
  GPIOC->PUPDR &= ~(3 << 12);
  GPIOC->PUPDR &= ~(3 << 14);
  GPIOC->PUPDR &= ~(3 << 16);
  GPIOC->PUPDR &= ~(3 << 18);

  // Set PC6 (red), PC7 (blue), PC8 (orange), and PC9 (green) to high
  GPIOC->ODR |= (1 << 6);
  GPIOC->ODR |= (1 << 7);
  GPIOC->ODR |= (1 << 8);
  GPIOC->ODR |= (1 << 9);

  // Enable USART3 clock
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  // Set baud rate to 115200 bits/s
  USART3->BRR = HAL_RCC_GetHCLKFreq() / 115200;
  // Enable transmission and reception
  USART3->CR1 |= USART_CR1_TE | USART_CR1_RE;
  // Enable USART3
  USART3->CR1 |= USART_CR1_UE;

  // Enable the RXNE interrupt
  USART3->CR1 |= USART_CR1_RXNEIE;

  // Enable the USART3 interrupt
  NVIC_EnableIRQ(USART3_4_IRQn);
  NVIC_SetPriority(USART3_4_IRQn, 0);

  // Send the initial message
  transmit_string("LED?");

  while (1)
  {
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
