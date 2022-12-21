/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <math.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd/lcd.h"
/* USER CODE END Includes */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);


void shoot()
{

}

int main(void) {
    /* USER CODE BEGIN 1 */

	HAL_Init();

    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_CAN2_Init();
    MX_USART1_UART_Init();
    MX_DMA_Init();
    MX_USART2_UART_Init();
    MX_TIM5_Init();
    /* USER CODE BEGIN 2 */


    tft_init(PIN_ON_BOTTOM, BLACK, WHITE, YELLOW, DARK_GREEN);
    tft_force_clear();

    can_init();
    while (1)
    {
    	static uint64_t last_ticks = 0;
    	static uint16_t motorSpeedLeft = 0;
    	static uint16_t motorSpeedRight = 0;
    	static uint16_t motorSpeedBack = 0;

    	static uint8_t trState = 0;
    	static uint8_t autoState = 1;
    	static char prevInput = '0';

    	static uint8_t dropper1 = 0;
    	static uint8_t dropper2 = 0;
    	static uint8_t dropper3 = 0;

    	static uint8_t manualEnable = 0;

    	HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    	UpdateMotorStatus();

    	static char data[1];

		HAL_UART_Receive(&huart1, (uint8_t*)&data ,sizeof(data),0xFFFF); //serial input, from coolterm

    	tft_prints(0, 0, "%s, %d, %d, %d", data, dropper1, dropper2, dropper3);
		tft_update(0);

		if (HAL_GetTick() - last_ticks >= 2000)
		{
			if (data[0] == 'M')
			{
				if (manualEnable == 1)
				{
					manualEnable = 0;
				}
				else if (manualEnable == 0)
				{
					manualEnable = 1;
				}

				last_ticks = HAL_GetTick() + 2000;
			}
		}

		if (manualEnable == 0)
		{
			trState = 0;
		}
		else if (manualEnable == 1)
		{
			trState = 1;
		}

		if (trState == 0) //Manual Mode
		{
			switch(data[0])
			{
				case '0': //stop
					motorSpeedLeft = 0; motorSpeedRight = 0; motorSpeedBack = 0;
					break;
				case '1': //forward
					motorSpeedLeft = -4000; motorSpeedRight = 4000; motorSpeedBack = 0;
					break;
				case '2': //backward
					motorSpeedLeft = 4000; motorSpeedRight = -4000; motorSpeedBack = 0;
					break;
				case '3': //left
					motorSpeedLeft = -2000; motorSpeedRight = 2000; motorSpeedBack = -3000;
					break;
				case '4': //right
					motorSpeedLeft = 2000; motorSpeedRight = -2000; motorSpeedBack = 3000;
					break;
				case '5': //rotate left
					motorSpeedLeft = -2000; motorSpeedRight = -2000; motorSpeedBack = -2000;
					break;
				case '6': //rotate right
					motorSpeedLeft = 2000; motorSpeedRight = 2000; motorSpeedBack = 2000;
					break;
				case '7': //Clamp and First Dropper Lock
					if (dropper1 == 0)
					{
						dropper1 = 1;
					}
					else
					{
						dropper1 = 0;
					}
					break;
				case '8': //Lifter and Second Dropper Lock
					if (dropper2 == 0)
					{
						dropper2 = 1;
					}
					else
					{
						dropper2 = 0;
					}
					break;
				case '9': // Third Dropper Lock
					if (dropper3 == 0)
					{
						dropper3 = 1;
					}
					else
					{
						dropper3 = 0;
					}
					break;
			}

			if (dropper1 == 0)
			{
				gpio_set(valve1);
			}
			else if (dropper1 == 1)
			{
				gpio_reset(valve1);
			}

			if (dropper2 == 0)
			{
				gpio_set(valve2);
			}
			else if (dropper2 == 1)
			{
				gpio_reset(valve2);
			}

			if (dropper3 == 0)
			{
				gpio_set(valve3);
			}
			else if (dropper3 == 1)
			{
				gpio_set(valve3);
			}

		}
		else if (trState == 1) //Automatic Mode
		{
			if (HAL_GetTick() - last_ticks >= 5000)
			{
				if (autoState == 0)
				{
					autoState = 1;
				}
				else
				{
					autoState = 0;
				}
				last_ticks = HAL_GetTick() + 5000;
			}

			switch(autoState)
			{
				case 1:	//Forward
					motorSpeedLeft = -4000; motorSpeedRight = 4000; motorSpeedBack = 0;
					break;
				case 0:	//Left
					motorSpeedLeft = -2000; motorSpeedRight = 2000; motorSpeedBack = -3000;
					break;
			}
		}
		CAN_cmd_motor(motorSpeedLeft/2,motorSpeedRight/2,motorSpeedBack/2,0, &hcan1);
		prevInput = data[0];
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

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
