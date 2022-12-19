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

void forward()
{

}

void left()
{

}

void right()
{

}

void backward()
{

}

void stop()
{

}

void rotateL()
{

}

void rotateR()
{

}

void kick()
{

}

int conversion(int value)
{
    value -= 48;
    return value;
}

void movement(char* coordinates)
{
	static	int xValue = 0, yValue = 0;
    int x = 0, y = 0, xDigits = 0, yDigits = 0, xNegative = 0, yNegative = 0, trigger = 0;
    for (int i = 1; coordinates[i] != '\0'; i++) //get the digit count of each coordinate
    {
        if (trigger == 0) //search for x
        {
            if (coordinates[i] == '-')
            {
                xNegative = 1; //signal that x is a negative coordinate
            }
            else if (coordinates[i] == ',')
            {
                trigger = 1;
            }
            else
            {
                xDigits++;
            }
        }
        else //search for y
        {
            if (coordinates[i] == '-')
            {
                yNegative = 1; //signal that y is a negative coordinate
            }
            else
            {
                yDigits++;
            }
        }
    }

    int valueTrigger = 0, xNegTrigger = 0, yNegTrigger = 0;
    for (int i = 1; coordinates[i] != '\0'; i++)
    {
        if (valueTrigger == 0) //get value of x
        {
            if (xNegative == 1)
            {
                i++;
                xNegative = 0;
                xNegTrigger = 1;
            }
            if (coordinates[i] != ',')
            {
                x += conversion(coordinates[i]) * pow(10, xDigits - 1);
                xDigits--;
            }
            else
            {
                valueTrigger = 1;
            }
        }
        else
        {
            if (yNegative == 1)
            {
                i++;
                yNegative = 0;
                yNegTrigger = 1;

            }
            y += conversion(coordinates[i]) * pow(10, yDigits - 1);
            yDigits--;
        }
    }

    if (xNegTrigger == 1)
    {
        x = -x;
    }
    if (yNegTrigger == 1)
    {
        y = -y;
    }

    xValue = x;
    yValue = y;
    tft_prints(0, 0, "x=%d, y=%d", xValue, yValue);


//    			if ( (xValue < 30 && xValue >-30)|| (yValue <30 && yValue >-30) ){
//    				return;
//    			}
//    			double joy_angle_rad = atan2(yValue,xValue);
//
//
//    			tft_update(10);
//
//    			if ( (xValue < 30 && xValue >-30)|| (yValue <30 && yValue >-30) ){
//    				return;
//    			}
//
//    			double angle_wheel_1_rad = 1.57 ;
//    			double angle_wheel_2_rad= 3.665 ;
//    			double angle_wheel_3_rad= 5.7596 ; //90,210,330 deg (assume relative to head)
//
//    			double theta_1_rad = angle_wheel_1_rad - joy_angle_rad;
//    			double theta_2_rad = angle_wheel_2_rad - joy_angle_rad;
//    			double theta_3_rad = angle_wheel_3_rad - joy_angle_rad;
//
//    			int16_t speed1 = speed*sin(theta_1_rad);
//    			int16_t speed2 = speed*sin(theta_2_rad);
//    			int16_t speed3 = speed*sin(theta_3_rad);
//
//    			tft_prints(0,10, "ANGLE: %f", joy_angle_rad);
//				tft_prints(0,2,"speed1: %d", speed1);
//				tft_prints(0,4,"speed2: %d", speed2);
//				tft_prints(0,6, "speed3: %d", speed3);
//				tft_update(0);
//    			CAN_cmd_motor(speed1,speed2,speed3,0, &hcan1);
//}
}



void rotation() {
	CAN_cmd_motor(4000,4000,4000,0, &hcan1);
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
    while (1) {
    	static uint64_t last_ticks = 0;
    	static uint16_t motorSpeedLeft = 0;
    	static uint16_t motorSpeedRight = 0;
    	static uint16_t motorSpeedBack = 0;
    	HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    	UpdateMotorStatus();

    	static char data[1];

		HAL_UART_Receive(&huart1, (uint8_t*)&data ,sizeof(data),0xFFFF); //serial input, from coolterm

    	tft_prints(0, 0, "%s", data);
		tft_update(0);

		switch(data[0])
		{
			case '0': //stop
				motorSpeedLeft = 0; motorSpeedRight = 0; motorSpeedBack = 0;
				break;
			case '1': //forward
				motorSpeedLeft = 4000; motorSpeedRight = -4000; motorSpeedBack = 0;
				break;
			case '2': //backward
				motorSpeedLeft = -4000; motorSpeedRight = 4000; motorSpeedBack = 0;
				break;
			case '3': //left
				motorSpeedLeft = 2000; motorSpeedRight = -2000; motorSpeedBack = 3000;
				break;
			case '4': //right
				motorSpeedLeft = -2000; motorSpeedRight = 2000; motorSpeedBack = -3000;
				break;
			case '5': //rotate left
				motorSpeedLeft = 2000; motorSpeedRight = 2000; motorSpeedBack = 2000;
				break;
			case '6': //rotate right
				motorSpeedLeft = -2000; motorSpeedRight = -2000; motorSpeedBack = -2000;
				break;
			case '7': //stop
				motorSpeedLeft = 0; motorSpeedRight = 0; motorSpeedBack = 0;
				//kicking function
				break;
		}
		CAN_cmd_motor(motorSpeedLeft,motorSpeedRight,motorSpeedBack,0, &hcan1);

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
