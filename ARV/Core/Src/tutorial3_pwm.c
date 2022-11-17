//#include "main.h"
//#include "tim.h"
//#include "lcd/lcd.h"
//#include "lcd/lcd_graphics.h"
//#include <string.h>
//
//
///* Private variables END */
//
///* Private function prototypes START */
//void delay();
///* Private function prototypes END */
//
//// Enums are always good
//typedef enum {
//    BOTH_BTN, BTN1_ONLY, BTN2_ONLY, ALL_OFF,
//} ButtonState;
//
//typedef enum {
//    BUTTON1, BUTTON2,
//
//    //This trick means the NumButtons will equal how many buttons there are (how many enum values)
//    //This works as long as the enum values are simple (count up from 0)
//    NUM_BUTTON,
//} Button;
//
//typedef enum {
//    LED1, LED2, LED3, LED4, NUM_LED,
//} LED;
//
///**
// * @brief read the button state
// * return 1 if the button is pressed;
// * return 0 if the button is released;
// */
//static inline uint8_t read_button(Button btn) {
//    switch (btn) {
//    case BUTTON1:
//        return !btn_read(BTN1);
//    case BUTTON2:
//        return !btn_read(BTN2);
//    default:
//        return 0;
//    }
//}
//
//static ButtonState btn_state(void) {
//    if (read_button(BUTTON1) && read_button(BUTTON2)) {
//        return BOTH_BTN;
//    } else if (read_button(BUTTON1)) {
//        return BTN1_ONLY;
//    } else if (read_button(BUTTON2)) {
//        return BTN2_ONLY;
//    } else {
//        return ALL_OFF;
//    }
//}
//
//
//void pwm_init(void) {
//	// init the pwm prescaler value and auto-reload value and start the pwm
//	/* Your code start here */
//	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); //servoMotor
//	TIM5->ARR = 39999;
//	TIM5->PSC = 41;
//	TIM5->CCR1 = 2999;
//	HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1); //leftMotor
//	TIM10->ARR = 839;
//	TIM10->PSC = 9;
//	TIM10->CCR1 = 0;
//	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1); //rightMotor
//	TIM11->ARR = 839;
//	TIM11->PSC = 9;
//	TIM11->CCR1 = 0;
//
//
//	/* Your code end here */
//}
//
//void pwm_classwork(void) {
//	/* Your code start here */
//	/*
//	static int state1 = 1;
//	static int counter;
//	int delay;
//	if (gpio_read(BTN1) == 0 && state1 == 1) //single move
//	{
//		if (TIM5->CCR1 == 2999)
//		{
//			state1 = gpio_read(BTN1);
//			for (int i = 0; i <= 2000; i++)
//			{
//				TIM5->CCR1 = 2999 + i;
//				counter = HAL_GetTick() + 10000;
//				while (HAL_GetTick() < counter)
//				{
//					delay++;
//				}
//			}
//		}
//		else if (TIM5->CCR1 == 4999)
//		{
//			state1 = gpio_read(BTN1);
//
//			TIM5->CCR1 = 4749;
//			TIM5->CCR1 = 4499;
//			TIM5->CCR1 = 4249;
//			TIM5->CCR1 = 3999;
//			TIM5->CCR1 = 3749;
//			TIM5->CCR1 = 3499;
//			TIM5->CCR1 = 3249;
//			TIM5->CCR1 = 2999;
//		}
//
//	}
//	if (gpio_read(BTN1) == 1 && state1 == 0)
//	{
//		state1 = gpio_read(BTN1);
//	}
//	*/
//	/* Your code end here */
//}
//
//void pwm_homework(void) {
//	/* Your code start here */
//	static int state1 = 1;
//	static int state2 = 1;
//	static int initialTime;
//	static int doubleMove = 0;
//	static int timer;
//	static int count = 0;
//	static int halfseconds = 0;
//	static int extraMarble = 0;
//	static int totalMarbles = 3;
//	static int benchmark;
//	static int doubleCondition = 0;
//	static int win = 0;
//
//	static int marble1 = 50;
//	static int marble2 = 50;
//	static int marble3 = 50;
//	static int marble4 = 50;
//	static int marble5 = 50;
//
//	static int inc1 = 0;
//	static int inc2 = 0;
//	static int inc3 = 0;
//	static int inc4 = 0;
//	static int inc5 = 0;
//
//	static int dec1 = 1;
//	static int dec2 = 1;
//	static int dec3 = 1;
//	static int dec4 = 1;
//	static int dec5 = 1;
//
//	static int a = 0; //marble 1
//	static int b = 0;
//
//	static int c = 0; //marble 2
//	static int d = 0;
//
//	static int e = 0; //marble 3
//	static int f = 0;
//
//	static int g = 0; //marble 4
//	static int h = 0;
//
//	static int i = 0; //marble 5
//	static int j = 0;
//
//	static int gain = 0;
//	static int loss = 0;
//
//	totalMarbles = 3 + gain - loss;
//
//	if (totalMarbles >= 6 && win == 0)
//	{
//		tft_force_clear();
//		win = 1;
//	}
//
//	if (win == 0)
//	{
//		if (extraMarble == 0)
//		{
//			tft_prints(0, 0, "M1: %d\nM2: %d\nM3: %d", marble1, marble2, marble3);
//		}
//		if (extraMarble == 1)
//		{
//			tft_prints(0, 0, "M1: %d\nM2: %d\nM3: %d\nM4: %d\nM5: %d", marble1, marble2, marble3, marble4, marble5);
//		}
//
//	}
//
//	if (win == 1)
//	{
//		tft_prints(0, 0, "You win!");
//	}
//
//
//
//	halfseconds = (HAL_GetTick() / 500); //decrement
//	if (marble1 > 0)
//	{
//		if (dec1 == 1) //falling
//		{
//			marble1 = 50 - halfseconds + b;
//			a = halfseconds;
//		}
//
//		if (dec1 == 0) //rising
//		{
//			marble1 = 50 + halfseconds - (2 * a) + b;
//			if (halfseconds - a >= inc1)
//			{
//				dec1 = 1;
//				b = b + (inc1 * 2);
//				inc1 = 0;
//			}
//		}
//	}
//	else
//	{
//		loss++;
//	}
//	if (marble2 > 0)
//	{
//		if (dec2 == 1) //falling
//		{
//			marble2 = 50 - halfseconds + d;
//			c = halfseconds;
//
//		}
//		if (dec2 == 0) //rising
//		{
//			marble2 = 50 + halfseconds - (2 * c) + d;
//			if (halfseconds - c >= inc2)
//			{
//				dec2 = 1;
//				d = d + (inc2 * 2);
//				inc2 = 0;
//			}
//		}
//	}
//	else
//	{
//		loss++;
//	}
//	if (marble3 > 0)
//	{
//		if (dec3 == 1)
//		{
//			marble3 = 50 - halfseconds + f;
//			e = halfseconds;
//		}
//		if (dec3 == 0)
//		{
//			marble3 = 50 + halfseconds - (2 * e) + f;
//			if (halfseconds - e >= inc3)
//			{
//				dec3 = 1;
//				f = f + (inc3 * 2);
//				inc3 = 0;
//			}
//		}
//	}
//	else
//	{
//		loss++;
//	}
//
//	if (extraMarble == 0)
//	{
//		benchmark = halfseconds;
//	}
//
//
//	if (extraMarble == 1)
//	{
//		if (marble4 > 0)
//		{
//			if (dec4 == 1)
//			{
//				marble4 = 50 - halfseconds + benchmark + h;
//				g = halfseconds;
//			}
//			if (dec4 == 0)
//			{
//				marble4 = 50 + halfseconds - (2 * g) + h;
//				if (halfseconds - g >= inc4)
//				{
//					dec4 = 1;
//					h = h + (inc4 * 2);
//					inc4 = 0;
//				}
//			}
//		}
//		else
//		{
//			loss++;
//		}
//		if (marble5 > 0)
//		{
//			if (dec5 == 1)
//			{
//				marble5 = 50 - halfseconds + benchmark + j;
//				i = halfseconds;
//			}
//			if (dec5 == 0)
//			{
//				marble5 = 50 + halfseconds - (2 * i) + j;
//				if (halfseconds - i >= inc5)
//				{
//					dec4 = 1;
//					j = j + (inc5 * 2);
//					inc5 = 0;
//				}
//			}
//		}
//		else
//		{
//			loss++;
//		}
//	}
//
//	if (gpio_read(BTN1) == 0 && state1 == 1) //single move
//	{
//		if (TIM5->CCR1 == 2999)
//		{
//			state1 = gpio_read(BTN1);
//			for (int i = 0; i <= 2000; i++)
//			{
//				TIM5->CCR1 = 2999 + i;
//				delay();
//			}
//		}
//		else if (TIM5->CCR1 == 4999)
//		{
//			state1 = gpio_read(BTN1);
//			for (int i = 0; i <= 2000; i++)
//			{
//				TIM5->CCR1 = 4999 - i;
//				delay();
//			}
//		}
//		if (marble1 >= 5 && marble1 <= 10)
//		{
//			inc1 = 10;
//			dec1 = 0;
//		}
//		if (marble2 >= 5 && marble2 <= 10)
//		{
//			inc2 = 10;
//			dec2 = 0;
//		}
//		if (marble3 >= 5 && marble3 <= 10)
//		{
//			inc3 = 10;
//			dec3 = 0;
//		}
//		if (marble4 >= 5 && marble4 <= 10)
//		{
//			inc4 = 10;
//			dec4 = 0;
//		}
//		if (marble5 >= 5 && marble5 <= 10)
//		{
//			inc5 = 10;
//			dec5 = 0;
//		}
//		doubleCondition++;
//	}
//
//	if (gpio_read(BTN2) == 0 && state2 == 1 && doubleCondition >= 2) //double move (start the timer/rising edge)
//	{
//		state2 = gpio_read(BTN2);
//		initialTime = HAL_GetTick();
//		if (TIM5->CCR1 == 2999)
//		{
//			TIM5->CCR1 = 3439;
//		}
//		if (TIM5->CCR1 == 4999)
//		{
//			TIM5->CCR1 = 4559;
//		}
//	}
//
//	if (gpio_read(BTN2) == 0 && doubleCondition >= 2) //double move (while charging)
//	{
//		state2 = gpio_read(BTN2);
//		if ((HAL_GetTick() - initialTime) >= 2000)
//		{
//			gpio_reset(LED1);
//			gpio_reset(LED2);
//			gpio_reset(LED3);
//			gpio_reset(LED4);
//			doubleMove = 1;
//		}
//		if ((HAL_GetTick() - initialTime) >= 1500 && (HAL_GetTick() - initialTime) < 2000)
//		{
//			gpio_reset(LED1);
//			gpio_reset(LED2);
//			gpio_reset(LED3);
//			gpio_set(LED4);
//			doubleMove = 0;
//		}
//		if ((HAL_GetTick() - initialTime) >= 1000 && (HAL_GetTick() - initialTime) < 1500)
//		{
//			gpio_reset(LED1);
//			gpio_reset(LED2);
//			gpio_set(LED3);
//			gpio_set(LED4);
//			doubleMove = 0;
//		}
//		if ((HAL_GetTick() - initialTime) >= 500 && (HAL_GetTick() - initialTime) < 1000)
//		{
//			gpio_reset(LED1);
//			gpio_set(LED2);
//			gpio_set(LED3);
//			gpio_set(LED4);
//			doubleMove = 0;
//		}
//		if ((HAL_GetTick() - initialTime) < 500)
//		{
//			gpio_set(LED1);
//			gpio_set(LED2);
//			gpio_set(LED3);
//			gpio_set(LED4);
//			doubleMove = 0;
//		}
//	}
//
//	if (gpio_read(BTN2) == 1 && state2 == 0 && doubleCondition >= 2) //double move (falling edge/when button is released)
//	{
//		state2 = gpio_read(BTN2);
//		gpio_set(LED1);
//		gpio_set(LED2);
//		gpio_set(LED3);
//		gpio_set(LED4);
//		if (doubleMove == 0) //not charged
//		{
//			if (TIM5->CCR1 == 3439)
//			{
//				TIM5->CCR1 = 4999;
//			}
//			if (TIM5->CCR1 == 4559)
//			{
//				TIM5->CCR1 = 2999;
//			}
//			if (marble1 >= 5 && marble1 <= 10)
//			{
//				inc1 = 20;
//				dec1 = 0;
//			}
//			if (marble2 >= 5 && marble2 <= 10)
//			{
//				inc2 = 20;
//				dec2 = 0;
//			}
//			if (marble3 >= 5 && marble3 <= 10)
//			{
//				inc3 = 20;
//				dec3 = 0;
//			}
//			if (marble4 >= 5 && marble4 <= 10)
//			{
//				inc4 = 20;
//				dec4 = 0;
//			}
//			if (marble5 >= 5 && marble5 <= 10)
//			{
//				inc5 = 20;
//				dec5 = 0;
//			}
//		}
//		else if (doubleMove == 1)// charged
//		{
//			timer = HAL_GetTick() + 500;
//			if (TIM5->CCR1 == 3439)
//			{
//				TIM5->CCR1 = 4999;
//				while (HAL_GetTick() < timer)
//				{
//					count++;
//				}
//				TIM5->CCR1 = 2999;
//			}
//			if (TIM5->CCR1 == 4559)
//			{
//				TIM5->CCR1 = 2999;
//				while (HAL_GetTick() < timer)
//				{
//					count++;
//				}
//				TIM5->CCR1 = 4999;
//			}
//			extraMarble = 1;
//			gain += 2;
//			if (marble1 >= 5 && marble1 <= 10)
//			{
//				inc1 = 30;
//				dec1 = 0;
//			}
//			if (marble2 >= 5 && marble2 <= 10)
//			{
//				inc2 = 30;
//				dec2 = 0;
//			}
//			if (marble3 >= 5 && marble3 <= 10)
//			{
//				inc3 = 30;
//				dec3 = 0;
//			}
//			if (marble4 >= 5 && marble4 <= 10)
//			{
//				inc4 = 30;
//				dec4 = 0;
//			}
//			if (marble5 >= 5 && marble5 <= 10)
//			{
//				inc5 = 30;
//				dec5 = 0;
//			}
//		}
//		doubleCondition = 0;
//	}
//
//	if (gpio_read(BTN1) == 1 && state1 == 0)
//	{
//		state1 = gpio_read(BTN1);
//	}
//	if (gpio_read(BTN2) == 1 && state2 == 0)
//	{
//		state2 = gpio_read(BTN2);
//	}
//	/* Your code end here*/
//}
//
//void delay()
//{
//	int counter= 0;
//	while (counter < 5000)
//	{
//		counter++;
//	}
//}
