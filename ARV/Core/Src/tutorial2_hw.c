#include "main.h"
#include "lcd/lcd.h"
#include "lcd/lcd_graphics.h"
#include <string.h>

/** Design ideologies:
 * More small functions >>> Few giant functions
 * Reusability and Extendability is most important (Dont-Repeat-Yourself)
 * then Readability (comments are good, self documenting code is best)
 * No "magic numbers", use defines, constants, variables etc
 * ALWAYS INDENT CORRECTLY
 * No blocking (long while loops, HAL_Delay)
 */

// Enums are always good
typedef enum {
    BOTH_BTN, BTN1_ONLY, BTN2_ONLY, ALL_OFF,
} ButtonState;

typedef enum {
    BUTTON1, BUTTON2,

    //This trick means the NumButtons will equal how many buttons there are (how many enum values)
    //This works as long as the enum values are simple (count up from 0)
    NUM_BUTTON,
} Button;

typedef enum {
    LED1, LED2, LED3, LED4, NUM_LED,
} LED;

/**
 * @brief read the button state
 * return 1 if the button is pressed;
 * return 0 if the button is released;
 */
static inline uint8_t read_button(Button btn) {
    switch (btn) {
    case BUTTON1:
        return !btn_read(BTN1);
    case BUTTON2:
        return !btn_read(BTN2);
    default:
        return 0;
    }
}

ButtonState btn_state(void) {
    if (read_button(BUTTON1) && read_button(BUTTON2)) {
        return BOTH_BTN;
    } else if (read_button(BUTTON1)) {
        return BTN1_ONLY;
    } else if (read_button(BUTTON2)) {
        return BTN2_ONLY;
    } else {
        return ALL_OFF;
    }
}

/* Private variables START */

ButtonState state = ALL_OFF;

/* Private variables END */

/* Private function prototypes START */

/* Private function prototypes END */

void gpio_classwork(void)
{
    /* Your code start here */
	uint8_t state1 = gpio_read(BTN1);
	uint8_t state2 = gpio_read(BTN2);
	uint32_t target1 = 0;
	uint32_t target2 = 0;
	uint32_t target3 = 0;

	if (state1 == 0 && state2 == 0)
	{
		target1 = HAL_GetTick() + 1000;
		while (HAL_GetTick() < target1)
		{
			gpio_reset(LED1);
			gpio_reset(LED3);
			if ((HAL_GetTick() - target2) >= 100)
			{
				gpio_toggle(LED2);
				target2 = HAL_GetTick();
			}
		}
		gpio_set(LED1);
		gpio_set(LED2);
		gpio_set(LED3);
		target1 = HAL_GetTick() + 1000;
		while (HAL_GetTick() < target1)
		{
			gpio_reset(LED2);
			if ((HAL_GetTick() - target2) >= 100)
			{
				gpio_toggle(LED1);
				gpio_toggle(LED3);
				target2 = HAL_GetTick();
			}
		}
		gpio_set(LED1);
		gpio_set(LED2);
		gpio_set(LED3);
	}
	else if (state1 == 0 && state2 == 1)
	{
		gpio_reset(LED1);
	}
	else if (state1 == 1 && state2 == 0)
	{
		gpio_set(LED2);
		while (gpio_read(BTN1) == 1 && gpio_read(BTN2) == 0)
		{
			if ((HAL_GetTick() - target3) >= 50)
			{
				gpio_toggle(LED2);
				target3 = HAL_GetTick();
			}
		}
	}
	else
	{
		gpio_set(LED1);
		gpio_set(LED2);
		gpio_set(LED3);
	}
    /* Your code end here */
}

void tft_classwork(void) {
    /* Your code start here */
	static int minutes = 0;
	static int seconds = 0;
	static int miliseconds = 0;
	tft_prints(0, 0, "%d:%d:%d", minutes, seconds, miliseconds);
	miliseconds = (HAL_GetTick() % 1000);
	seconds = (HAL_GetTick() / 1000) - (minutes * 60);
	minutes = (HAL_GetTick() / 60000);

	if (seconds % 2 == 0)
	{
		tft_print_rectangle(RED, 0, 50, 50, 50);
	}
	else if (seconds % 2 == 1)
	{
		tft_print_rectangle(BLUE, 0, 50, 50, 50);
	}
    /* Your code end here */
}

void tutorial2_homework(void) {
    /* Your code start here */
	static int state = 1;
	static int stringTarget = 0;
	static int ledTarget = 0;
	static int target = 0;

	if ((gpio_read(BTN1) == 0) && state == 1)
	{
		state = gpio_read(BTN1);
		stringTarget = HAL_GetTick() + 1000;
	}
	if (HAL_GetTick() < stringTarget)
	{
		tft_prints(0, 0, "Hello, Ethann");
		tft_update(100);
	}

	if ((gpio_read(BTN1) == 1) && state == 0)
	{
		state = gpio_read(BTN1);
		ledTarget = HAL_GetTick() + 1000;
	}

	if (HAL_GetTick() < ledTarget)
	{
		target = HAL_GetTick() + 50;
		if (HAL_GetTick() > target)
		{
			gpio_toggle(LED1);
			target = target + 50;
		}
	}
	if (HAL_GetTick() > ledTarget)
	{
		gpio_set(LED1);
	}

    /* Your code end here*/
}
/*
	if (gpio_read(BTN1) == 0)
	{
		tft_prints(0, 0, "Hello, Ethann");
		gpio_set(LED1);
	}
	else if (gpio_read(BTN1) == 1)
	{
		tft_force_clear();
		int lastticks = 0;
		while (gpio_read(BTN1) == 1)
		{
			if ((HAL_GetTick() - lastticks) >= 50)
			{
				gpio_toggle(LED1);
				lastticks = HAL_GetTick();
			}
		}
	}
 */
// You can define your helper functions down below
// You can also modify the function definition above as long as your move is reasonable
