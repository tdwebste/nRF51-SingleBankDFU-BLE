/* Copyright (c) 2014 SoundofMotion All Rights Reserved.
 *
 * The information contained herein is property of SoundofMotion
 *
 * NO WARRANTY of ANY KIND is provided. This heading must NOT
 * be removed from the file.
 *
 */

#include <string.h>
#include <stdint.h>

#include "led_lib.h"

/**@brief Function for initialization of LEDs.
 *
 * @details Initializes all LEDs used by the application.
 */
void leds_init(void)
{
    nrf_gpio_cfg_output(LED_0);
    nrf_gpio_cfg_output(LED_1);
}


/**@brief Function for clearing the LEDs.
 *
 * @details Clears all LEDs used by the application.
 */
void leds_off(void)
{
    nrf_gpio_pin_clear(LED_0);
    nrf_gpio_pin_clear(LED_1);
}

void leds_on(void)
{
    nrf_gpio_pin_set(LED_0);
    nrf_gpio_pin_set(LED_1);
}


