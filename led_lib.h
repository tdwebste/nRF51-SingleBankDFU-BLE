/* Copyright (c) 2014 SoundofMotion All Rights Reserved.
 *
 * The information contained herein is property of SoundofMotion
 *
 * NO WARRANTY of ANY KIND is provided. This heading must NOT
 * be removed from the file.
 *
 */

/**@file
 *
 * @defgroup data_gather
 * @{
 * @ingroup  data
 * @brief    Data Gathering and processing
 *
 * @details  handlers (callbacks) from events and timers
 *

 */

#ifndef LED_LIB_H__
#define LED_LIB_H__

#include "nrf_gpio.h"

//#include "twi_master_config.h"  // eval board requires flag DEBUG_EVAL
#include "cscboard.h"  // eval board requires flag DEBUG_EVAL

#define ENTER_BOOTLOADER_VALUE              (1)

/**< request read data  millisec to ticks */
#define RF_LED_ON_INTERVAL              APP_TIMER_TICKS(3000, APP_TIMER_PRESCALER) 
#define RF_LED_OFF_INTERVAL             APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
#define ADV_LED_ON_INTERVAL             APP_TIMER_TICKS(400, APP_TIMER_PRESCALER)
#define ADV_LED_OFF_INTERVAL            APP_TIMER_TICKS(600, APP_TIMER_PRESCALER)
#define BON_LED_ON_INTERVAL             APP_TIMER_TICKS(500, APP_TIMER_PRESCALER)
#define BON_LED_OFF_INTERVAL            APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)

#define SLEEP_LED_ON_INTERVAL           APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)
#define SLEEP_LED_OFF_INTERVAL          APP_TIMER_TICKS(200, APP_TIMER_PRESCALER)
#define SLEEP_LED_TIMEOUT               APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)

#define DETECT_LED_ON_INTERVAL           APP_TIMER_TICKS(300, APP_TIMER_PRESCALER)
#define DETECT_LED_OFF_INTERVAL          APP_TIMER_TICKS(100, APP_TIMER_PRESCALER)
#define DETECT_LED_TIMEOUT               APP_TIMER_TICKS(2000, APP_TIMER_PRESCALER)

#define RF_LED_ON_INTERVAL              APP_TIMER_TICKS(3000, APP_TIMER_PRESCALER) /**< request read data  millisec to ticks */
#define RF_LED_OFF_INTERVAL             APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
#define ADV_LED_ON_INTERVAL             APP_TIMER_TICKS(400, APP_TIMER_PRESCALER)
#define ADV_LED_OFF_INTERVAL            APP_TIMER_TICKS(600, APP_TIMER_PRESCALER)
#define BON_LED_ON_INTERVAL             APP_TIMER_TICKS(500, APP_TIMER_PRESCALER)
#define BON_LED_OFF_INTERVAL            APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)


/**@brief Function for initialization of LEDs.
 *
 * @details Initializes all LEDs used by the application.
 */
void leds_init(void);


/**@brief Function for clearing the LEDs.
 *
 * @details Clears all LEDs used by the application.
 */
void leds_off(void);

void leds_on(void);



#endif // LED_LIB_H__

/** @} */
