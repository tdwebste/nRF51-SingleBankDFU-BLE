/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/**@file
 *
 * @defgroup ble_sdk_app_bootloader_main main.c
 * @{
 * @ingroup dfu_bootloader_api
 * @brief Bootloader project main file.
 *
 * -# Receive start data package.
 * -# Based on start packet, prepare NVM area to store received data.
 * -# Receive data packet.
 * -# Validate data packet.
 * -# Write Data packet to NVM.
 * -# If not finished - Wait for next packet.
 * -# Receive stop data packet.
 * -# Activate Image, boot application.
 *
 */


#ifndef MAIN_H__
#define MAIN_H__

#include "dfu.h"
#include "dfu_transport.h"
#include "bootloader.h"

#include <stdint.h>
#include <string.h>
#include <stddef.h>

#include "nordic_common.h"
#include "nrf.h"
#ifndef S310_STACK
#include "nrf_mbr.h"
#endif // S310_STACK
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf51_bitfields.h"
#include "ble.h"
#include "nrf51.h"
#include "ble_hci.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nrf_error.h"
//#include "boards.h"
//#include "app_gpiote.h"
#include "ble_debug_assert_handler.h"
#include "softdevice_handler.h"
#include "pstorage_platform.h"
#include "nrf_delay.h"
#include "led_lib.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT     0                                                   /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

//#define BOOTLOADER_BUTTON_PIN           BUTTON_7                                                /**< Button used to enter SW update mode. */
//#define APP_GPIOTE_MAX_USERS            1                                                       /**< Number of GPIOTE users in total. Used by button module and dfu_transport_serial module (flow control). */

#define APP_TIMER_PRESCALER             0                                                       /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            4                                                       /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                                       /**< Size of timer operation queues. */

//#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)                /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, 0)                        /**< Maximum size of scheduler events. */

#define SCHED_QUEUE_SIZE                20

#define RF_LED_ON_INTERVAL              APP_TIMER_TICKS(3000, APP_TIMER_PRESCALER) /**< request read data  millisec to ticks */
#define RF_LED_OFF_INTERVAL             APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)


#define DFU_SUCESS_ON_INTERVAL          200 //delay ms
#define DFU_SUCESS_OFF_INTERVAL         1000  //delay ms
#define DFU_SUCESS_COUNT                3
#define DFU_FAIL_ON_INTERVAL            3000 //delay ms

#define DFU_STATE_TIME                  APP_TIMER_TICKS(60*1000, APP_TIMER_PRESCALER)             /**< dfu state time  millisec to ticks */

#define BONDED_STATE_TIME               APP_TIMER_TICKS(5*60*1000, APP_TIMER_PRESCALER)           /**< bonded state time   millisec to ticks */


extern app_timer_id_t               m_led_on_timer_id;
extern app_timer_id_t               m_led_off_timer_id;
extern app_timer_id_t               m_dfu_state_timer_id;

extern uint32_t led_t_on, led_t_off, led_repeat;


void led_timers_start(void);

void state_timers_start(void);
void state_timers_stop(void);
void bonded_state_timers_start(void);

void led_timers_stop(void);

//void leds_off(void);
//void leds_on(void);






#endif // MAIN_H__

/** @} */
