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
#include "main.h"

app_timer_id_t               m_led_on_timer_id;
app_timer_id_t               m_led_off_timer_id;
app_timer_id_t               m_dfu_state_timer_id;

uint32_t led_t_on, led_t_off, led_repeat;

static ble_gap_addr_t           m_ble_addr;


/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    nrf_gpio_pin_set(LED_0);
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover on reset.
    NVIC_SystemReset();
}


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] file_name   File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for initialization of LEDs.
 *
 * @details Initializes all LEDs used by the application.
 */

/**@brief Function for entering application
 *
 * @details
 *  Select a bank region to use as application region.
 *  @note: Only applications running from DFU_BANK_0_REGION_START is supported.
 */
static void launch_app()
{
    uint32_t err_code;

    leds_off();
    if (bootloader_app_is_valid(DFU_BANK_0_REGION_START))
    {
        err_code = sd_power_gpregret_clr(0xFF);
        err_code = sd_power_gpregret_set(ENTER_BOOTLOADER_VALUE);
        if (err_code != NRF_SUCCESS)
        {
            APP_ERROR_CHECK(err_code);
        }
        bootloader_app_start(DFU_BANK_0_REGION_START);
    }
        /* //debugging failed launches
        leds_off();
        for (int i=0; i<3; i++)
        {
            leds_on();
            nrf_delay_ms(1000);
            leds_off();
            nrf_delay_ms(200);
            leds_on();
            nrf_delay_ms(500);
            leds_off();
            nrf_delay_ms(200);
            leds_on();
            nrf_delay_ms(250);
            leds_off();
            nrf_delay_ms(100);
            leds_on();
            nrf_delay_ms(250);
            leds_off();
            nrf_delay_ms(200);
            leds_on();
            nrf_delay_ms(500);
            leds_off();
            nrf_delay_ms(200);
            leds_on();
            nrf_delay_ms(1000);
            leds_off();
        }
        */
}


/**@brief Function for initializing the GPIOTE handler module.
 */
 /*
static void gpiote_init(void)
{
    APP_GPIOTE_INIT(APP_GPIOTE_MAX_USERS);
}
*/

void led_on_timeout_handler(void * p_context)
{
    //on timeout
    leds_off();
}
void led_cycle_handler(void * p_context)
{
    uint32_t err_code;
    //repeat cycle
    leds_on();
    // Start led timers.
    err_code = app_timer_start(m_led_on_timer_id, led_t_on, NULL);
    APP_ERROR_CHECK(err_code);
}
void dfu_led_fail(void)
{
    //indicate no dfu
    led_timers_stop();
    leds_off();
    leds_on();
    nrf_delay_ms(DFU_FAIL_ON_INTERVAL);
    leds_off();
}
void dfu_led_success(void)
{
    //success
    for (int i=0; i<DFU_SUCESS_COUNT; i++)
    {
        leds_on();
        nrf_delay_ms(DFU_SUCESS_ON_INTERVAL);
        leds_off();
        nrf_delay_ms(DFU_SUCESS_OFF_INTERVAL);
    }
}
void dfu_timeout_handler(void * p_context)
{
    dfu_led_fail();
    launch_app();
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    uint32_t err_code;
    // Initialize timer module, making it use the scheduler.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS,
            APP_TIMER_OP_QUEUE_SIZE, true);

    // led on timer.
    err_code = app_timer_create(&m_led_on_timer_id,
            APP_TIMER_MODE_SINGLE_SHOT,
            led_on_timeout_handler);
    APP_ERROR_CHECK(err_code);
    // led cycle timer.
    err_code = app_timer_create(&m_led_off_timer_id,
            APP_TIMER_MODE_REPEATED,
            led_cycle_handler);
    APP_ERROR_CHECK(err_code);

    // dfu state timer
    err_code = app_timer_create(&m_dfu_state_timer_id,
            APP_TIMER_MODE_SINGLE_SHOT,
            dfu_timeout_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting application timers.
 */
void state_timers_start(void)
{
    uint32_t err_code;
    // Start state timers.
    err_code = app_timer_start(m_dfu_state_timer_id,
            DFU_STATE_TIME, NULL);
    APP_ERROR_CHECK(err_code);
    //advertise
    led_timers_stop();
    led_t_on = ADV_LED_ON_INTERVAL;
    led_t_off = ADV_LED_OFF_INTERVAL;
    led_timers_start();
}
void bonded_state_timers_start(void)
{
    uint32_t err_code;
     //Connected Bonded
    state_timers_stop();
    led_timers_stop();
    // Start state timers.
    err_code = app_timer_start(m_dfu_state_timer_id,
            BONDED_STATE_TIME, NULL);
    APP_ERROR_CHECK(err_code);
    led_t_on = BON_LED_ON_INTERVAL;
    led_t_off = BON_LED_OFF_INTERVAL;
    led_timers_start();
}
void state_timers_stop(void)
{
    uint32_t err_code;
    // Stop state timers.
    err_code = app_timer_stop(m_dfu_state_timer_id);
    APP_ERROR_CHECK(err_code);
}
void led_timers_start(void)
{
    uint32_t err_code;
    leds_on();
    // Start led timers.
    err_code = app_timer_start(m_led_on_timer_id, led_t_on, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_start(m_led_off_timer_id,
            (led_t_on + led_t_off), NULL);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for stoping application timers.
 */
void led_timers_stop(void)
{
    uint32_t err_code;
    // Stop led timers.
    err_code = app_timer_stop(m_led_on_timer_id);
    APP_ERROR_CHECK(err_code);
    err_code = app_timer_stop(m_led_off_timer_id);
    APP_ERROR_CHECK(err_code);
    leds_off();
}

/**@brief Function for initializing the button module.
 */

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void sys_evt_dispatch(uint32_t event)
{
    pstorage_sys_event_handler(event);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

#ifndef S310_STACK
    sd_mbr_command_t com = {SD_MBR_COMMAND_INIT_SD, };

    err_code = sd_mbr_command(&com);
    APP_ERROR_CHECK(err_code);

    err_code = sd_softdevice_vector_table_base_set(BOOTLOADER_REGION_START);
    APP_ERROR_CHECK(err_code);
#endif // S310_STACK

    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, true);

#ifndef S310_STACK
    // Enable BLE stack
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    err_code = sd_ble_gap_address_get(&m_ble_addr);
    APP_ERROR_CHECK(err_code);
    err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &m_ble_addr);
    APP_ERROR_CHECK(err_code);

#endif // S310_STACK

    err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for event scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief 2.4G rf
 */
static void test_rf(void)
{
	// RF_TEST pin set in board file
    nrf_gpio_cfg_sense_input(RF_TEST_IN_PIN_30,
                             NRF_GPIO_PIN_PULLUP,
                             NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(RF_TEST_IN_PIN_00,
                             NRF_GPIO_PIN_PULLUP,
                             NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(RF_TEST_IN_PIN_01,
                             NRF_GPIO_PIN_PULLUP,
                             NRF_GPIO_PIN_SENSE_LOW);
    nrf_gpio_cfg_sense_input(RF_TEST_IN_PIN_02,
                             NRF_GPIO_PIN_PULLUP,
                             NRF_GPIO_PIN_SENSE_LOW);
    nrf_delay_us(1000); //1ms stablize
	if( (nrf_gpio_pin_read(RF_TEST_IN_PIN_30) == 0)
        || (nrf_gpio_pin_read(RF_TEST_IN_PIN_00) == 0)
        || (nrf_gpio_pin_read(RF_TEST_IN_PIN_01) == 0)
        || (nrf_gpio_pin_read(RF_TEST_IN_PIN_02) == 0) )
    {
        launch_app();
	}
}

/**@brief Function for application main entry.
 */
int main(void)
{
    uint32_t err_code;
	uint32_t gprepret_value;
    bool     power_on_reset = true;

    leds_init();

    // This check ensures that the defined fields in the bootloader corresponds with actual
    // setting in the nRF51 chip.
    APP_ERROR_CHECK_BOOL(NRF_UICR->CLENR0 == CODE_REGION_1_START);

    APP_ERROR_CHECK_BOOL(*((uint32_t *)NRF_UICR_BOOT_START_ADDRESS) == BOOTLOADER_REGION_START);
    APP_ERROR_CHECK_BOOL(NRF_FICR->CODEPAGESIZE == CODE_PAGE_SIZE);

    // Initialize.
    timers_init();

//    gpiote_init();
    ble_stack_init();
    scheduler_init();

    //Read alatech rf test pin
    test_rf();

	//Read the retained memory for deciding wether DFU need or not
    err_code = sd_power_gpregret_get(&gprepret_value);
    APP_ERROR_CHECK(err_code);
    power_on_reset = ((gprepret_value == ENTER_BOOTLOADER_VALUE)? false: true);
    if (power_on_reset || (!bootloader_app_is_valid(DFU_BANK_0_REGION_START)))
    {
        // Initiate an update of the firmware.
        err_code = bootloader_dfu_start();
        if (err_code != NRF_SUCCESS)
        {
            APP_ERROR_CHECK(err_code);
            dfu_led_fail();
            err_code = sd_power_gpregret_clr(0xFF);
            NVIC_SystemReset();
        }
        else
        {
            power_on_reset = false;
            dfu_led_success();
        }
        if (err_code != NRF_SUCCESS)
        {
            APP_ERROR_CHECK(err_code);
        }
    }

    launch_app();
    // if cannot launch_app
    // clear flag to force the dfu to run
    // reset
    err_code = sd_power_gpregret_clr(0xFF);
    NVIC_SystemReset();
}
