/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
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

/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
// #include "nrf.h"
#include "app_error.h"
#include "softdevice_handler.h"
#include "app_scheduler.h"
#include "app_timer_appsh.h"


#include "strato_app_config.h"
#include "drv_led.h"
#include "strato_ignition.h"

#include "drv_pressure.h"
#include "drv_pressure_mpl3115A2.h"
#include "nrf_gpio.h"
#include "pca20027.h"
#include "drv_mpu9250.h"
#include "SEGGER_RTT.h"
#include "app_util_platform.h"
#include "drv_humid_temp.h"
#include "app_timer.h"

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the Power Manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

// static void press_evt_handler(drv_pressure_evt_t const * p_evt,
//                               void *                     p_context)
// {
//
// }
//
// static void humid_temp_evt_handler(drv_humid_temp_evt_t const * p_event)
// {
//     SEGGER_RTT_printf(0,"Humidity: %d", drv_humid_temp_humidity_get());
// }

static ret_code_t leds_init()
{
    static drv_led_lpp_rgb_t rgb = DRV_LED_LPP_RGB_INIT();

    APP_TIMER_DEF(m_r_timer);
    APP_TIMER_DEF(m_g_timer);
    APP_TIMER_DEF(m_b_timer);

    drv_led_lpp_rgb_timers_t rgb_timers = {&m_r_timer,&m_g_timer,&m_b_timer};
    drv_led_lpp_rgb_config_t rgb_config = DRV_LED_LPP_RGB_CONFIG_INIT(LED_R,
                                                                      LED_G,
                                                                      LED_B,
                                                                      false,
                                                                      &rgb_timers);

    return drv_led_rgb_init(&rgb,&rgb_config);

}
/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);

    ret_code_t err_code;
    err_code = leds_init();
    APP_ERROR_CHECK(err_code);






    // Enter main loop.


//     drv_pressure_init_t press_init;
//     static const  nrf_drv_twi_t m_twi_sensors = NRF_DRV_TWI_INSTANCE(0);
//     static const nrf_drv_twi_config_t m_twi_config =
//     {
//         .scl = TWI_SCL_PIN_NUMBER,
//         .sda = TWI_SDA_PIN_NUMBER,
//         .frequency = NRF_TWI_FREQ_100K,
//         .interrupt_priority = APP_IRQ_PRIORITY_LOW
//     };
//
//     press_init.p_twi_instance = &m_twi_sensors;
//     press_init.pressure_evt_handler = press_evt_handler;
//     press_init.mode = DRV_PRESSURE_MODE_ALTIMETER;
//     drv_pressure_init(&press_init);
//
// //    drv_mpu9250_init_t motion_init;
// //    motion_init.p_twi_instance = &m_twi_sensors;
// //    motion_init.p_twi_cfg = &m_twi_config;
// //    drv_mpu9250_init(&motion_init);
//
// //    uint8_t who_am_i = 0;
// //    drv_mpu9250_read(0x68,0x75,1,&who_am_i);
// //    SEGGER_RTT_printf(0,"who am i: %d \r\n", who_am_i);
//
//     static const drv_humid_temp_init_t humid_temp_init =
//     {
//         .p_twi_instance = &m_twi_sensors,
//         .evt_handler = humid_temp_evt_handler
//     };
//
//     APP_TIMER_INIT(0,6,false);
//     uint32_t err_code = drv_humid_temp_init(&humid_temp_init);
//     APP_ERROR_CHECK(err_code);
//
//     err_code = drv_humid_temp_sample_begin( SENSOR_TYPE_HUMIDITY );
//     APP_ERROR_CHECK(err_code);

    for (;;)
    {
//        gpio_test_all_read();
    //    power_manage();
    }
}


/**
 * @}
 */
