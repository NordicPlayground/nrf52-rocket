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
#include "strato_led.h"
#include "strato_ignition.h"
#include "strato_ble_ctrl_sys.h"

#include "pca20027.h"
#include "drv_mpu9250.h"
#include "SEGGER_RTT.h"
#include "app_util_platform.h"
#include "drv_humid_temp.h"

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
    leds_set_rgb(0x00FF0000);
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the Power Manager.
 */
#define FPU_EXCEPTION_MASK 0x0000009F
static void power_manage(void)
{
    //Fix FPU stuck in raised flag state
    __set_FPSCR(__get_FPSCR()  & ~(FPU_EXCEPTION_MASK));
    (void) __get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);

    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
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

    err_code = leds_set_rgb(0x0000FF00);
    APP_ERROR_CHECK(err_code);

    strato_ble_ctrl_sys_init();

    // Enter main loop.



//
// //    drv_mpu9250_init_t motion_init;
// //    motion_init.p_twi_instance = &m_twi_sensors;
// //    motion_init.p_twi_cfg = &m_twi_config;
// //    drv_mpu9250_init(&motion_init);
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
       app_sched_execute();
       power_manage();
    }
}


/**
 * @}
 */
