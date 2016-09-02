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

#ifndef __DRV_LED_H__
#define __DRV_LED_H__

#include <stdint.h>
#include "pca20027.h"
#include "low_power_pwm.h"
#include "app_error.h"

/**
 * @brief Structure holding the PWM instances of each RGB group
 */
typedef struct
{
    low_power_pwm_t red;
    low_power_pwm_t green;
    low_power_pwm_t blue;
}drv_led_lpp_rgb_t;

/**
 * @brief Structure holding the PWM configs of each RGB group
 */
typedef struct
{
    low_power_pwm_config_t red_config;
    low_power_pwm_config_t green_config;
    low_power_pwm_config_t blue_config;
}drv_led_lpp_rgb_config_t;

/**
 * @brief Structure holding the PWM timers of each RGB group
 */
typedef struct
{
    const app_timer_id_t  * p_red_timer;
    const app_timer_id_t  * p_green_timer;
    const app_timer_id_t  * p_blue_timer;
}drv_led_lpp_rgb_timers_t;

#define DRV_LED_LPP_RGB_INIT() {0}

#define DRV_LED_LPP_RGB_CONFIG_INIT(R,G,B,IS_ACTIVE_HIGH,P_TIMERS) \
{                                                                              \
    .red_config = LOW_POWER_PWM_DEFAULT_CONFIG((1UL << R)) ,                     \
    .red_config.active_high = IS_ACTIVE_HIGH ,                                 \
    .red_config.p_timer_id = (P_TIMERS)->p_red_timer ,                         \
    .green_config = LOW_POWER_PWM_DEFAULT_CONFIG((1UL << G)) ,                   \
    .green_config.active_high = IS_ACTIVE_HIGH ,                               \
    .green_config.p_timer_id = (P_TIMERS)->p_green_timer ,                     \
    .blue_config = LOW_POWER_PWM_DEFAULT_CONFIG((1UL << B)) ,                    \
    .blue_config.active_high = IS_ACTIVE_HIGH ,                                \
    .blue_config.p_timer_id = (P_TIMERS)->p_blue_timer                         \
}

/**
 * @brief Function for initializing RGB LED and associated PWM drivers.
 *
 * @param[in]   p_rgb            Pointer to a struct of 3 (R,G,B) PWM instances
 * @param[in]   p_rgb_config     Pointer to a struct of 3 (R,G,B) PWM configs
 *
 * @retval Values returned by @ref low_power_pwm_init
 */
ret_code_t drv_led_rgb_init(drv_led_lpp_rgb_t * p_rgb,
                            drv_led_lpp_rgb_config_t * p_rgb_config);

/**
 * @brief Function for setting the RGB value of an LED
 *
 * @param[in]   p_rgb      Pointer to a struct of 3 (R,G,B) PWM instances
 * @param[in]   rgb_val    RGB value in the 3 LSBs.
 *
 * @retval Values returned by @ref low_power_pwm_duty_set
 * @retval Values returned by @ref low_power_pwm_start
 * @retval NRF_ERROR_INVALID_PARAM   If the MSB is not 00.
 */
ret_code_t drv_led_rgb_set(drv_led_lpp_rgb_t * p_rgb, uint32_t rgb_val);

/**
 * @brief Function for stopping the PWM instnaces to reduce power consumption when
 *        LED is not in use for prolonged periods
 *
 * @param[in]   p_rgb      Pointer to a struct of 3 (R,G,B) PWM instances
 *
 * @retval Values returned by @ref low_power_pwm_stop
 */
ret_code_t drv_led_rgb_stop_pwm(drv_led_lpp_rgb_t * p_rgb);


/**
 * @brief Function for initializing a single LED
 */
void drv_led_single_init(uint8_t pin);

/**
 * @brief Function for turning on a LED
 */
void drv_led_on(uint8_t pin);

/**
 * @brief Function for turning off a LED
 */
void drv_led_off(uint8_t pin);

/**
 * @brief Function for toggling a LED
 */
void drv_led_toggle(uint8_t pin);

#endif /*__DRV_LED_H__*/
