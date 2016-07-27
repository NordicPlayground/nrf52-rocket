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

 #include "drv_led.h"
 #include "nrf_gpio.h"
 #include "low_power_pwm.h"
 #include "app_error.h"

#define RETURN_IF_ERROR(x) if(x != NRF_SUCCESS) return x;
#define LED_COMP_COUNT     3

static void lp_pwm_timeout_handler(void * context)
{
    //Not Required.
}

ret_code_t drv_led_rgb_init(drv_led_lpp_rgb_t * p_rgb,
                            drv_led_lpp_rgb_config_t * p_rgb_config)
{
    ret_code_t err_code;

    /**Loop through the members (representing R,G,B components) of the structs
    to which the input pointers are pointed and init the PWM instances with their
    respective configs*/

    low_power_pwm_t * p_incr = &(p_rgb->red);
    low_power_pwm_config_t * p_incr_config = &(p_rgb_config->red_config);
    for (uint8_t i = 0; i < LED_COMP_COUNT; i++)
    {
        err_code = low_power_pwm_init(p_incr + i, p_incr_config + i, lp_pwm_timeout_handler);
        RETURN_IF_ERROR(err_code);
    }
    return NRF_SUCCESS;
}

ret_code_t drv_led_rgb_set(drv_led_lpp_rgb_t * p_rgb, uint32_t rgb_val)
{
    //First byte must be empty, expecting a 3-byte value for RGB
    if (rgb_val & 0xFF000000) {
        return NRF_ERROR_INVALID_PARAM;
    }

    ret_code_t err_code;

    uint8_t rgb_value[LED_COMP_COUNT] = {(uint8_t)(rgb_val >> 16),
                                        (uint8_t)(((rgb_val) & (0xFF00)) >> 8),
                                        (uint8_t)(rgb_val & 0xFF)};
    /** Loop through all PWM instances and set their respective values */
    low_power_pwm_t * p_incr = &(p_rgb->red);
    for (uint8_t i = 0; i < LED_COMP_COUNT; i++)
    {
        if (rgb_value[i] > 0)
        {
            err_code = low_power_pwm_duty_set(p_incr + i, rgb_value[i]);
            RETURN_IF_ERROR(err_code);
            err_code = low_power_pwm_start(p_incr + i, (p_incr + i)->bit_mask);
            RETURN_IF_ERROR(err_code);
        }
        else
        {
            err_code = low_power_pwm_stop(p_incr + i);
            RETURN_IF_ERROR(err_code);            
        }
    }
    return NRF_SUCCESS;
}

ret_code_t drv_led_rgb_stop_pwm(drv_led_lpp_rgb_t * p_rgb)
{
    ret_code_t err_code;
    low_power_pwm_t * p_incr = &(p_rgb->red);
    for (uint8_t i = 0; i < LED_COMP_COUNT; i++)
    {
        err_code = low_power_pwm_stop(p_incr + i);
        RETURN_IF_ERROR(err_code);
    }
    return NRF_SUCCESS;
}


 void __inline drv_led_single_init(uint8_t pin)
 {
     nrf_gpio_cfg_output(pin);
 }
void __inline drv_led_on(uint8_t pin)
{
    //active low
    nrf_gpio_pin_clear(pin);
}

void __inline drv_led_off(uint8_t pin)
{
    //active low
    nrf_gpio_pin_set(pin);
}

void __inline drv_led_toggle(uint8_t pin)
{
    nrf_gpio_pin_toggle(pin);
}
