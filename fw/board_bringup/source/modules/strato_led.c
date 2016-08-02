#include "strato_led.h"
#include "app_error.h"
#include "drv_led.h"
#include "app_timer.h"

static drv_led_lpp_rgb_t m_rgb = DRV_LED_LPP_RGB_INIT();

ret_code_t leds_init()
{
    APP_TIMER_DEF(m_r_timer);
    APP_TIMER_DEF(m_g_timer);
    APP_TIMER_DEF(m_b_timer);

    drv_led_lpp_rgb_timers_t rgb_timers = {&m_r_timer,&m_g_timer,&m_b_timer};
    drv_led_lpp_rgb_config_t rgb_config = DRV_LED_LPP_RGB_CONFIG_INIT(LED_R,
                                                                      LED_G,
                                                                      LED_B,
                                                                      false,
                                                                      &rgb_timers);

    return drv_led_rgb_init(&m_rgb,&rgb_config);

}

ret_code_t leds_set_rgb(uint32_t rgb)
{
    return drv_led_rgb_set(&m_rgb, rgb);
}

ret_code_t leds_off(void)
{
    return drv_led_rgb_stop_pwm(&m_rgb);
}
