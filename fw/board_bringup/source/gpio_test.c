#include "gpio_test.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "SEGGER_RTT.h"

void gpio_test_all_toggle(void)
{
    nrf_gpio_range_cfg_output(0, 31);
    for (uint8_t pin = 0; pin < 32; pin++)
    {
        nrf_gpio_pin_set(pin);
        nrf_delay_ms(100);
        nrf_gpio_pin_clear(pin);
        nrf_delay_ms(100);
    }

}

void gpio_test_all_read(void)
{
    nrf_gpio_range_cfg_input(0, 31, NRF_GPIO_PIN_PULLUP);
    for (uint8_t pin = 0; pin < 32; pin++)
    {
        if(nrf_gpio_pin_read(pin) == 0)
        {
            SEGGER_RTT_printf(0, "Pin %d is LOW\r\n", pin);
        }
    }
}
