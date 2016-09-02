#ifndef STRATO_LED_H
#define STRATO_LED_H

#include "app_error.h"

ret_code_t leds_init(void);

ret_code_t leds_set_rgb(uint32_t rgb);

ret_code_t leds_off(void);

uint32_t leds_current_value_get(void);

#endif /*STRATO_LED_H*/
