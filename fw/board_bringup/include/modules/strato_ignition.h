#ifndef STRATO_IGNITION_H
#define STRATO_IGNITION_H

#include "app_error.h"
#include "nrf_drv_saadc.h"

typedef void (*ignition_adc_evt_handler_t) (double result);
typedef struct
{
    ignition_adc_evt_handler_t adc_evt_handler;
    uint32_t                   adc_sampling_period_ms;
} ignition_init_t;

void ignition_init(ignition_init_t * p_params);

ret_code_t ignition_cap_adc_sample_begin(void);

ret_code_t ignition_cap_adc_sample_end(void);

void power_5v_enable(bool state);

void ignition_dump_cap(bool state);

/* invalid state if voltage is too low */
ret_code_t ignition_trigger_on(uint8_t channel);

void ignition_trigger_off(uint8_t channel);

#endif /*ROCKETRY_IGNITION_H*/
