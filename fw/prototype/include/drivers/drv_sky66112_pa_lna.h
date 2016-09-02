#ifndef DRV_SKY66112_PA_LNA_H__
#define DRV_SKY66112_PA_LNA_H__

#include "ble.h"
#include "app_error.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"

typedef enum
{
    PA_LNA_ANT1,
    PA_LNA_ANT2
} pa_lna_ant_t;

//Set up softdevice's PA/LNA assist
void drv_sky66112_init(uint32_t pa_gpio, uint32_t lna_gpio, pa_lna_ant_t default_ant);

void drv_sky66112_ant_select(pa_lna_ant_t ant);

void drv_sky66112_tx_high_power(void);

void drv_sky66112_tx_low_power(void);

void drv_sky66112_rx_lna(void);

void drv_sky66112_sleep(void);

void drv_sky66112_bypass(void);

#endif /*DRV_SKY66112_PA_LNA_H__*/
