#include "drv_pressure.h"
#include "nrf_drv_twi.h"
#include "pca20027.h"

static const nrf_drv_twi_config_t m_twi_config =
{
    .scl = TWI_SCL_PIN_NUMBER,
    .sda = TWI_SDA_PIN_NUMBER,
    .frequency = NRF_TWI_FREQ_400K,
    .interrupt_priority = APP_IRQ_PRIORITY_LOW
};
