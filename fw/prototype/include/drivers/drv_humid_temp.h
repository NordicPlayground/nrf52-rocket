#ifndef DRV_HUMID_TEMP_H
#define DRV_HUMID_TEMP_H

#include "nrf_drv_twi.h"

/**
 * @brief Temp-Humidity driver status types.
 */
typedef enum
{
    DRV_HUMID_TEMP_STATUS_DATA_READY,  /**<ADC value ready to be read*/
    DRV_HUMID_TEMP_STATUS_BUS_ERROR     /**<Error on the I2C Bus */
} drv_humid_temp_status_t;

/**
 * @brief Temp-Humidity driver sensor types.
 */
typedef enum
{
    SENSOR_TYPE_TEMPERATURE,
    SENSOR_TYPE_HUMIDITY
} sensor_type_t;

/**
 * @brief Temp-Humidity driver event types.
 */
typedef struct
{
    drv_humid_temp_status_t status;
    sensor_type_t           type;
} drv_humid_temp_evt_t;

/**
 * @brief Temp-Humidity driver event handler.
 */
typedef void (* drv_humid_temp_evt_handler_t)(drv_humid_temp_evt_t const * p_event);

typedef struct
{
    nrf_drv_twi_t const          *  p_twi_instance;
    drv_humid_temp_evt_handler_t    evt_handler;
} drv_humid_temp_init_t;

/**
 * @brief Init Humidity-Temp driver
 * @param[in]   p_init   pointer to init struct
 * @retval  NRF_SUCESS if init successful, otherwise an error
 */
ret_code_t drv_humid_temp_init(drv_humid_temp_init_t const * p_init);

ret_code_t drv_humid_temp_sample_begin( sensor_type_t type );

float drv_humid_temp_temperature_get( void );

uint8_t drv_humid_temp_humidity_get( void );


#endif /*DRV_HUMID_TEMP_H*/
