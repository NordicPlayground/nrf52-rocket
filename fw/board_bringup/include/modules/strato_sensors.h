#ifndef STRATO_SENSORS_H
#define STRATO_SENSORS_H

#include "drv_pressure.h"
#include "app_error.h"

typedef void (*altitude_data_cb_t)(float altitude);
typedef void (*acceleration_data_cb_t)(int16_t x, int16_t y, int16_t z);

typedef struct
{
    altitude_data_cb_t altitude;
    acceleration_data_cb_t accel;
} strato_sensor_data_cb_t;

ret_code_t strato_sensors_init(altitude_data_cb_t altitude_cb, acceleration_data_cb_t accel_cb);

ret_code_t strato_altitude_sample_freq_set( uint16_t ms );

ret_code_t strato_altitude_enable( uint16_t ms );

ret_code_t strato_altitude_disable(void);

ret_code_t strato_altitude_gnd_zero (void);

#endif /*STRATO_SENSORS_H*/
