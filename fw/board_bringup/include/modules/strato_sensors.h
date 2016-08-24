#ifndef STRATO_SENSORS_H
#define STRATO_SENSORS_H

#include "drv_pressure.h"
#include "app_error.h"



typedef struct
{
    int16_t current;
    int16_t max;
    int16_t vertical_velocity;
} strato_altitude_data_t;

typedef void (*altitude_data_cb_t)(strato_altitude_data_t * p_data);
typedef void (*acceleration_data_cb_t)(int16_t x, int16_t y, int16_t z);

typedef struct
{
    altitude_data_cb_t altitude;
    acceleration_data_cb_t accel;
} strato_sensor_data_cb_t;


ret_code_t strato_sensors_init(altitude_data_cb_t altitude_cb, acceleration_data_cb_t accel_cb);

ret_code_t strato_altitude_sample_period_set( uint16_t ms );

ret_code_t strato_altitude_enable( void );

ret_code_t strato_altitude_disable(void);

ret_code_t strato_altitude_gnd_zero (void);

#endif /*STRATO_SENSORS_H*/
