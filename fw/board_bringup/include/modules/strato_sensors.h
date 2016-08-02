#ifndef STRATO_SENSORS_H
#define STRATO_SENSORS_H

#include "drv_pressure.h"
#include "app_error.h"

typedef void (*altitude_data_cb_t)(uint32_t integer, uint8_t decimal);
typedef void (*acceleration_data_cb_t)(int16_t x, int16_t y, int16_t z);

ret_code_t sensors_init()



#endif /*STRATO_SENSORS_H*/
