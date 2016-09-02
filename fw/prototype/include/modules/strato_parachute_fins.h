#ifndef STRATO_PARACHUTE_FINS_H
#define STRATO_PARACHUTE_FINS_H

#include "app_error.h"

typedef struct
{
    uint8_t fin1;
    uint8_t fin2;
    uint8_t fin3;
    uint8_t fin4;
} fin_degrees_t; //in degrees from 0 to 180

void parachute_fins_init(void);

void parachute_hatch_open(void);

void parachute_hatch_close(void);

void parachute_end_values_set(uint8_t deg_open, uint8_t deg_closed); //open and closed values

void fin_values_set( fin_degrees_t * p_fin_deg );

void fin_disable(void);



#endif //STRATO_PARACHUTE_H
