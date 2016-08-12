#ifndef STRATO_PARACHUTE_H
#define STRATO_PARACHUTE_H

#include "app_error.h"

void parachute_init(void);

void parachute_hatch_open(void);

void parachute_hatch_close(void);

void parachute_end_values_set(uint8_t deg_open, uint8_t deg_closed); //open and closed values



#endif //STRATO_PARACHUTE_H
