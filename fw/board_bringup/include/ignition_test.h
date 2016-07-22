#ifndef IGNITION_TEST_H
#define IGNITION_TEST_H

#include "app_error.h"

void ignition_init(void);

ret_code_t ignition_read_cap(void);

void ignition_dump_cap(bool state);

void ignition_trigger_on(uint8_t channel);

void ignition_trigger_off(uint8_t channel);

#endif /*IGNITION_TEST_H
