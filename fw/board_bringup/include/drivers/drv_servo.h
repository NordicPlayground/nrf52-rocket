#ifndef DRV_SERVO_H
#define DRV_SERVO_H


#define PWM_TOP 20200
//BASE CLOCK FREQ/DESIRED FREQUENCY = PWM_TOP

#include <stdint.h>

typedef struct {
    float motor1;
    float motor2;
    float motor3;
    float motor4;
} servo_values_t ;
//Desired pulse width = (motorN*(PWM_TOP)/(2^16)) / BASE CLOCK FREQ

void servo_init(void);

void servo0_values_update(servo_values_t values); //parachute servo
void servo1_values_update(servo_values_t values); //fin servos (4 servos in 1 pwm instance)

uint32_t servo0_stop(void);
uint32_t servo1_stop(void);

#endif //DRV_SERVO_H
