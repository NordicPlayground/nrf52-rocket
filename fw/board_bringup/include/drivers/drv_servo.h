#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#define PWM_TOP 16666 // 60Hz

typedef struct {
    float motor1;
    float motor2;
    float motor3;
    float motor4;
} motor_values ;


void motor_init(void);
void motor_values_update(motor_values values);

#endif //DRV_SERVO_H
