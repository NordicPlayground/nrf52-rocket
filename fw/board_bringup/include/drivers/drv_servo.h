#ifndef MOTOR_H__
#define MOTOR_H__

#define PWM_TOP 16666 // 60Hz

typedef struct {
    float motor1;
    float motor2;
    float motor3;
    float motor4;
} motor_values ;


void motor_init(void);
void motor_values_update(motor_values values);

#endif //MOTOR_H__
