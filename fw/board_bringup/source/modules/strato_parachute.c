#include "strato_parachute.h"
#include "drv_servo.h"



uint8_t m_deg_open = 90; //in degrees for servo position
uint8_t m_deg_closed = 0;

motor_values values =
{
    .motor1 = 0,
};

void parachute_init(void)
{
    motor_init();
};

static float servo_degrees_to_pwm_val( int8_t servo_degrees )
{
    //Applicable for :
    // .base_clock = NRF_PWM_CLK_1MHz,
    // .count_mode = NRF_PWM_MODE_UP,
    // .top_value  = PWM_TOP = 16666 (60Hz)
    // float pwm_scaler = (65535.0/PWM_TOP);

    //180 degrees = 10000
    //0 degrees = 2275
    //map servo_degrees from 0 to 180 -> 2275 to 10000
    return ((float)(servo_degrees)/(180.0f))*(10000-2275) + 2275;

}

void parachute_hatch_open(void)
{
    values.motor1 = servo_degrees_to_pwm_val(m_deg_open);
    motor_values_update(values);
}

void parachute_hatch_close(void)
{
    values.motor1 = servo_degrees_to_pwm_val(m_deg_closed);
    motor_values_update(values);
}

void parachute_end_values_set(uint8_t deg_open, uint8_t deg_closed)
{
    m_deg_open = deg_open;
    m_deg_closed = deg_closed;
}
