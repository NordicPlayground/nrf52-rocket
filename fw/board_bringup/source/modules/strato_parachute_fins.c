#include "strato_parachute_fins.h"
#include "drv_servo.h"
#include "app_error.h"



uint8_t m_deg_open = 90; //in degrees for servo position
uint8_t m_deg_closed = 0;


//PWM values

servo_values_t parachute_values =
{
    .motor1 = 0,
};

servo_values_t fin_values =
{
    .motor1 = 0,
    .motor2 = 0,
    .motor3 = 0,
    .motor4 = 0,
};

void parachute_fins_init(void)
{
    servo_init();
};

static float servo_degrees_to_pwm_val( uint8_t servo_degrees )
{
    //Applicable for :
    // .base_clock = NRF_PWM_CLK_1MHz,
    // .count_mode = NRF_PWM_MODE_UP,
    // float pwm_scaler = (65535.0/PWM_TOP);

    //Pulse widths
    //180 degrees = 0.54 ms
    //0 degrees = 2.4 ms
    if (servo_degrees <= 180)
    {
        float servo_min = 1000000*0.00054*(65535/((float)PWM_TOP));
        float servo_max = 1000000*0.0024*(65535/((float)PWM_TOP));

        return ((float)(servo_degrees)/(180.0f))*(servo_max-servo_min) + servo_min;
    }

    else
    {
        return 0;
    }
}

void parachute_hatch_open(void)
{
    parachute_values.motor1 = servo_degrees_to_pwm_val(m_deg_open);
    servo0_values_update(parachute_values);
}

void parachute_hatch_close(void)
{
    parachute_values.motor1 = servo_degrees_to_pwm_val(m_deg_closed);
    servo0_values_update(parachute_values);
}

void parachute_end_values_set(uint8_t deg_open, uint8_t deg_closed)
{
    if (deg_open <= 180)
    {
        m_deg_open = deg_open;
    }

    if (deg_closed <= 180)
    {
        m_deg_closed = deg_closed;
    }
}

void fin_values_set(fin_degrees_t * p_fin_deg)
{
    fin_values.motor1 = servo_degrees_to_pwm_val(p_fin_deg->fin1);
    fin_values.motor2 = servo_degrees_to_pwm_val(p_fin_deg->fin2);
    fin_values.motor3 = servo_degrees_to_pwm_val(p_fin_deg->fin3);
    fin_values.motor4 = servo_degrees_to_pwm_val(p_fin_deg->fin4);
    servo1_values_update(fin_values);
}

void fin_disable(void)
{
    ret_code_t err_code;
    err_code = servo1_stop();
    APP_ERROR_CHECK(err_code);
}
