#include "drv_servo.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_clock.h"
#include "pca20027.h"
#include <string.h>

void pwm_event_handler (nrf_drv_pwm_evt_type_t event_type);
void pwm_init(void);
void pwm0_start(void);
void pwm0_update(void);
void pwm1_start(void);
void pwm1_update(void);

//int pwm_scaler = (UINT16_MAX / PWM_TOP );
float pwm_scaler = (65535.0/PWM_TOP);

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0); //Parachute Servo
static nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1); //Fin Servos

static servo_values_t current_values0;
static servo_values_t current_values1;

static nrf_pwm_values_common_t pwm0_values[] =
    {
        0,/*0,0,0,*/
    };

static nrf_pwm_values_common_t pwm1_values[] =
    {
        0,0,0,0
    };

static  nrf_pwm_sequence_t const pwm0_seq =
    {
        .values.p_common = pwm0_values,
        .length          = NRF_PWM_VALUES_LENGTH(pwm0_values),
        .repeats         = 0,
        .end_delay       = 0
    };

static  nrf_pwm_sequence_t const pwm1_seq =
    {
        .values.p_common = pwm1_values,
        .length          = NRF_PWM_VALUES_LENGTH(pwm1_values),
        .repeats         = 0,
        .end_delay       = 0
    };

void servo_init(void){

    memset(&current_values0,0,sizeof(current_values0));
    memset(&current_values1,0,sizeof(current_values1));

    pwm_init();
    pwm0_start();
    pwm1_start();
}

void servo0_values_update(servo_values_t values){
    // save these values, and update them using the pwm event handler
    current_values0 = values;

    //scale to pwm size
    current_values0.motor1 = current_values0.motor1 / pwm_scaler;
    // current_values1.motor2 = current_values1.motor2 / pwm_scaler;
    // current_values1.motor3 = current_values1.motor3 / pwm_scaler;
    // current_values1.motor4 = current_values1.motor4 / pwm_scaler;

    pwm0_update();
}

void servo1_values_update(servo_values_t values){
    // save these values, and update them using the pwm event handler
    current_values1 = values;

    //scale to pwm size
    current_values1.motor1 = current_values1.motor1 / pwm_scaler;
    current_values1.motor2 = current_values1.motor2 / pwm_scaler;
    current_values1.motor3 = current_values1.motor3 / pwm_scaler;
    current_values1.motor4 = current_values1.motor4 / pwm_scaler;

    pwm1_update();
}

void pwm_init(void){
    uint32_t err_code;
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            SERVO_1,             // channel 0
            NRF_DRV_PWM_PIN_NOT_USED,
            NRF_DRV_PWM_PIN_NOT_USED,
            NRF_DRV_PWM_PIN_NOT_USED,
        },
        .base_clock = NRF_PWM_CLK_1MHz,
        .count_mode = NRF_PWM_MODE_UP,
        .top_value  = PWM_TOP,
        .load_mode  = NRF_PWM_LOAD_COMMON,
        .step_mode  = NRF_PWM_STEP_AUTO
    };

    nrf_drv_pwm_config_t const config1 =
    {
        .output_pins =
        {
            SERVO_2,           // channel 0
            SERVO_3,           // channel 1
            SERVO_4,           // channel 2
            SERVO_5,           // channel 3
        },
        .base_clock = NRF_PWM_CLK_1MHz,
        .count_mode = NRF_PWM_MODE_UP,
        .top_value  = PWM_TOP,
        .load_mode  = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode  = NRF_PWM_STEP_AUTO
    };

    err_code = nrf_drv_pwm_init(&m_pwm0, &config0, NULL);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_pwm_init(&m_pwm1, &config1, NULL);
    APP_ERROR_CHECK(err_code);
}

void pwm0_start(void){
    nrf_drv_pwm_simple_playback(&m_pwm0, &pwm0_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

void pwm1_start(void){
    nrf_drv_pwm_simple_playback(&m_pwm1, &pwm1_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

void pwm0_update(void){

    // | with 0x8000 to invert pwm signal
    uint16_t temp_value = 0;
    temp_value= (uint16_t)current_values0.motor1;
    pwm0_values[0] = temp_value | 0x8000 ;

    pwm0_start();

}

void pwm1_update(void){

    // | with 0x8000 to invert pwm signal
    uint16_t temp_value = 0;
    temp_value= (uint16_t)current_values1.motor1;
    pwm1_values[0] = temp_value | 0x8000 ;
    temp_value= (uint16_t)current_values1.motor2;
    pwm1_values[1] = temp_value | 0x8000 ;
    temp_value= (uint16_t)current_values1.motor3;
    pwm1_values[2] = temp_value | 0x8000 ;
    temp_value= (uint16_t)current_values1.motor4;
    pwm1_values[3] = temp_value | 0x8000 ;

    pwm1_start();

}

// not used
void pwm_event_handler (nrf_drv_pwm_evt_type_t event_type){

    if (event_type == NRF_DRV_PWM_EVT_FINISHED){

    }
}

uint32_t servo0_stop(void){
    uint32_t err_code;
    err_code = nrf_drv_pwm_stop	(&m_pwm0,false);
    return err_code;
}

uint32_t servo1_stop(void){
    uint32_t err_code;
    err_code = nrf_drv_pwm_stop	(&m_pwm1,false);
    return err_code;
}
