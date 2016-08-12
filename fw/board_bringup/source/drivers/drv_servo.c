#include "drv_servo.h"
#include "nrf_drv_pwm.h"
#include "nrf_drv_clock.h"
#include "pca20027.h"

void motor_testing(void);
void pwm_event_handler (nrf_drv_pwm_evt_type_t event_type);
void pwm_start(void);
void pwm_init(void);
void pwm_update(void);

//int pwm_scaler = (UINT16_MAX / PWM_TOP );
float pwm_scaler = (65535.0/PWM_TOP);

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
static motor_values current_values;
static nrf_pwm_values_common_t pwm_values[] =
    {
        0,/*0,0,0,*/
    };

static  nrf_pwm_sequence_t const pwm_seq =
    {
        .values.p_common = pwm_values,
        .length          = NRF_PWM_VALUES_LENGTH(pwm_values),
        .repeats         = 0,
        .end_delay       = 0
    };

void motor_init(void){

    memset(&current_values,0,sizeof(current_values));
    pwm_init();
    pwm_start();
}

void motor_values_update(motor_values values){
    // save these values, and update them using the pwm event handler
    current_values = values;

    //scale to pwm size
    current_values.motor1 = current_values.motor1 / pwm_scaler;
    // current_values.motor2 = current_values.motor2 / pwm_scaler;
    // current_values.motor3 = current_values.motor3 / pwm_scaler;
    // current_values.motor4 = current_values.motor4 / pwm_scaler;

    pwm_update();
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
            // SERVO_2,             // channel 1
            // SERVO_3,             // channel 2
            // SERVO_4,             // channel 3
        },
        .base_clock = NRF_PWM_CLK_1MHz,
        .count_mode = NRF_PWM_MODE_UP,
        .top_value  = PWM_TOP,
        .load_mode  = NRF_PWM_LOAD_COMMON,
        .step_mode  = NRF_PWM_STEP_AUTO
    };
    err_code = nrf_drv_pwm_init(&m_pwm0, &config0, NULL);
    APP_ERROR_CHECK(err_code);
}

void pwm_start(void){
    nrf_drv_pwm_simple_playback(&m_pwm0, &pwm_seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

void pwm_update(void){

    // | with 0x8000 to invert pwm signal
    uint16_t temp_value = 0;
    temp_value= (uint16_t)current_values.motor1;
    pwm_values[0] = temp_value | 0x8000 ;
    // temp_value= (uint16_t)current_values.motor2;
    // pwm_values[1] = temp_value | 0x8000 ;
    // temp_value= (uint16_t)current_values.motor3;
    // pwm_values[2] = temp_value | 0x8000 ;
    // temp_value= (uint16_t)current_values.motor4;
    // pwm_values[3] = temp_value | 0x8000 ;

    pwm_start();

}

// not used
void pwm_event_handler (nrf_drv_pwm_evt_type_t event_type){

    if (event_type == NRF_DRV_PWM_EVT_FINISHED){
        pwm_update();
    }
}

uint32_t motor_stop(void){
    uint32_t err_code;
    err_code = nrf_drv_pwm_stop	(&m_pwm0,false);
    return err_code;
}
