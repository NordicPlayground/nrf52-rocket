#include "drv_humid_temp.h"
#include "app_timer.h"
#include "drv_htu21d.h"

#define APP_TIMER_PRESCALER     (0)

//ADC wait time depends on desired resolution, see HTU21D spec sheet
#define ADC_HUMID_WAIT_TIME     (APP_TIMER_TICKS(16, APP_TIMER_PRESCALER))
#define ADC_TEMP_WAIT_TIME      (APP_TIMER_TICKS(50, APP_TIMER_PRESCALER))

#define RETURN_IF_ERROR(PARAM)                                                                    \
        if ((PARAM) != NRF_SUCCESS)                                                               \
        {                                                                                         \
            return (PARAM);                                                                       \
        }

typedef struct
{
    uint32_t humid_conv_ticks;
    uint32_t temp_conv_ticks;
} adc_conv_ticks_t;

/**callback to the application when data is ready to be read via
@ref drv_humid_temp_temperature_get or @ref drv_humid_temp_humidity_get */
drv_humid_temp_evt_handler_t m_evt_cb;

APP_TIMER_DEF(m_temp_conv_timer_id);
APP_TIMER_DEF(m_humid_conv_timer_id);

static void temp_conv_wait_timeout_handler( void * p_context )
{
    drv_humid_temp_evt_t evt = {
        .status = DRV_HUMID_TEMP_STATUS_DATA_READY,
        .type   = SENSOR_TYPE_TEMPERATURE
    };

    m_evt_cb(&evt);
}

static void humid_conv_wait_timeout_handler( void * p_context )
{
    drv_humid_temp_evt_t evt = {
        .status = DRV_HUMID_TEMP_STATUS_DATA_READY,
        .type   = SENSOR_TYPE_HUMIDITY
    };
    m_evt_cb(&evt);
}

static ret_code_t adc_conv_wait_timers_init( void )
{
    ret_code_t err_code;
    err_code = app_timer_create(&m_temp_conv_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                temp_conv_wait_timeout_handler);
    RETURN_IF_ERROR(err_code);
    err_code = app_timer_create(&m_humid_conv_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                humid_conv_wait_timeout_handler);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

ret_code_t drv_humid_temp_init(drv_humid_temp_init_t const * p_init)
{
    ret_code_t err_code;
    m_evt_cb = p_init->evt_handler;

    const drv_htu21d_config_t config =
    {
        .adc_res = DRV_HTU21D_ADC_RES_12_14B,
        .heater_enable = false
    };

    drv_htu21d_init_t drv_init =
    {
        .p_twi_instance = p_init->p_twi_instance,
        .p_config       = &config
    };

    err_code = adc_conv_wait_timers_init();
    RETURN_IF_ERROR(err_code);

    err_code = drv_htu21d_init( &drv_init );
    return err_code;
}

ret_code_t drv_humid_temp_sample_begin( sensor_type_t type )
{
    ret_code_t err_code;
    switch ( type )
    {
        case SENSOR_TYPE_HUMIDITY:
            err_code = drv_htu21d_humid_conv_begin();
            RETURN_IF_ERROR(err_code);
            return app_timer_start(m_humid_conv_timer_id, ADC_HUMID_WAIT_TIME, NULL);

        case SENSOR_TYPE_TEMPERATURE:
            err_code = drv_htu21d_temp_conv_begin();
            RETURN_IF_ERROR(err_code);
            return app_timer_start(m_temp_conv_timer_id, ADC_TEMP_WAIT_TIME, NULL);
        
        default:
            break;
    }
    return NRF_ERROR_INVALID_PARAM;
}

float drv_humid_temp_temperature_get( void )
{
    ret_code_t err_code;
    float temp;
    err_code = drv_htu21d_temperature_get(&temp);
    if (err_code != NRF_SUCCESS)
    {
        drv_humid_temp_evt_t evt = {
            .status = DRV_HUMID_TEMP_STATUS_BUS_ERROR,
            .type   = SENSOR_TYPE_TEMPERATURE
        };
        m_evt_cb(&evt);
    }
    return temp;
}

uint8_t drv_humid_temp_humidity_get( void )
{
    ret_code_t err_code;
    uint8_t humid;
    err_code = drv_htu21d_humidity_get(&humid);
    if (err_code != NRF_SUCCESS)
    {
        drv_humid_temp_evt_t evt = {
            .status = DRV_HUMID_TEMP_STATUS_BUS_ERROR,
            .type   = SENSOR_TYPE_TEMPERATURE
        };
        m_evt_cb(&evt);
    }
    return humid;
}
