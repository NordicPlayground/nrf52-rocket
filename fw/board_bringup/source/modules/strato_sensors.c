#include "drv_pressure.h"
#include "nrf_drv_twi.h"
#include "pca20027.h"
#include "strato_sensors.h"
#include "strato_app_config.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "app_scheduler.h"

/**@brief Check if the error code is equal to NRF_SUCCESS, if not return the error code.
 */
#define RETURN_IF_ERROR(PARAM)                                                                    \
        if ((PARAM) != NRF_SUCCESS)                                                               \
        {                                                                                         \
            return (PARAM);                                                                       \
        }

APP_TIMER_DEF(altitutde_timer_id);
APP_TIMER_DEF(accel_timer_id);


static strato_sensor_data_cb_t m_sensor_cb;

static strato_altitude_data_t m_alti_data =
{
    .current = 0,
    .max = 0,
    .vertical_velocity = 0,
    .max_vertical_velocity = 0
};

static int16_t m_ground_level = 0;

static bool  m_ground_level_calib_flag = false;
static uint16_t m_altitude_period = ALTITUDE_SAMPLE_PERIOD_MS; //ms

static void press_evt_handler(drv_pressure_evt_t const * p_evt,
                              void *                     p_context)
{
    ret_code_t err_code;
    drv_pressure_mode_t mode = (*((drv_pressure_mode_t*)p_context));

    switch (*p_evt)
    {
        case DRV_PRESSURE_EVT_DATA_RDY:
        {
            if (mode == DRV_PRESSURE_MODE_BAROMETER)
            {
                //drv_pressure_get()

            }
            else if (mode == DRV_PRESSURE_MODE_ALTIMETER)
            {
                if (m_ground_level_calib_flag == true)
                {
                    m_ground_level = drv_pressure_altitude_int_get();
                    m_ground_level_calib_flag = false;

                    err_code = strato_altitude_disable();
                    APP_ERROR_CHECK(err_code);
                }

                else
                {
                    int16_t new_alti = drv_pressure_altitude_int_get() - m_ground_level;
                    int16_t delta_alti = new_alti - m_alti_data.current;

                    if (new_alti > m_alti_data.max)
                    {
                        m_alti_data.max = new_alti;
                    }
                    m_alti_data.current = new_alti;

                    int16_t new_velo = delta_alti*1000/m_altitude_period;
                    if (new_velo > m_alti_data.max_vertical_velocity)
                    {
                        m_alti_data.max_vertical_velocity = new_velo;
                    }

                    m_alti_data.vertical_velocity = new_velo;
                    m_sensor_cb.altitude(&m_alti_data);
                }
            }
        }
        break;

        case DRV_PRESSURE_EVT_BUS_ERROR:
            break;

        default:
            break;
    }
}
//
// static void humid_temp_evt_handler(drv_humid_temp_evt_t const * p_event)
// {
//     SEGGER_RTT_printf(0,"Humidity: %d", drv_humid_temp_humidity_get());
// }


static void altitude_timeout_handler(void * p_context)
{
   uint32_t err_code;

   err_code = drv_pressure_sample();
   APP_ERROR_CHECK(err_code);
}

static const  nrf_drv_twi_t m_twi_sensors = NRF_DRV_TWI_INSTANCE(0);

ret_code_t strato_sensors_init(altitude_data_cb_t altitude_cb, acceleration_data_cb_t accel_cb)
{
    //Pressure Sensor Init
    ret_code_t err_code;

    drv_pressure_init_t press_init;
    press_init.p_twi_instance = &m_twi_sensors;
    press_init.pressure_evt_handler = press_evt_handler;
    press_init.mode = DRV_PRESSURE_MODE_ALTIMETER;
    err_code = drv_pressure_init(&press_init);
    RETURN_IF_ERROR(err_code);

    m_sensor_cb.altitude = altitude_cb;
    m_sensor_cb.accel = accel_cb;

    err_code = app_timer_create(&altitutde_timer_id, APP_TIMER_MODE_REPEATED, altitude_timeout_handler);
    RETURN_IF_ERROR(err_code);

    // err_code = app_timer_create(&accel_timer_id, APP_TIMER_MODE_REPEATED, accel_timeout_handler);
    // RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;

}

ret_code_t strato_altitude_sample_period_set( uint16_t ms )
{
    m_altitude_period = ms;
    return strato_altitude_enable();
}

ret_code_t strato_altitude_enable( void )
{
    ret_code_t err_code;

    //Disable it first to make sure no double enables
    err_code = strato_altitude_disable();
    RETURN_IF_ERROR(err_code);

    err_code = drv_pressure_enable();
    APP_ERROR_CHECK(err_code);

    if (m_altitude_period <= 11)
    {
        err_code = drv_pressure_oversampling_rate_set(0);
    }
    else if (m_altitude_period <= 19 && m_altitude_period > 11 )
    {
        err_code = drv_pressure_oversampling_rate_set(1);
    }
    else if (m_altitude_period <= 35 && m_altitude_period > 19 )
    {
        err_code = drv_pressure_oversampling_rate_set(2);
    }
    else if (m_altitude_period <= 67 && m_altitude_period > 35 )
    {
        err_code = drv_pressure_oversampling_rate_set(3);
    }
    else if (m_altitude_period <= 131 && m_altitude_period > 35 )
    {
        err_code = drv_pressure_oversampling_rate_set(4);
    }
    else if (m_altitude_period <= 259 && m_altitude_period > 131 )
    {
        err_code = drv_pressure_oversampling_rate_set(5);
    }
    else if (m_altitude_period <= 513 && m_altitude_period > 259 )
    {
        err_code = drv_pressure_oversampling_rate_set(6);
    }
    else if (m_altitude_period > 513)
    {
        err_code = drv_pressure_oversampling_rate_set(7);
    }

    APP_ERROR_CHECK(err_code);


    err_code = drv_pressure_sample();
    APP_ERROR_CHECK(err_code);

    return app_timer_start(altitutde_timer_id,
                       APP_TIMER_TICKS(m_altitude_period, APP_TIMER_PRESCALER),
                       NULL);
}

ret_code_t strato_altitude_disable(void)
{
    uint32_t err_code;

    err_code = app_timer_stop(altitutde_timer_id);
    RETURN_IF_ERROR(err_code);

    return drv_pressure_disable();
}

ret_code_t strato_altitude_gnd_zero (void)
{
    ret_code_t err_code;

    err_code = drv_pressure_enable();
    APP_ERROR_CHECK(err_code);

    err_code = drv_pressure_oversampling_rate_set(7); //max osr for best resolution
    APP_ERROR_CHECK(err_code);

    m_ground_level_calib_flag = true;

    return drv_pressure_sample();

}
