#include "drv_mpu9250.h"
#include "nrf_error.h"
#include "nrf_drv_gpiote.h"
#include "pca20027.h"
#include "app_timer.h"
// #include "mltypes.h"
#include "app_scheduler.h"
#include <string.h>

/**@brief Check if the error code is equal to NRF_SUCCESS, if not return the error code.
 */
#define RETURN_IF_ERROR(PARAM)                                                                    \
        if ((PARAM) != NRF_SUCCESS)                                                               \
        {                                                                                         \
            return (PARAM);                                                                       \
        }

/**@brief Check if the input pointer is NULL, if so it returns NRF_ERROR_NULL.
 */
#define NULL_PARAM_CHECK(PARAM)                                                                   \
        if ((PARAM) == NULL)                                                                      \
        {                                                                                         \
            return NRF_ERROR_NULL;                                                                \
        }


static struct
{
    drv_mpu9250_init_t        init;
    void                      (*cb)(void);
    bool                      initialized;
    bool                      int_registered;
} m_mpu9250 = {.initialized = false, .int_registered = false};

static void gpiote_evt_sceduled(void * p_event_data, uint16_t event_size)
{
    m_mpu9250.cb();
}

static void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint32_t err_code;

    if ((pin == MPU_INT) && (action == NRF_GPIOTE_POLARITY_HITOLO) && m_mpu9250.int_registered)
    {
        err_code = app_sched_event_put(0, 0, gpiote_evt_sceduled);
        APP_ERROR_CHECK(err_code);
    }
}

/**
 * @brief Function to init / allocate the TWI module
 */
static __inline uint32_t twi_open(void)
{
    uint32_t err_code;

    err_code = nrf_drv_twi_init(m_mpu9250.init.p_twi_instance,
                                m_mpu9250.init.p_twi_cfg,
                                NULL,
                                NULL);
    RETURN_IF_ERROR(err_code);

    nrf_drv_twi_enable(m_mpu9250.init.p_twi_instance);

    return NRF_SUCCESS;
}

/**
 * @brief Function to deinit the TWI module when this driver does not need to
 *        communicate on the TWI bus, so that other drivers can use the module
 */
static __inline uint32_t twi_close(void)
{
    nrf_drv_twi_disable(m_mpu9250.init.p_twi_instance);

    nrf_drv_twi_uninit(m_mpu9250.init.p_twi_instance);

    return NRF_SUCCESS;
}

int drv_mpu9250_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data)
{
    uint32_t err_code;
    uint8_t buffer[length+1];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], data, length);

    err_code = twi_open();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_tx( m_mpu9250.init.p_twi_instance,
                               slave_addr,
                               buffer,
                               length + 1,
                               false);
    APP_ERROR_CHECK(err_code);

    err_code = twi_close();
    APP_ERROR_CHECK(err_code);

    return 0;
}

int drv_mpu9250_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
    uint32_t err_code;

    err_code = twi_open();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_tx( m_mpu9250.init.p_twi_instance,
                               slave_addr,
                               &reg_addr,
                               1,
                               true );
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_twi_rx( m_mpu9250.init.p_twi_instance,
                               slave_addr,
                               data,
                               length );
    APP_ERROR_CHECK(err_code);

    err_code = twi_close();
    APP_ERROR_CHECK(err_code);

    return 0;
}

int drv_mpu9250_ms_get(unsigned long *count)
{
//    uint32_t ticks;
//    uint32_t err_code;

//    err_code = app_timer_cnt_get(&ticks);
//    APP_ERROR_CHECK(err_code);

//    *count = (ticks * (APP_TIMER_PRESCALER + 1) * 1000) / APP_TIMER_CLOCK_FREQ;

    return 0;
}

int drv_mpu9250_int_register(struct int_param_s *int_param)
{
//    uint32_t err_code;

//    if (!m_mpu9250.int_registered)
//    {
//        m_mpu9250.int_registered = true;
//        m_mpu9250.cb = int_param->cb;

//        /* Configure gpiote for the sensors data ready interrupt */
//        if (!nrf_drv_gpiote_is_init())
//        {
//            err_code = nrf_drv_gpiote_init();
//            APP_ERROR_CHECK(err_code);
//        }

//        nrf_drv_gpiote_in_config_t gpiote_in_config;
//        gpiote_in_config.is_watcher  = false;
//        gpiote_in_config.hi_accuracy = false;
//        gpiote_in_config.pull        = NRF_GPIO_PIN_PULLUP;
//        gpiote_in_config.sense       = NRF_GPIOTE_POLARITY_HITOLO;
//        err_code = nrf_drv_gpiote_in_init(MPU_INT, &gpiote_in_config, gpiote_evt_handler);
//        APP_ERROR_CHECK(err_code);

//        nrf_drv_gpiote_in_event_enable(MPU_INT, true);
//    }

    return 0;
}

uint32_t drv_mpu9250_init(drv_mpu9250_init_t * p_params)
{
    NULL_PARAM_CHECK(p_params);

    m_mpu9250.init.p_twi_cfg      = p_params->p_twi_cfg;
    m_mpu9250.init.p_twi_instance = p_params->p_twi_instance;
    m_mpu9250.initialized = true;

    return NRF_SUCCESS;
}
