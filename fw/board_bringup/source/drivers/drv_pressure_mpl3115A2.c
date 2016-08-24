#include <stdint.h>
#include "drv_pressure_mpl3115A2.h"
#include "drv_pressure.h"
#include "pca20027.h"
#include "app_util_platform.h"
#include "nrf_drv_gpiote.h"

#define DRV_PRESSURE_DEBUG

#ifdef DRV_PRESSURE_DEBUG
    #include "SEGGER_RTT.h"
    #define DEBUG_PRINTF SEGGER_RTT_printf
#else
    #define DEBUG_PRINTF(...)
#endif

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

static drv_mp3115a2_data_t m_data;

static const nrf_drv_twi_config_t twi_config =
{
    .scl                = TWI_SCL_PIN_NUMBER,
    .sda                = TWI_SDA_PIN_NUMBER,
    .frequency          = NRF_TWI_FREQ_400K,
    .interrupt_priority = APP_IRQ_PRIORITY_LOW
};

typedef drv_pressure_init_t drv_pressure_t;
static drv_pressure_t m_drv_pressure;

/**
 * @brief Function to deinit the TWI module when this driver does not need to
 *        communicate on the TWI bus, so that other drivers can use the module
 */
static __inline uint32_t twi_open(void)
{
    uint32_t err_code;

    err_code = nrf_drv_twi_init(m_drv_pressure.p_twi_instance,
                                &twi_config,
                                NULL,
                                NULL);
    RETURN_IF_ERROR(err_code);

    nrf_drv_twi_enable(m_drv_pressure.p_twi_instance);

    return NRF_SUCCESS;
}

/**
 * @brief Function to deinit the TWI module when this driver does not need to
 *        communicate on the TWI bus, so that other drivers can use the module
 */
static __inline uint32_t twi_close(void)
{
    nrf_drv_twi_disable(m_drv_pressure.p_twi_instance);

    nrf_drv_twi_uninit(m_drv_pressure.p_twi_instance);

    return NRF_SUCCESS;
}

/**
 * @brief Function for reading a sensor register
 *
 * @param[in]  reg_addr            address of the register to read
 * @param[out] p_reg_val           pointer to a buffer to receive the read value
 *
 * @retval NRF_SUCCESS             If operation was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy
 */
static uint32_t reg_read(uint8_t reg_addr, uint8_t * p_reg_val)
{
    uint32_t err_code;

    err_code = twi_open();
    RETURN_IF_ERROR(err_code);

    err_code = nrf_drv_twi_tx( m_drv_pressure.p_twi_instance,
                               MPL3115A2_ADDR,
                               &reg_addr,
                               1,
                               true );
    RETURN_IF_ERROR(err_code);

    err_code = nrf_drv_twi_rx( m_drv_pressure.p_twi_instance,
                               MPL3115A2_ADDR,
                               p_reg_val,
                               1 );
    RETURN_IF_ERROR(err_code);

    err_code = twi_close();
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

/**
 * @brief Function for writing to a sensor register
 *
 * @param[in]  reg_addr            address of the register to write to
 * @param[in]  reg_val             value to write to the register
 *
 * @retval NRF_SUCCESS             If operation was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy
 */
static uint32_t reg_write(uint8_t reg_addr, uint8_t reg_val)
{
    uint32_t err_code;

    uint8_t buffer[2] = {reg_addr,reg_val};

    err_code = twi_open();
    RETURN_IF_ERROR(err_code);

    err_code = nrf_drv_twi_tx( m_drv_pressure.p_twi_instance,
                               MPL3115A2_ADDR,
                               buffer,
                               2,
                               false );
    RETURN_IF_ERROR(err_code);

    err_code = twi_close();
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

/**
 * @brief Function for verifying the address of the sensor (slave)
 *
 * @retval NRF_SUCCESS             If sensor WHO_AM_I register value matches expected
 * @retval NRF_ERROR_NOT_FOUND     If the sensor did not respond or did not match expected
 */
uint32_t drv_pressure_verify_chip(void)
{
    uint32_t err_code;
    uint8_t  who_am_i;

    DEBUG_PRINTF(0, "drv_pressure_verify_chip: ");

    err_code = reg_read(MPL3115A2_WHO_AM_I, &who_am_i);
    if (err_code != NRF_SUCCESS)
    {
        DEBUG_PRINTF(0, "twi failed\r\n");
    }

    DEBUG_PRINTF(0, "who_am_i = 0x%x\r\n", who_am_i);

    return who_am_i == MPL3115A2_ID ? NRF_SUCCESS : NRF_ERROR_NOT_FOUND;
}

/**
 * @brief Function for resetting the chip to all default register values
 *
 * @retval NRF_SUCCESS             If operation was successful
 * @retval NRF_ERROR_BUSY          If TWI bus was busy
 */
uint32_t drv_pressure_soft_reset(void)
{
    uint32_t err_code;
    uint8_t ctrl_reg1;

    err_code = reg_read(MPL3115A2_CTRL_REG1, &ctrl_reg1);
    RETURN_IF_ERROR(err_code);

    ctrl_reg1 |= MPL3115A2_CTRL_REG1_RST;

    return reg_write(MPL3115A2_CTRL_REG1, ctrl_reg1);
}


/**
 * @brief Function for putting the sensor into altimeter or barometer mode and into
 *        standby mode
 *
 * @param[in] mode                 Altimeter or Barometer mode
 *
 * @retval NRF_SUCCESS             If operation was successful
 * @retval NRF_ERROR_BUSY          If TWI bus was busy
 */
uint32_t drv_pressure_standby_mode_set(drv_pressure_mode_t mode)
{
    uint32_t err_code;
    uint8_t ctrl_reg1;

    err_code = reg_read(MPL3115A2_CTRL_REG1, &ctrl_reg1);
    RETURN_IF_ERROR(err_code);

    ctrl_reg1 &= ~MPL3115A2_CTRL_REG1_SBYB_MASK;

    if ( mode == DRV_PRESSURE_MODE_ALTIMETER )
    {
        ctrl_reg1 |= MPL3115A2_CTRL_REG1_ALT_MASK;
    }
    else
    {
        ctrl_reg1 &= ~MPL3115A2_CTRL_REG1_ALT_MASK;
    }

    return reg_write(MPL3115A2_CTRL_REG1, ctrl_reg1);
}

/**
 * @brief Function for putting the sensor into altimeter or barometer mode and into
 *        active mode
 *
 * @param[in] mode                 Altimeter or Barometer mode
 *
 * @retval NRF_SUCCESS             If operation was successful
 * @retval NRF_ERROR_BUSY          If TWI bus was busy
 */
uint32_t drv_pressure_active_mode_set(drv_pressure_mode_t mode)
{
    uint32_t err_code;
    uint8_t ctrl_reg1;

    err_code = reg_read(MPL3115A2_CTRL_REG1, &ctrl_reg1);
    RETURN_IF_ERROR(err_code);

    /* Set active mode */
    if ( mode == DRV_PRESSURE_MODE_ALTIMETER)
    {
        /* Set ALT bit and SBYB bit */
        ctrl_reg1 |= MPL3115A2_CTRL_REG1_SBYB_MASK | MPL3115A2_CTRL_REG1_ALT_MASK;
    }
    else
    {
        /* Clear ALT bit and set SBYB bit */
        ctrl_reg1 &= ~MPL3115A2_CTRL_REG1_ALT_MASK;
        ctrl_reg1 |= MPL3115A2_CTRL_REG1_SBYB_MASK;
    }

    return reg_write(MPL3115A2_CTRL_REG1, ctrl_reg1);
}

/**
 * @brief Function for configuring over sampling rate (internal averaging and noise reduction)
 *
 * @param[in] osr                  2^(osr) of oversampling
 *
 * @retval NRF_SUCCESS             If configuration was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy
 */

uint32_t drv_pressure_oversampling_rate_set(uint8_t osr)
{
    uint32_t err_code;
    uint8_t ctrl_reg1;

    if (osr > 7)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    err_code = reg_read(MPL3115A2_CTRL_REG1, &ctrl_reg1);
    RETURN_IF_ERROR(err_code);

    ctrl_reg1 &= ~MPL3115A2_CTRL_REG1_OS_MASK;
    ctrl_reg1 |= (osr << MPL3115A2_CTRL_REG1_OS_POS);

    return reg_write(MPL3115A2_CTRL_REG1, ctrl_reg1);
}

/**
 * @brief Function for configuring the period of sensor's auto-sampling
 *
 * @param[in] time_step            2^(time_step) seconds is set as period
 *
 * @retval NRF_SUCCESS             If configuration was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy
 */

uint32_t drv_pressure_time_step_set(uint8_t time_step)
{
    uint32_t err_code;
    uint8_t ctrl_reg2;

    if (time_step > 0xF)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    err_code = reg_read(MPL3115A2_CTRL_REG2, &ctrl_reg2);
    RETURN_IF_ERROR(err_code);

    ctrl_reg2 &= ~MPL3115A2_CTRL_REG2_ST_MASK;
    ctrl_reg2 |= (time_step << MPL3115A2_CTRL_REG2_ST_POS);

    return reg_write(MPL3115A2_CTRL_REG2, ctrl_reg2);
}

/**
 * @brief Function for immediately taking one sensor measurement
 *
 * @retval NRF_SUCCESS             If measurement was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy
 */

uint32_t drv_pressure_one_shot_toggle(void)
{
    uint32_t err_code;
    uint8_t ctrl_reg1;

    err_code = reg_read(MPL3115A2_CTRL_REG1, &ctrl_reg1);
    RETURN_IF_ERROR(err_code);

    ctrl_reg1 &= ~MPL3115A2_CTRL_REG1_OST;

    err_code = reg_write(MPL3115A2_CTRL_REG1, ctrl_reg1);
    RETURN_IF_ERROR(err_code);

    err_code = reg_read(MPL3115A2_CTRL_REG1, &ctrl_reg1);
    RETURN_IF_ERROR(err_code);

    ctrl_reg1 |= MPL3115A2_CTRL_REG1_OST;

    return reg_write(MPL3115A2_CTRL_REG1, ctrl_reg1);
}

/**
 * @brief Function for enabling data flags when data is ready
 *
 * @retval NRF_SUCCESS             If configuration was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy
 */

uint32_t drv_pressure_data_flag_enable(void)
{
    uint8_t pt_data_cfg;

    pt_data_cfg = MPL3115A2_PT_DATA_CFG_PDEFE | MPL3115A2_PT_DATA_CFG_TDEFE | MPL3115A2_PT_DATA_CFG_DREM;

    return reg_write(MPL3115A2_PT_DATA_CFG, pt_data_cfg);
}

/**
 * @brief Function for reading the data registers of the sensor
 *
 * @param[out] p_data       Pointer to the data buffer for receiving sensor data
 */

uint32_t mpl3115a2_data_read(drv_mp3115a2_data_t * p_data)
{
    uint32_t err_code;
    drv_pressure_evt_t press_event;
    drv_pressure_mode_t mode = m_drv_pressure.mode;

    // DEBUG_PRINTF(0, "mpl3115a2_data_read:\r\n");

    err_code = reg_read(MPL3115A2_OUT_P_MSB, &p_data->pressure.msb);
    RETURN_IF_ERROR(err_code);
    err_code = reg_read(MPL3115A2_OUT_P_CSB, &p_data->pressure.csb);
    RETURN_IF_ERROR(err_code);
    err_code = reg_read(MPL3115A2_OUT_P_LSB, &p_data->pressure.lsb);
    RETURN_IF_ERROR(err_code);
    err_code = reg_read(MPL3115A2_OUT_T_MSB, &p_data->temp.msb);
    RETURN_IF_ERROR(err_code);
    err_code = reg_read(MPL3115A2_OUT_T_LSB, &p_data->temp.lsb);
    RETURN_IF_ERROR(err_code);
    err_code = reg_read(MPL3115A2_STATUS, &p_data->status);
    RETURN_IF_ERROR(err_code);

    press_event = DRV_PRESSURE_EVT_DATA_RDY;
    m_drv_pressure.pressure_evt_handler(&press_event, &mode);

    return NRF_SUCCESS;
}

/**
 * @brief HW Interrupt handler
 *
 * @param[in] pin                 The GPIO on which the interrupt occured
 * @param[in] action              The logic transtion of the interrupt
 */
static void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint32_t err_code;

    if ((pin == MPL_INT2) && (action == NRF_GPIOTE_POLARITY_HITOLO))
    {
        uint8_t int_source;
        err_code = reg_read(0x12, &int_source);
        APP_ERROR_CHECK(err_code);

        if (int_source & 0x80)
        {
            err_code = mpl3115a2_data_read(&m_data);
            APP_ERROR_CHECK(err_code);
        }
    }
}

/**
 * @brief Function for configuring the sensor's interrupt
 *
 * @retval NRF_SUCCESS             If configuration was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy
 */
static uint32_t mpl3115a2_int_configure(void)
{
    uint32_t            err_code;
    /* Set standby mode */
    err_code = drv_pressure_standby_mode_set(m_drv_pressure.mode);
    RETURN_IF_ERROR(err_code);

    /* Max over sampliong rate for minimum noise*/
    err_code = drv_pressure_oversampling_rate_set(0);
    RETURN_IF_ERROR(err_code);

    err_code = drv_pressure_data_flag_enable();
    RETURN_IF_ERROR(err_code);

    /* Set auto-sampling to max period (2^15 seconds) */
    err_code = drv_pressure_time_step_set(15);
    RETURN_IF_ERROR(err_code);

    /* Configure the INT pins for Open Drain and Active Low. */
    err_code = reg_write(MPL3115A2_CTRL_REG3, (MPL3115A2_CTRL_REG3_PP_OD2 | MPL3115A2_CTRL_REG3_PP_OD1));
    RETURN_IF_ERROR(err_code);

    /* Enable the Data Ready Interrupt and route to INT 2 */
    return reg_write(MPL3115A2_CTRL_REG4, MPL3115A2_CTRL_REG4_INT_EN_DRDY);
}

float drv_pressure_altitude_get(void)
{
  // The least significant bytes l_altitude and l_temp are 4-bit,
  // fractional values, so you must cast the calulation in (float),
  // shift the value over 4 spots to the right and divide by 16 (since
  // there are 16 values in 4-bits).

  // float tempcsb = (m_data.pressure.lsb >> 4) / 16.0f;
  // float altitude = (float)( (m_data.pressure.msb << 8) | m_data.pressure.csb) + tempcsb;

  float altitude = (float)
                  ((((uint32_t)(m_data.pressure.msb) << 24)
                  |
                  ((uint32_t)(m_data.pressure.csb) << 16)
                  |
                  ((uint32_t)(m_data.pressure.lsb) << 8)))/65536;

  return altitude;
}

float drv_pressure_get(void)
{
    // Pressure comes back as a left shifted 20 bit number
    uint32_t pressure_whole = ((uint32_t)m_data.pressure.msb << 16 | (uint32_t)m_data.pressure.csb << 8 | (uint32_t)m_data.pressure.lsb);
    pressure_whole >>= 6; //Pressure is an 18 bit number with 2 bits of decimal. Get rid of decimal portion.

    m_data.pressure.lsb &= 0x30;                                //Bits 5/4 represent the fractional component
    m_data.pressure.lsb >>= 4;                                 //Get it right aligned
    float pressure_decimal = (float)m_data.pressure.lsb / 4.0f; //Turn it into fraction

    float pressure = (float)pressure_whole + pressure_decimal;

    return(pressure);
}

uint32_t drv_pressure_sample(void)
{
    uint32_t err_code;

    err_code = drv_pressure_one_shot_toggle();
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_pressure_mode_set(drv_pressure_mode_t mode)
{
    uint32_t err_code;

    m_drv_pressure.mode = mode;
    err_code = drv_pressure_standby_mode_set(m_drv_pressure.mode);
    RETURN_IF_ERROR(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_pressure_enable(void)
{
    uint32_t err_code;

    nrf_drv_gpiote_in_config_t gpiote_in_config;
    gpiote_in_config.is_watcher  = false;
    gpiote_in_config.hi_accuracy = false;
    gpiote_in_config.pull        = NRF_GPIO_PIN_PULLUP;
    gpiote_in_config.sense       = NRF_GPIOTE_POLARITY_HITOLO;
    err_code = nrf_drv_gpiote_in_init(MPL_INT2, &gpiote_in_config, gpiote_evt_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(MPL_INT2, true);

    /* Configure the sensors to set up interupts for when data is ready */
    err_code = mpl3115a2_int_configure();
    APP_ERROR_CHECK(err_code);

    return NRF_SUCCESS;
}

uint32_t drv_pressure_disable(void)
{

    DEBUG_PRINTF(0, "drv_pressure_disable:\r\n");

    nrf_drv_gpiote_in_event_disable(MPL_INT2);

    nrf_drv_gpiote_in_uninit(MPL_INT2);

    return NRF_SUCCESS;
}

uint32_t drv_pressure_init(drv_pressure_init_t * p_params)
{
    uint32_t err_code;

    DEBUG_PRINTF(0, "drv_pressure_init:\r\n");

    m_drv_pressure.p_twi_instance       = p_params->p_twi_instance;
    m_drv_pressure.pressure_evt_handler = p_params->pressure_evt_handler;
    m_drv_pressure.mode                 = p_params->mode;

    /* Verify connection with the sensor */
    err_code = drv_pressure_verify_chip();
    RETURN_IF_ERROR(err_code);

    /* Configure gpiote for the sensors data ready interrupt */
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        RETURN_IF_ERROR(err_code);
    }
    return NRF_SUCCESS;
}
