/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */
#include "drv_htu21d.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "pca20027.h"
#include "nordic_common.h"
#include <string.h>

//Device Address
#define HUMID_SENS_ADDR     0x40

//Commands
#define HUMID_CONV_CMD      0xF5    /**no hold master mode*/
#define TEMP_CONV_CMD       0xF3    /**no hold master mode*/
#define USER_REG_READ_CMD   0xE7
#define USER_REG_WRITE_CMD  0xE6
#define drv_pressure_soft_reset_CMD      0xFE


//#define HUMID_DEBUG

#ifdef HUMID_DEBUG
    #include "SEGGER_RTT.h"
    #define HUMID_DEBUG_PRINTF SEGGER_RTT_printf
#else
    #define HUMID_DEBUG_PRINTF(...)
#endif

#define RETURN_IF_ERROR(PARAM)                                                                    \
        if ((PARAM) != NRF_SUCCESS)                                                               \
        {                                                                                         \
            return (PARAM);                                                                       \
        }

//Formula found on HTU21D Datasheet
#define DATA_MASK(x)            (x & 0xFFFC)     /*the status bits are set to 0*/
#define CONVERT_TO_RH(x)        (-6+(int8_t)(125*DATA_MASK(x)/65536))
#define CONVERT_TO_TEMP(x)      (-46.85+(175.72*DATA_MASK(x)/65536))

/**
 * @brief Humidity driver struct
 */
typedef struct
{
    const nrf_drv_twi_t *         p_twi_instance;
    drv_htu21d_config_t           config;
} drv_htu21d_t;

static drv_htu21d_t            m_drv_htu21d;

const nrf_drv_twi_config_t twi_htu21d_config =
{
    .scl                = TWI_SCL_PIN_NUMBER,
    .sda                = TWI_SDA_PIN_NUMBER,
    .frequency          = NRF_TWI_FREQ_400K,
    .interrupt_priority = APP_IRQ_PRIORITY_LOW
};

/**
 * @brief Function to deinit the TWI module when this driver does not need to
 *        communicate on the TWI bus, so that other drivers can use the module
 */
static void __inline twi_close(void)
{
    nrf_drv_twi_disable(m_drv_htu21d.p_twi_instance);
    nrf_drv_twi_uninit(m_drv_htu21d.p_twi_instance);
}

/**
 * @brief Function to occupy the TWI module when this driver needs to
 *        communicate on the TWI bus, so that other drivers cannot use the module
 */
static ret_code_t __inline twi_open(void)
{
    ret_code_t err_code;
    err_code = nrf_drv_twi_init(m_drv_htu21d.p_twi_instance,
                                &twi_htu21d_config,
                                NULL,
                                NULL);
    RETURN_IF_ERROR(err_code);
    nrf_drv_twi_enable(m_drv_htu21d.p_twi_instance);
    return NRF_SUCCESS;
}

ret_code_t drv_htu21d_temp_raw_read( uint8_t * p_temp_buffer )
{
    ret_code_t err_code;

    err_code = twi_open();
    RETURN_IF_ERROR(err_code);

    err_code = nrf_drv_twi_rx(m_drv_htu21d.p_twi_instance,
                              HUMID_SENS_ADDR,
                              p_temp_buffer,
                              DATA_BYTES_CNT);
    twi_close();
    return err_code;
}

ret_code_t drv_htu21d_humid_raw_read( uint8_t * p_humid_buffer )
{
    ret_code_t err_code;

    err_code = twi_open();
    RETURN_IF_ERROR(err_code);
    err_code = nrf_drv_twi_rx(m_drv_htu21d.p_twi_instance,
                              HUMID_SENS_ADDR,
                              p_humid_buffer,
                              DATA_BYTES_CNT);

    twi_close();
    return err_code;
}

ret_code_t drv_htu21d_user_reg_write( uint8_t * p_user_reg )
{
    ret_code_t err_code;
    uint8_t user_reg_cmd = USER_REG_WRITE_CMD;

    err_code = twi_open();
    RETURN_IF_ERROR(err_code);
    nrf_drv_twi_xfer_desc_t xfer_desc = NRF_DRV_TWI_XFER_DESC_TXTX(
                                        HUMID_SENS_ADDR,
                                        &user_reg_cmd,
                                        1,
                                        p_user_reg,
                                        1);
    err_code = nrf_drv_twi_xfer(m_drv_htu21d.p_twi_instance,
                                &xfer_desc,0);
    twi_close();
    return err_code;
}

ret_code_t drv_htu21d_user_reg_read( uint8_t * p_user_reg )
{
    ret_code_t err_code;
    uint8_t user_reg_cmd = USER_REG_READ_CMD;

    err_code = twi_open();
    RETURN_IF_ERROR(err_code);
    nrf_drv_twi_xfer_desc_t xfer_desc = NRF_DRV_TWI_XFER_DESC_TXRX(
                                        HUMID_SENS_ADDR,
                                        &user_reg_cmd,
                                        1,
                                        p_user_reg,
                                        1);
    err_code = nrf_drv_twi_xfer(m_drv_htu21d.p_twi_instance,
                                &xfer_desc,0);
    twi_close();
    return err_code;
}

ret_code_t drv_htu21d_drv_pressure_soft_reset(void)
{
    ret_code_t err_code;
    uint8_t reset_cmd = drv_pressure_soft_reset_CMD;

    err_code = twi_open();
    RETURN_IF_ERROR(err_code);

    err_code = nrf_drv_twi_tx(m_drv_htu21d.p_twi_instance,
                              HUMID_SENS_ADDR,
                              &reset_cmd,
                              1,
                              true);
    twi_close();

    return err_code;
}

ret_code_t drv_htu21d_init(drv_htu21d_init_t * p_params)
{

   m_drv_htu21d.p_twi_instance       = p_params->p_twi_instance;
   m_drv_htu21d.config.adc_res       = p_params->p_config->adc_res;
   m_drv_htu21d.config.heater_enable = p_params->p_config->heater_enable;

   return NRF_SUCCESS;
}

ret_code_t drv_htu21d_config(drv_htu21d_config_t * p_params)
{
    ret_code_t err_code;
    uint8_t user_reg;
    uint8_t config_byte;
    err_code = drv_htu21d_user_reg_read(&user_reg);
    config_byte = user_reg;

    switch(p_params->adc_res)
    {
         case DRV_HTU21D_ADC_RES_12_14B:
             CLR_BIT(config_byte,7);
             CLR_BIT(config_byte,0);
         break;
         case DRV_HTU21D_ADC_RES_8_12B:
             CLR_BIT(config_byte,7);
             SET_BIT(config_byte,0);
         break;
         case DRV_HTU21D_ADC_RES_10_13B:
             SET_BIT(config_byte,7);
             CLR_BIT(config_byte,0);
         break;
         case DRV_HTU21D_ADC_RES_11_11B:
             SET_BIT(config_byte,7);
             SET_BIT(config_byte,0);
         break;
         default:
         break;
    }
    if(p_params->heater_enable)
    {
        SET_BIT(config_byte,2);
    }
    else
    {
        CLR_BIT(config_byte,2);
    }

    /*Use the mask to check if only bits 0,2,7 are changed before and after the modification.
    If allowing more bits to be modified, then adjust the check mask accordingly*/
    const uint8_t user_reg_check_mask = 0x7A;

    if((config_byte & user_reg_check_mask) ==
       (user_reg & user_reg_check_mask))
    {
           err_code = drv_htu21d_user_reg_write(&config_byte);
     }
     else
     {
         return NRF_ERROR_INVALID_PARAM;
     }

    return err_code;
}

ret_code_t drv_htu21d_temp_conv_begin(void)
{
    uint8_t temp_meas_cmd = TEMP_CONV_CMD;
    ret_code_t err_code;

    err_code = twi_open();
    RETURN_IF_ERROR(err_code);

    err_code = nrf_drv_twi_tx(m_drv_htu21d.p_twi_instance,
                              HUMID_SENS_ADDR,
                              &temp_meas_cmd,
                              1,
                              false);
    twi_close();
    return err_code;
}

ret_code_t drv_htu21d_humid_conv_begin(void)
{
    uint8_t humid_meas_cmd = HUMID_CONV_CMD;
    ret_code_t err_code;

    err_code = twi_open();
    RETURN_IF_ERROR(err_code);
    err_code = nrf_drv_twi_tx(m_drv_htu21d.p_twi_instance,
                              HUMID_SENS_ADDR,
                              &humid_meas_cmd,
                              1,
                              false);
    twi_close();
    return err_code;
}

ret_code_t drv_htu21d_humidity_get(uint8_t * p_humidity)
{
    uint8_t humid_buffer[DATA_BYTES_CNT] = {0};

    ret_code_t err_code;
    err_code = drv_htu21d_humid_raw_read(humid_buffer);
    RETURN_IF_ERROR(err_code);

    uint32_t data = (humid_buffer[0] << 8) | (humid_buffer[1]);
    int8_t temporary = CONVERT_TO_RH(data);

    if( temporary >= 0 && temporary <= 100 )
    {
        *p_humidity = (uint8_t)temporary;
    }
    else if (temporary < 0)
    {
        *p_humidity = 0;
    }
    else if (temporary > 100)
    {
        *p_humidity = 100;
    }
    return NRF_SUCCESS;
}

ret_code_t drv_htu21d_temperature_get( float * p_temperature)
{
    uint8_t temp_buffer[DATA_BYTES_CNT] = {0};

    ret_code_t err_code;
    err_code = drv_htu21d_temp_raw_read(temp_buffer);
    RETURN_IF_ERROR(err_code);

    uint32_t data = (temp_buffer[0] << 8) | (temp_buffer[1]);

    *p_temperature = CONVERT_TO_TEMP(data);

    return NRF_SUCCESS;
}
