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
#ifndef __DRV_HTU21D_H__
#define __DRV_HTU21D_H__

#include "nrf_drv_twi.h"

/*Number of bytes to read in the sensor raw data. When using CRC, set to 3*/
#define DATA_BYTES_CNT          2
/**
 * @brief Humidity driver resolution types.
 * The two numbers correspond to Humidity
 * and temperature, respectively.
 */
typedef enum
{
    DRV_HTU21D_ADC_RES_12_14B,
    DRV_HTU21D_ADC_RES_8_12B,
    DRV_HTU21D_ADC_RES_10_13B,
    DRV_HTU21D_ADC_RES_11_11B
} drv_htu21d_adc_res_t;

/**
 * @brief Configuration struct for humidity driver
 */
typedef struct
{
    drv_htu21d_adc_res_t       adc_res;
    bool                       heater_enable;   //Only for diagnostic purposes, keep off by default
}drv_htu21d_config_t;

/**
 * @brief Initialization struct for humid driver
 */
typedef struct
{
    nrf_drv_twi_t const *        p_twi_instance;
    drv_htu21d_config_t const *  p_config;
}drv_htu21d_init_t;

/**
 * @brief Function for initializing the humidity driver.
 *
 * @param[in] p_params      Pointer to init parameters.
 *
 * @retval NRF_SUCCESS             If initialization was successful.
 * @retval NRF_ERROR_INVALID_STATE If the driver is in invalid state.
 */
ret_code_t drv_htu21d_init(drv_htu21d_init_t * p_params);

/**
 * @brief Function for configuring the user register of the humidity sensor
 * @note since the sensor's user register has reserved fields that change without notice,
 *       this function reads the register first before writing back to preserve the reserved
 *       fields, it is the recommended interface for writing to the user register
 *
 * @param[in] p_params             Pointer to config parameters.
 *
 * @retval NRF_SUCCESS             If configuration was successful.
 * @retval NRF_ERROR_INVALID_STATE If the sensor could not be configured.
 */
ret_code_t drv_htu21d_config(drv_htu21d_config_t * p_params);

/**
 * @brief Function for reading the user register of the humidity sensor.
 *
 * @param[in] p_user_reg      Pointer to the read buffer
 *
 * @retval NRF_SUCCESS             If configuration was successful.
 * @retval NRF_ERROR_INVALID_STATE If the sensor could not be configured.
 */
ret_code_t drv_htu21d_user_reg_write(uint8_t * p_user_reg);

/**
 * @brief Function for writing the user register of the humidity sensor.
 *
 * @param[in] p_params      Pointer to the write buffer
 *
 * @retval NRF_SUCCESS             If configuration was successful.
 * @retval NRF_ERROR_INVALID_STATE If the sensor could not be configured.
 */
ret_code_t drv_htu21d_user_reg_read(uint8_t * p_user_reg);

/**
 * @brief Function for soft reseting the sensor to default configs.
 *
 * @retval NRF_SUCCESS             If soft reset was successful.
 * @retval NRF_ERROR_INVALID_STATE If the sensor could not be reset.
 */
ret_code_t drv_htu21d_drv_pressure_soft_reset(void);

/**
 * @brief Function for starting the ADC conversion of humidity

 * @retval NRF_SUCCESS        If ADC conversion was successful.
 * @retval NRF_ERROR_BUSY     If the driver is not ready for a new transfer.
 * @retval NRF_ERROR_INTERNAL If an error was detected by hardware.
 *
 * Note: When an error is returned, a DRV_HTU21D_EVT_BUS_ERROR will also trigger
 * in a defined drv_htu21d_evt_handler_t
 */
ret_code_t drv_htu21d_humid_conv_begin(void);

/**
 * @brief Function for starting the ADC conversion of temperature

 * @retval NRF_SUCCESS        If ADC conversion was successful.
 * @retval NRF_ERROR_BUSY     If the driver is not ready for a new transfer.
 * @retval NRF_ERROR_INTERNAL If an error was detected by hardware.
 *
 */
ret_code_t drv_htu21d_temp_conv_begin(void);
/**
 * @brief Function for reading the sensor's raw humidity data .
 * @param[out]   p_temp_buffer  buffer of size DATA_BYTES_CNT where the data will be read to
 * @retval NRF_SUCCESS        If the read process was successful.
 * @retval NRF_ERROR_BUSY     If the driver is not ready for a new transfer.
 * @retval NRF_ERROR_INTERNAL If an error was detected by hardware.
 */
ret_code_t drv_htu21d_humid_raw_read( uint8_t * p_temp_buffer );
/**
 * @brief Function for reading the sensor's raw temperatures data.
 * @param[out]   p_humid_buffer  buffer of size DATA_BYTES_CNT where the data will be read to
 * @retval NRF_SUCCESS        If the read process was successful.
 * @retval NRF_ERROR_BUSY     If the driver is not ready for a new transfer.
 * @retval NRF_ERROR_INTERNAL If an error was detected by hardware.
 */
ret_code_t drv_htu21d_temp_raw_read( uint8_t * p_humid_buffer );
/**
 * @brief Function for reading the sensor's humidity in readable format.
 *
 * @param[out] p_humidity     pointer to humidity byte
 *
 * @retval NRF_SUCCESS        If the read process was successful.
 * @retval NRF_ERROR_BUSY     If the driver is not ready for a new transfer.
 * @retval NRF_ERROR_INTERNAL If an error was detected by hardware.
 */
ret_code_t drv_htu21d_humidity_get( uint8_t * p_humidity);
/**
 * @brief Function for reading the sensor's temp in readable format.
 *
 * @param[out]                pointer to temperature float
 *
 * @retval NRF_SUCCESS        If the read process was successful.
 * @retval NRF_ERROR_BUSY     If the driver is not ready for a new transfer.
 * @retval NRF_ERROR_INTERNAL If an error was detected by hardware.
 */
ret_code_t drv_htu21d_temperature_get( float * p_temperature);

#endif /*__DRV_HTU21D_H__*/
