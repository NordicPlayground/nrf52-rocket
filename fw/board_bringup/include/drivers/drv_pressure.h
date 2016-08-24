#ifndef __DRV_PRESSURE_H__
#define __DRV_PRESSURE_H__

#include "nrf_drv_twi.h"
#include "drv_pressure_mpl3115A2.h"

/**
 * @brief Pressure driver event types.
 */
typedef enum
{
    DRV_PRESSURE_EVT_DATA_RDY,    /**<Converted value ready to be read*/
    DRV_PRESSURE_EVT_BUS_ERROR    /**<HW error on the communication bus*/
}drv_pressure_evt_t;

/**
 * @brief Pressure driver event handler callback type.
 */
typedef void (*drv_pressure_evt_handler_t)(drv_pressure_evt_t const * p_evt,
                                           void *                     p_context);

typedef enum
{
    DRV_PRESSURE_MODE_ALTIMETER,
    DRV_PRESSURE_MODE_BAROMETER
}drv_pressure_mode_t;

/**
 * @brief Initialization struct for pressure driver
 */
typedef struct
{
    const nrf_drv_twi_t *       p_twi_instance;
    drv_pressure_evt_handler_t  pressure_evt_handler;
    drv_pressure_mode_t         mode;
}drv_pressure_init_t;

/**
 * @brief Function for initializing the pressure driver.
 *
 * @param[in] p_params      Pointer to init parameters.
 *
 * @retval NRF_SUCCESS             If initialization was successful.
 * @retval NRF_ERROR_INVALID_STATE If the driver is in invalid state.
 */
uint32_t drv_pressure_init(drv_pressure_init_t * p_params);

uint32_t drv_pressure_enable(void);

uint32_t drv_pressure_disable(void);
/**
* @brief Function for putting the sensor into altimeter or barometer mode and into
*        active mode
*
* @param[in] mode                 Altimeter or Barometer mode
*
* @retval NRF_SUCCESS             If operation was successful
* @retval NRF_ERROR_BUSY          If TWI bus was busy
*/
uint32_t drv_pressure_active_mode_set(drv_pressure_mode_t mode);

/**
 * @brief Function for putting the sensor into altimeter or barometer mode and into
 *        standby mode
 *
 * @param[in] mode                 Altimeter or Barometer mode
 *
 * @retval NRF_SUCCESS             If operation was successful
 * @retval NRF_ERROR_BUSY          If TWI bus was busy
 */
uint32_t drv_pressure_standby_mode_set(drv_pressure_mode_t mode);

/**
* @brief Function for resetting the chip to all default register values
*
* @retval NRF_SUCCESS             If operation was successful
* @retval NRF_ERROR_BUSY          If TWI bus was busy
*/
uint32_t drv_pressure_soft_reset(void);

/**
 * @brief Function for verifying the address of the sensor (slave)
 *
 * @retval NRF_SUCCESS             If sensor WHO_AM_I register value matches expected
 * @retval NRF_ERROR_NOT_FOUND     If the sensor did not respond or did not match expected
 */
uint32_t drv_pressure_verify_chip(void);

/**
 * @brief Function for changing the mode of the pressure sensor
 *
 * @param[in] mode                 Altimeter or Barometer
 *
 * @retval NRF_SUCCESS             If configuration was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy
 */
uint32_t drv_pressure_mode_set(drv_pressure_mode_t mode);

/**
 * @brief Function for configuring the period of sensor's auto-sampling
 *
 * @param[in] time_step            2^(time_step) seconds is set as period
 *
 * @retval NRF_SUCCESS             If configuration was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy
 */

uint32_t drv_pressure_time_step_set(uint8_t time_step);

/**
 * @brief Function for configuring over sampling rate (internal averaging and noise reduction)
 *
 * @param[in] osr                  2^(osr) of oversampling
 *
 * @retval NRF_SUCCESS             If configuration was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy
 */

uint32_t drv_pressure_oversampling_rate_set(uint8_t osr);

/**
 * @brief Function for immediately taking one sensor measurement
 *
 * @retval NRF_SUCCESS             If measurement was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy
 */

uint32_t drv_pressure_one_shot_toggle(void);

/**
 * @brief Function for enabling data flags when data is ready
 *
 * @retval NRF_SUCCESS             If configuration was successful.
 * @retval NRF_ERROR_BUSY          If the TWI drivers are busy
 */

uint32_t drv_pressure_data_flag_enable(void);

/**
 * @brief Function for reading the data registers of the sensor
 *
 * @param[out] p_data       Pointer to the data buffer for receiving sensor data
 */

uint32_t mpl3115a2_data_read(drv_mp3115a2_data_t * p_data);

 /**
  * @brief Function for getting the altitude data.
  *
  * @retval Altitude data.
  */
float drv_pressure_altitude_get(void);

/**
 * @brief Function for getting the pressure data.
 *
 * @retval Pressure data.
 */
float drv_pressure_get(void);

/**
 * @brief Function to start sampling.
 *
 * @retval NRF_SUCCESS             If start sampling was successful.
 */
uint32_t drv_pressure_sample(void);

/**
 * @brief Function to put sensor in sleep.
 *
 * @retval NRF_SUCCESS             If sleep was successful.
 */
uint32_t drv_pressure_sleep(void);

#endif
