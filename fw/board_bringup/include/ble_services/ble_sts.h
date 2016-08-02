/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
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

/**@file
 *
 * @defgroup ble_sdk_srv_sts Strato Telemetry Service
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Strato Telemetry Service implementation.
 *
 * @details The Strato Telemetry Service is a simple GATT-based service with multiple characteristics for reading sensor data and configuring the sensors.
 *
 * @note The application must propagate S132 SoftDevice events to the Strato Telemetry Service module
 *       by calling the ble_sts_on_ble_evt() function from the ble_stack_handler callback.
 */

#ifndef BLE_STS_H__
#define BLE_STS_H__

#include "ble.h"
#include "ble_srv_common.h"
#include "app_util_platform.h"
#include <stdint.h>
#include <stdbool.h>

#define BLE_UUID_STS_SERVICE 0x0001                      /**< The UUID of the Strato Telemetry Service. */
#define BLE_STS_MAX_DATA_LEN (GATT_MTU_SIZE_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Strato Telemetry service module. */

#ifdef __GNUC__
    #ifdef PACKED
        #undef PACKED
    #endif

    #define PACKED(TYPE) TYPE __attribute__ ((packed))
#endif

typedef PACKED( struct
{
    int8_t  integer;
    uint8_t decimal;
}) ble_sts_temperature_t;

typedef PACKED( struct
{
    int32_t  integer;
    uint8_t  decimal;
}) ble_sts_altitude_t;

typedef PACKED( struct
{
    int16_t  x;
    int16_t  y;
    int16_t  z;
}) ble_sts_accel_t;

// typedef uint8_t ble_sts_humidity_t;

typedef enum
{
    STS_PRESSURE_MODE_BAROMETER,
    STS_PRESSURE_MODE_ALTIMETER
} ble_sts_pressure_mode_t;

typedef PACKED( struct
{
    uint16_t                temperature_interval_ms;
    uint16_t                altitude_interval_ms;
    uint16_t                accel_interval_ms;
    ble_sts_pressure_mode_t pressure_mode;
}) ble_sts_config_t;

typedef enum
{
    BLE_STS_EVT_NOTIF_TEMPERATURE,
    BLE_STS_EVT_NOTIF_ALTITUDE,
    BLE_STS_EVT_NOTIF_ACCEL,
    BLE_STS_EVT_CONFIG_RECEIVED
}ble_sts_evt_type_t;

/* Forward declaration of the ble_sts_t type. */
typedef struct ble_sts_s ble_sts_t;

/**@brief Strato Telemetry Service event handler type. */
typedef void (*ble_sts_evt_handler_t) (ble_sts_t        * p_sts,
                                       ble_sts_evt_type_t evt_type,
                                       uint8_t          * p_data,
                                       uint16_t           length);

/**@brief Strato Telemetry Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_sts_init
 *          function.
 */
typedef struct
{
    ble_sts_temperature_t * p_init_temperature;
    ble_sts_altitude_t    * p_init_altitude;
    ble_sts_accel_t       * p_init_accel;
    ble_sts_config_t      * p_init_config;
    ble_sts_evt_handler_t   evt_handler; /**< Event handler to be called for handling received data. */
} ble_sts_init_t;

/**@brief Strato Telemetry Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_sts_s
{
    uint8_t                  uuid_type;                    /**< UUID type for Strato Telemetry Service Base UUID. */
    uint16_t                 service_handle;               /**< Handle of Strato Telemetry Service (as provided by the S110 SoftDevice). */
    ble_gatts_char_handles_t temperature_handles;          /**< Handles related to the temperature characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t altitude_handles;             /**< Handles related to the altitude characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t accel_handles;                /**< Handles related to the accel characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t config_handles;               /**< Handles related to the config characteristic (as provided by the S132 SoftDevice). */
    bool                     is_temperature_notif_enabled; /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool                     is_altitude_notif_enabled;    /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool                     is_accel_notif_enabled;    /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    uint16_t                 conn_handle;                  /**< Handle of the current connection (as provided by the S110 SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_sts_evt_handler_t    evt_handler;                  /**< Event handler to be called for handling received data. */
};

/**@brief Function for initializing the Strato Telemetry Service.
 *
 * @param[out] p_sts      Strato Telemetry Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_sts_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_sts or p_sts_init is NULL.
 */
uint32_t ble_sts_init(ble_sts_t * p_sts, const ble_sts_init_t * p_sts_init);

/**@brief Function for handling the Strato Telemetry Service's BLE events.
 *
 * @details The Strato Telemetry Service expects the application to call this function each time an
 * event is received from the S110 SoftDevice. This function processes the event if it
 * is relevant and calls the Strato Telemetry Service event handler of the
 * application if necessary.
 *
 * @param[in] p_sts       Strato Telemetry Service structure.
 * @param[in] p_ble_evt   Event received from the S110 SoftDevice.
 */
void ble_sts_on_ble_evt(ble_sts_t * p_sts, ble_evt_t * p_ble_evt);

/**@brief Function for setting the temperature.
 *
 * @details This function sends the input temperature as an temperature characteristic notification to the
 *          peer.
 *
 * @param[in] p_sts       Pointer to the Strato Telemetry Service structure.
 * @param[in] p_data      Pointer to the temperature data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_sts_temperature_set(ble_sts_t * p_sts, ble_sts_temperature_t * p_data);

/**@brief Function for setting the altitude.
 *
 * @details This function sends the input altitude as an altitude characteristic notification to the
 *          peer.
 *
 * @param[in] p_sts       Pointer to the Strato Telemetry Service structure.
 * @param[in] p_data      Pointer to the altitude data.
 * @param[in] length      Length of the string.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_sts_altitude_set(ble_sts_t * p_sts, ble_sts_altitude_t * p_data);

/**@brief Function for setting the acceleration.
 *
 * @details This function sends the input acceleration as an acceleration characteristic notification to the
 *          peer.
 *
 * @param[in] p_sts       Pointer to the Strato Telemetry Service structure.
 * @param[in] p_data      Pointer to the accel data.
 * @param[in] length      Length of the string.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_sts_accel_set(ble_sts_t * p_sts, ble_sts_accel_t * p_data);

#endif // BLE_STS_H__

/** @} */
