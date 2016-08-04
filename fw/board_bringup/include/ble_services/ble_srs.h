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
 * @defgroup ble_sdk_srv_srs Strato Rocketry Service
 * @{
 * @ingroup  ble_sdk_srv
 * @brief    Strato Rocketry Service implementation.
 *
 * @details The Strato Rocketry Service is a simple GATT-based service with multiple characteristics for reading sensor data and configuring the sensors.
 *
 * @note The application must propagate S132 SoftDevice events to the Strato Rocketry Service module
 *       by calling the ble_srs_on_ble_evt() function from the ble_stack_handler callback.
 */

#ifndef BLE_SRS_H__
#define BLE_SRS_H__

#include "ble.h"
#include "ble_srv_common.h"
#include "app_util_platform.h"
#include <stdint.h>
#include <stdbool.h>

#define BLE_UUID_SRS_SERVICE 0x0000                      /**< The UUID of the Strato Rocketry Service. */
#define BLE_SRS_MAX_DATA_LEN (GATT_MTU_SIZE_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Strato Rocketry service module. */

#ifdef __GNUC__
    #ifdef PACKED
        #undef PACKED
    #endif

    #define PACKED(TYPE) TYPE __attribute__ ((packed))
#endif

typedef enum
{
    PARA_SERVO_CLOSE,
    PARA_SERVO_OPEN,
} para_servo_ctrl_t;

typedef PACKED( struct
{
    int8_t  position_open;   /* -90 to 90 degrees*/
    int8_t  position_closed;
}) para_servo_config_t;

typedef PACKED( struct
{
    para_servo_config_t * p_config;
    para_servo_ctrl_t  ctrl;
}) ble_srs_parachute_servo_t;

typedef enum
{
    BLE_SRS_CAP_IDLE,
    BLE_SRS_CAP_DRAIN
}ble_srs_cap_ctrl_t;

typedef PACKED( struct
{
    uint8_t  integer;
    uint8_t  decimal;
}) ble_srs_cap_volt_t;

typedef PACKED( struct
{
    ble_srs_cap_ctrl_t ctrl;
    ble_srs_cap_volt_t * p_voltage;
}) ble_srs_cap_t;

typedef enum
{
    BLE_SRS_IGNITION_OFF,
    BLE_SRS_IGNITION_ON,
}ble_srs_ignition_ctrl_t;

typedef enum
{
    BLE_SRS_POWER_OFF,
    BLE_SRS_POWER_ON,
}ble_srs_power_t;

typedef enum
{
    BLE_SRS_EVT_POWER,
    BLE_SRS_EVT_IGNITION,
    BLE_SRS_EVT_CAP_CTRL,
    BLE_SRS_EVT_CAP_VOLT,
    BLE_SRS_EVT_SERVO_CTRL,
    BLE_SRS_EVT_SERVO_CONFIG,
}ble_srs_evt_type_t;

/* Forward declaration of the ble_srs_t type. */
typedef struct ble_srs_s ble_srs_t;

/**@brief Strato Rocketry Service event handler type. */
typedef void (*ble_srs_evt_handler_t) (ble_srs_t        * p_srs,
                                       ble_srs_evt_type_t evt_type,
                                       uint8_t          * p_data,
                                       uint16_t           length);

/**@brief Strato Rocketry Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_srs_init
 *          function.
 */
typedef struct
{
    ble_srs_power_t                 power_init;
    ble_srs_cap_t                 * p_init_cap;
    ble_srs_ignition_ctrl_t         ignition_init;
    ble_srs_parachute_servo_t     * p_init_para_servo;
    ble_srs_evt_handler_t           evt_handler; /**< Event handler to be called for handling received data. */
} ble_srs_init_t;

/**@brief Strato Rocketry Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_srs_s
{
    uint8_t                  uuid_type;                    /**< UUID type for Strato Rocketry Service Base UUID. */
    uint16_t                 service_handle;               /**< Handle of Strato Rocketry Service (as provided by the S110 SoftDevice). */
    ble_gatts_char_handles_t power_handles;
    ble_gatts_char_handles_t ignition_handles;             /**< Handles related to the temperature characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t cap_volt_handles;            /**< Handles related to the altitude characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t cap_ctrl_handles;            /**< Handles related to the altitude characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t para_servo_ctrl_handles;     /**< Handles related to the config characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t para_servo_config_handles;     /**< Handles related to the config characteristic (as provided by the S132 SoftDevice). */
    uint16_t                 conn_handle;                  /**< Handle of the current connection (as provided by the S110 SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_srs_evt_handler_t    evt_handler;                  /**< Event handler to be called for handling received data. */
};

/**@brief Function for initializing the Strato Rocketry Service.
 *
 * @param[out] p_srs      Strato Rocketry Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_srs_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_srs or p_srs_init is NULL.
 */
uint32_t ble_srs_init(ble_srs_t * p_srs, const ble_srs_init_t * p_srs_init);

/**@brief Function for handling the Strato Rocketry Service's BLE events.
 *
 * @details The Strato Rocketry Service expects the application to call this function each time an
 * event is received from the S110 SoftDevice. This function processes the event if it
 * is relevant and calls the Strato Rocketry Service event handler of the
 * application if necessary.
 *
 * @param[in] p_srs       Strato Rocketry Service structure.
 * @param[in] p_ble_evt   Event received from the S110 SoftDevice.
 */
void ble_srs_on_ble_evt(ble_srs_t * p_srs, ble_evt_t * p_ble_evt);

/**@brief Function for setting the temperature.
 *
 * @details This function sends the input voltage as a capacitor characteristic notification to the
 *          peer.
 *
 * @param[in] p_srs       Pointer to the Strato Rocketry Service structure.
 * @param[in] p_data      Pointer to the voltage data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_srs_capacitor_voltage_set(ble_srs_t * p_srs, ble_srs_cap_volt_t * p_data);



#endif // BLE_SRS_H__

/** @} */
