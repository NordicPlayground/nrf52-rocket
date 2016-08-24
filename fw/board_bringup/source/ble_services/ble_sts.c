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

#include "ble_sts.h"
#include "ble_srv_common.h"
#include "sdk_common.h"

#define BLE_UUID_STS_ALTITUDE_CHAR    0x0002                      /**< The UUID of the altitude Characteristic. */
#define BLE_UUID_STS_ACCEL_CHAR       0x0003                      /**< The UUID of the accel Characteristic. */
#define BLE_UUID_STS_TEMP_CHAR        0x0004                      /**< The UUID of the temperature Characteristic. */
#define BLE_UUID_STS_CONFIG_CHAR      0x0005                      /**< The UUID of the config Characteristic. */

// 1beexxxx-5806-11e6-8b77-86f30ca893d3
#define STS_BASE_UUID                  {{0xd3, 0x93, 0xa8, 0x0c, 0xf3, 0x86, 0x77, 0x8b, 0xe6, 0x11, 0x06, 0x58, 0x00, 0x00, 0xee, 0x1b}} /**< Used vendor specific UUID. */

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S132 SoftDevice.
 *
 * @param[in] p_sts     Strato Telemetry Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_sts_t * p_sts, ble_evt_t * p_ble_evt)
{
    p_sts->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S132 SoftDevice.
 *
 * @param[in] p_sts     Strato Telemetry Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_sts_t * p_sts, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_sts->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S132 SoftDevice.
 *
 * @param[in] p_sts     Strato Telemetry Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_sts_t * p_sts, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;


    if ( (p_evt_write->handle == p_sts->temperature_handles.cccd_handle) &&
         (p_evt_write->len == 2) )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_sts->is_temperature_notif_enabled = true;
        }
        else
        {
            p_sts->is_temperature_notif_enabled = false;
        }

        if (p_sts->evt_handler != NULL)
        {
            p_sts->evt_handler(p_sts, BLE_STS_EVT_NOTIF_TEMPERATURE, p_evt_write->data, p_evt_write->len);
        }
    }
    else if ( (p_evt_write->handle == p_sts->altitude_handles.cccd_handle) &&
         (p_evt_write->len == 2) )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_sts->is_altitude_notif_enabled = true;
        }
        else
        {
            p_sts->is_altitude_notif_enabled = false;
        }

        if (p_sts->evt_handler != NULL)
        {
            p_sts->evt_handler(p_sts, BLE_STS_EVT_NOTIF_ALTITUDE, p_evt_write->data, p_evt_write->len);
        }
    }
    else if ( (p_evt_write->handle == p_sts->accel_handles.cccd_handle) &&
         (p_evt_write->len == 2) )
    {
        if (ble_srv_is_notification_enabled(p_evt_write->data))
        {
            p_sts->is_accel_notif_enabled = true;
        }
        else
        {
            p_sts->is_accel_notif_enabled = false;
        }

        if (p_sts->evt_handler != NULL)
        {
            p_sts->evt_handler(p_sts, BLE_STS_EVT_NOTIF_ACCEL, p_evt_write->data, p_evt_write->len);
        }
    }
    else if ( (p_evt_write->handle == p_sts->config_handles.value_handle) &&
              (p_sts->evt_handler != NULL) && (p_evt_write->len == sizeof(ble_sts_config_t)) )
    {
        p_sts->evt_handler(p_sts, BLE_STS_EVT_CONFIG_RECEIVED, p_evt_write->data, p_evt_write->len);
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}

/**@brief Function for adding temperature characteristic.
 *
 * @param[in] p_sts       Strato Telemetry Service structure.
 * @param[in] p_sts_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t temperature_char_add(ble_sts_t * p_sts, const ble_sts_init_t * p_sts_init)
{
    /**@snippet [Adding proprietary characteristic to S132 SoftDevice] */
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_sts->uuid_type;
    ble_uuid.uuid = BLE_UUID_STS_TEMP_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(ble_sts_temperature_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value   = (uint8_t *)p_sts_init->p_init_temperature;
    attr_char_value.max_len   = sizeof(ble_sts_temperature_t);

    return sd_ble_gatts_characteristic_add(p_sts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sts->temperature_handles);
    /**@snippet [Adding proprietary characteristic to S132 SoftDevice] */
}

/**@brief Function for adding altitude characteristic.
 *
 * @param[in] p_sts       Strato Telemetry Service structure.
 * @param[in] p_sts_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t altitude_char_add(ble_sts_t * p_sts, const ble_sts_init_t * p_sts_init)
{
    /**@snippet [Adding proprietary characteristic to S132 SoftDevice] */
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_sts->uuid_type;
    ble_uuid.uuid = BLE_UUID_STS_ALTITUDE_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(ble_sts_altitude_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value   = (uint8_t *)p_sts_init->p_init_altitude;
    attr_char_value.max_len   = sizeof(ble_sts_altitude_t);;

    return sd_ble_gatts_characteristic_add(p_sts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sts->altitude_handles);
    /**@snippet [Adding proprietary characteristic to S132 SoftDevice] */
}

/**@brief Function for adding accelcharacteristic.
 *
 * @param[in] p_sts       Strato Telemetry Service structure.
 * @param[in] p_sts_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t accel_char_add(ble_sts_t * p_sts, const ble_sts_init_t * p_sts_init)
{
    /**@snippet [Adding proprietary characteristic to S132 SoftDevice] */
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_sts->uuid_type;
    ble_uuid.uuid = BLE_UUID_STS_ACCEL_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(ble_sts_accel_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value   = (uint8_t *)p_sts_init->p_init_accel;
    attr_char_value.max_len   = sizeof(ble_sts_accel_t);

    return sd_ble_gatts_characteristic_add(p_sts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sts->accel_handles);
    /**@snippet [Adding proprietary characteristic to S132 SoftDevice] */
}

/**@brief Function for adding configuration characteristic.
 *
 * @param[in] p_sts       Strato Telemetry Service structure.
 * @param[in] p_sts_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t config_char_add(ble_sts_t * p_sts, const ble_sts_init_t * p_sts_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read          = 1;
    char_md.char_props.write         = 1;
    char_md.char_props.write_wo_resp = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_sts->uuid_type;
    ble_uuid.uuid = BLE_UUID_STS_CONFIG_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(ble_sts_config_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value   = (uint8_t *)p_sts_init->p_init_config;
    attr_char_value.max_len   = sizeof(ble_sts_config_t);

    return sd_ble_gatts_characteristic_add(p_sts->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_sts->config_handles);
}


void ble_sts_on_ble_evt(ble_sts_t * p_sts, ble_evt_t * p_ble_evt)
{
    if ((p_sts == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_sts, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_sts, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_sts, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_sts_init(ble_sts_t * p_sts, const ble_sts_init_t * p_sts_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t sts_base_uuid = STS_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_sts);
    VERIFY_PARAM_NOT_NULL(p_sts_init);

    // Initialize the service structure.
    p_sts->conn_handle                  = BLE_CONN_HANDLE_INVALID;
    p_sts->evt_handler                  = p_sts_init->evt_handler;
    p_sts->is_temperature_notif_enabled = false;
    p_sts->is_altitude_notif_enabled    = false;
    p_sts->is_accel_notif_enabled    = false;

    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&sts_base_uuid, &p_sts->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_sts->uuid_type;
    ble_uuid.uuid = BLE_UUID_STS_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_sts->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add the altitude Characteristic.
    err_code = altitude_char_add(p_sts, p_sts_init);
    VERIFY_SUCCESS(err_code);

    // Add the accelCharacteristic.
    err_code = accel_char_add(p_sts, p_sts_init);
    VERIFY_SUCCESS(err_code);

    // Add the temperature Characteristic.
    err_code = temperature_char_add(p_sts, p_sts_init);
    VERIFY_SUCCESS(err_code);

    // Add the config Characteristic.
    err_code = config_char_add(p_sts, p_sts_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t ble_sts_temperature_set(ble_sts_t * p_sts, ble_sts_temperature_t * p_data)
{
    ble_gatts_hvx_params_t hvx_params;
    uint16_t               length = sizeof(ble_sts_temperature_t);

    VERIFY_PARAM_NOT_NULL(p_sts);

    if ((p_sts->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_sts->is_temperature_notif_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_STS_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_sts->temperature_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_sts->conn_handle, &hvx_params);
}

uint32_t ble_sts_altitude_set(ble_sts_t * p_sts, ble_sts_altitude_t * p_data)
{
    ble_gatts_hvx_params_t hvx_params;
    uint16_t               length = sizeof(ble_sts_altitude_t);

    VERIFY_PARAM_NOT_NULL(p_sts);

    if ((p_sts->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_sts->is_altitude_notif_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_STS_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_sts->altitude_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_sts->conn_handle, &hvx_params);
}

uint32_t ble_sts_accel_set(ble_sts_t * p_sts, ble_sts_accel_t * p_data)
{
    ble_gatts_hvx_params_t hvx_params;
    uint16_t               length = sizeof(ble_sts_accel_t);

    VERIFY_PARAM_NOT_NULL(p_sts);

    if ((p_sts->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_sts->is_accel_notif_enabled))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (length > BLE_STS_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_sts->accel_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_sts->conn_handle, &hvx_params);
}
