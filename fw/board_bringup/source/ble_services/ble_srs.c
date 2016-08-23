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

#include "ble_srs.h"
#include "ble_srv_common.h"
#include "sdk_common.h"

#define BLE_UUID_SRS_POWER_CHAR                    0x0001
#define BLE_UUID_SRS_IGNITION_CHAR                 0x0002
#define BLE_UUID_SRS_CAP_VOLT_CHAR                 0x0003
#define BLE_UUID_SRS_CAP_CTRL_CHAR                 0x0004
#define BLE_UUID_SRS_PARA_SERVO_CTRL_CHAR          0x0005
#define BLE_UUID_SRS_PARA_SERVO_CONFIG_CHAR        0x0006
#define BLE_UUID_SRS_FIN_CTRL_CHAR                 0x0007


// 4998xxxx-b822-42d3-9ab5-c4913f1b5caa
#define SRS_BASE_UUID                  {{0xaa, 0x5c, 0x1b, 0x3f, 0x91, 0xc4, 0xb5, 0x9a, 0xd3, 0x42, 0x22, 0xb8, 0x00, 0x00, 0x98, 0x49}} /**< Used vendor specific UUID. */

/**@brief Function for handling the @ref BLE_GAP_EVT_CONNECTED event from the S132 SoftDevice.
 *
 * @param[in] p_srs     Strato Rocketry Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_connect(ble_srs_t * p_srs, ble_evt_t * p_ble_evt)
{
    p_srs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

/**@brief Function for handling the @ref BLE_GAP_EVT_DISCONNECTED event from the S132 SoftDevice.
 *
 * @param[in] p_srs     Strato Rocketry Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_disconnect(ble_srs_t * p_srs, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_srs->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the @ref BLE_GATTS_EVT_WRITE event from the S132 SoftDevice.
 *
 * @param[in] p_srs     Strato Rocketry Service structure.
 * @param[in] p_ble_evt Pointer to the event received from BLE stack.
 */
static void on_write(ble_srs_t * p_srs, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ( (p_evt_write->handle == p_srs->power_handles.value_handle) &&
         (p_evt_write->len == 1) )
    {
        if (p_srs->evt_handler != NULL)
        {
            p_srs->evt_handler(p_srs, BLE_SRS_EVT_POWER, p_evt_write->data, p_evt_write->len);
        }
    }
    else if ( (p_evt_write->handle == p_srs->ignition_handles.value_handle) &&
         (p_evt_write->len == 1) )
    {
        if (p_srs->evt_handler != NULL)
        {
            p_srs->evt_handler(p_srs, BLE_SRS_EVT_IGNITION, p_evt_write->data, p_evt_write->len);
        }
    }
    else if ( (p_evt_write->handle == p_srs->cap_ctrl_handles.value_handle) &&
         (p_evt_write->len == 1) )
    {
        if (p_srs->evt_handler != NULL)
        {
            p_srs->evt_handler(p_srs, BLE_SRS_EVT_CAP_CTRL, p_evt_write->data, p_evt_write->len);
        }
    }
    else if ( (p_evt_write->handle == p_srs->cap_volt_handles.cccd_handle) &&
         (p_evt_write->len == 2) )
    {
        if (p_srs->evt_handler != NULL)
        {
            p_srs->evt_handler(p_srs, BLE_SRS_EVT_CAP_VOLT, p_evt_write->data, p_evt_write->len);
        }
    }
    else if ( (p_evt_write->handle == p_srs->para_servo_ctrl_handles.value_handle) &&
              (p_evt_write->len == 1) )
    {
        if (p_srs->evt_handler != NULL)
        {
            p_srs->evt_handler(p_srs, BLE_SRS_EVT_PSERVO_CTRL, p_evt_write->data, p_evt_write->len);
        }
    }
    else if ( (p_evt_write->handle == p_srs->para_servo_config_handles.value_handle) &&
              (p_evt_write->len == 2) )
    {
        if (p_srs->evt_handler != NULL)
        {
            p_srs->evt_handler(p_srs, BLE_SRS_EVT_PSERVO_CONFIG, p_evt_write->data, p_evt_write->len);
        }
    }
    else if ( (p_evt_write->handle == p_srs->fin_ctrl_handles.value_handle) &&
              (p_evt_write->len == 4) )
    {
        if (p_srs->evt_handler != NULL)
        {
            p_srs->evt_handler(p_srs, BLE_SRS_EVT_FIN_CTRL, p_evt_write->data, p_evt_write->len);
        }
    }
    else
    {
        // Do Nothing. This event is not relevant for this service.
    }
}

/**@brief Function for adding temperature characteristic.
 *
 * @param[in] p_srs       Strato Rocketry Service structure.
 * @param[in] p_srs_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t power_char_add(ble_srs_t * p_srs, const ble_srs_init_t * p_srs_init)
{
    /**@snippet [Adding proprietary characteristic to S132 SoftDevice] */
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_srs->uuid_type;
    ble_uuid.uuid = BLE_UUID_SRS_POWER_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(ble_srs_power_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value   = (uint8_t *)&(p_srs_init->power_init);
    attr_char_value.max_len   = sizeof(ble_srs_power_t);

    return sd_ble_gatts_characteristic_add(p_srs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_srs->power_handles);
    /**@snippet [Adding proprietary characteristic to S132 SoftDevice] */
}


/**@brief Function for adding temperature characteristic.
 *
 * @param[in] p_srs       Strato Rocketry Service structure.
 * @param[in] p_srs_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t ignition_char_add(ble_srs_t * p_srs, const ble_srs_init_t * p_srs_init)
{
    /**@snippet [Adding proprietary characteristic to S132 SoftDevice] */
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_srs->uuid_type;
    ble_uuid.uuid = BLE_UUID_SRS_IGNITION_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(ble_srs_ignition_ctrl_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value   = (uint8_t *)&(p_srs_init->ignition_init);
    attr_char_value.max_len   = sizeof(ble_srs_ignition_ctrl_t);

    return sd_ble_gatts_characteristic_add(p_srs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_srs->ignition_handles);
    /**@snippet [Adding proprietary characteristic to S132 SoftDevice] */
}

/**@brief Function for adding altitude characteristic.
 *
 * @param[in] p_srs       Strato Rocketry Service structure.
 * @param[in] p_srs_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t cap_volt_char_add(ble_srs_t * p_srs, const ble_srs_init_t * p_srs_init)
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

    ble_uuid.type = p_srs->uuid_type;
    ble_uuid.uuid = BLE_UUID_SRS_CAP_VOLT_CHAR;

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
    attr_char_value.init_len  = sizeof(ble_srs_cap_volt_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value   = (uint8_t *)p_srs_init->p_cap_init->p_voltage;
    attr_char_value.max_len   = sizeof(ble_srs_cap_volt_t);;

    return sd_ble_gatts_characteristic_add(p_srs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_srs->cap_volt_handles);
    /**@snippet [Adding proprietary characteristic to S132 SoftDevice] */
}

/**@brief Function for adding temperature characteristic.
 *
 * @param[in] p_srs       Strato Rocketry Service structure.
 * @param[in] p_srs_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t cap_ctrl_char_add(ble_srs_t * p_srs, const ble_srs_init_t * p_srs_init)
{
    /**@snippet [Adding proprietary characteristic to S132 SoftDevice] */
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_srs->uuid_type;
    ble_uuid.uuid = BLE_UUID_SRS_CAP_CTRL_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(ble_srs_cap_ctrl_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value   = (uint8_t *)&(p_srs_init->p_cap_init->ctrl);
    attr_char_value.max_len   = sizeof(ble_srs_cap_ctrl_t);

    return sd_ble_gatts_characteristic_add(p_srs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_srs->cap_ctrl_handles);
    /**@snippet [Adding proprietary characteristic to S132 SoftDevice] */
}

/**@brief Function for adding configuration characteristic.
 *
 * @param[in] p_srs       Strato Rocketry Service structure.
 * @param[in] p_srs_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t para_servo_ctrl_char_add(ble_srs_t * p_srs, const ble_srs_init_t * p_srs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read          = 1;
    char_md.char_props.write         = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_srs->uuid_type;
    ble_uuid.uuid = BLE_UUID_SRS_PARA_SERVO_CTRL_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(para_servo_ctrl_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value   = (uint8_t *)&(p_srs_init->p_para_servo_init->ctrl);
    attr_char_value.max_len   = sizeof(para_servo_ctrl_t);

    return sd_ble_gatts_characteristic_add(p_srs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_srs->para_servo_ctrl_handles);
}

/**@brief Function for adding configuration characteristic.
 *
 * @param[in] p_srs       Strato Rocketry Service structure.
 * @param[in] p_srs_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t para_servo_config_char_add(ble_srs_t * p_srs, const ble_srs_init_t * p_srs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read          = 1;
    char_md.char_props.write         = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_srs->uuid_type;
    ble_uuid.uuid = BLE_UUID_SRS_PARA_SERVO_CONFIG_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(para_servo_config_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value   = (uint8_t *)p_srs_init->p_para_servo_init->p_config;
    attr_char_value.max_len   = sizeof(para_servo_config_t);

    return sd_ble_gatts_characteristic_add(p_srs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_srs->para_servo_config_handles);
}

/**@brief Function for adding configuration characteristic.
 *
 * @param[in] p_srs       Strato Rocketry Service structure.
 * @param[in] p_srs_init  Information needed to initialize the service.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t fin_ctrl_char_add(ble_srs_t * p_srs, const ble_srs_init_t * p_srs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read          = 1;
    char_md.char_props.write         = 1;
    char_md.p_char_user_desc         = NULL;
    char_md.p_char_pf                = NULL;
    char_md.p_user_desc_md           = NULL;
    char_md.p_cccd_md                = NULL;
    char_md.p_sccd_md                = NULL;

    ble_uuid.type = p_srs->uuid_type;
    ble_uuid.uuid = BLE_UUID_SRS_FIN_CTRL_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    attr_md.vloc    = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen    = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(ble_srs_fin_ctrl_t);
    attr_char_value.init_offs = 0;
    attr_char_value.p_value   = (uint8_t *)&(p_srs_init->p_fin_init);
    attr_char_value.max_len   = sizeof(ble_srs_fin_ctrl_t);

    return sd_ble_gatts_characteristic_add(p_srs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_srs->fin_ctrl_handles);
}

void ble_srs_on_ble_evt(ble_srs_t * p_srs, ble_evt_t * p_ble_evt)
{
    if ((p_srs == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_srs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_srs, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_srs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_srs_init(ble_srs_t * p_srs, const ble_srs_init_t * p_srs_init)
{
    uint32_t      err_code;
    ble_uuid_t    ble_uuid;
    ble_uuid128_t srs_base_uuid = SRS_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_srs);
    VERIFY_PARAM_NOT_NULL(p_srs_init);

    // Initialize the service structure.
    p_srs->conn_handle                  = BLE_CONN_HANDLE_INVALID;
    p_srs->evt_handler                  = p_srs_init->evt_handler;

    // Add a custom base UUID.
    err_code = sd_ble_uuid_vs_add(&srs_base_uuid, &p_srs->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_srs->uuid_type;
    ble_uuid.uuid = BLE_UUID_SRS_SERVICE;

    // Add the service.
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
                                        &ble_uuid,
                                        &p_srs->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add the altitude Characteristic.
    err_code = power_char_add(p_srs, p_srs_init);
    VERIFY_SUCCESS(err_code);
    // Add the altitude Characteristic.
    err_code = ignition_char_add(p_srs, p_srs_init);
    VERIFY_SUCCESS(err_code);

    // Add the accelCharacteristic.
    err_code = cap_volt_char_add(p_srs, p_srs_init);
    VERIFY_SUCCESS(err_code);

    // Add the temperature Characteristic.
    err_code = cap_ctrl_char_add(p_srs, p_srs_init);
    VERIFY_SUCCESS(err_code);

    // Add the config Characteristic.
    err_code = para_servo_ctrl_char_add(p_srs, p_srs_init);
    VERIFY_SUCCESS(err_code);

    // Add the config Characteristic.
    err_code = para_servo_config_char_add(p_srs, p_srs_init);
    VERIFY_SUCCESS(err_code);

    // Add the config Characteristic.
    err_code = fin_ctrl_char_add(p_srs, p_srs_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

uint32_t ble_srs_capacitor_voltage_set(ble_srs_t * p_srs, ble_srs_cap_volt_t * p_data)
{
    ble_gatts_hvx_params_t hvx_params;
    uint16_t length = sizeof(ble_srs_cap_volt_t);

    VERIFY_PARAM_NOT_NULL(p_srs);

    if (length > BLE_SRS_MAX_DATA_LEN)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    memset(&hvx_params, 0, sizeof(hvx_params));

    hvx_params.handle = p_srs->cap_volt_handles.value_handle;
    hvx_params.p_data = (uint8_t *)p_data;
    hvx_params.p_len  = &length;
    hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;

    return sd_ble_gatts_hvx(p_srs->conn_handle, &hvx_params);
}
