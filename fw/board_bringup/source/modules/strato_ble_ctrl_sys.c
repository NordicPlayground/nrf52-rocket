#include "strato_ble_ctrl_sys.h"
#include "ble_gap.h"
#include "softdevice_handler.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble.h"
#include "app_error.h"
#include "nordic_common.h"
#include "nrf.h"
#include "pca20027.h"
#include "strato_app_config.h"
#include "drv_sky66112_pa_lna.h"
#include "ble_hci.h"
#include "ble_sts.h"
#include "ble_srs.h"
#include "strato_led.h"
#include "strato_ignition.h"
#include "strato_parachute_fins.h"
#include "strato_sensors.h"
#include "drv_servo.h"
#include "SEGGER_RTT.h"
#include "nrf_delay.h"

static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_sts_t                        m_sts;                                      /**< Instance of Strato Telemetry Service. */
static ble_srs_t                        m_srs;                                      /**< Instance of Strato Telemetry Service. */

//Forward declarations
static void ble_sts_evt_handler(ble_sts_t        * p_sts,
                                ble_sts_evt_type_t evt_type,
                                uint8_t          * p_data,
                                uint16_t           length);

static void ble_srs_evt_handler(ble_srs_t        * p_srs,
                                ble_srs_evt_type_t evt_type,
                                uint8_t          * p_data,
                                uint16_t           length);

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;

    ble_uuid_t adv_uuids = {BLE_UUID_STS_SERVICE, m_sts.uuid_type};

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    advdata.uuids_more_available.uuid_cnt = 1;
    advdata.uuids_more_available.p_uuids = &adv_uuids;

    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    //Init Telemetry Service
    uint32_t err_code;
    ble_sts_temperature_t temp_init =
    {
        .integer = 0,
        .decimal = 0
    };

    ble_sts_altitude_t alti_init =
    {
        .current = 0,
        .max = 0,
        .vertical_velocity = 0
    };

    ble_sts_accel_t accel_init =
    {
        .x = 0,
        .y = 0,
        .z = 0
    };

    ble_sts_config_t config_init =
    {
        .temperature_interval_ms = TEMP_SAMPLE_PERIOD_MS,
        .altitude_interval_ms = ALTITUDE_SAMPLE_PERIOD_MS,
        .accel_interval_ms = ACCEL_SAMPLE_PERIOD_MS,
        .pressure_mode = STS_PRESSURE_MODE_ALTIMETER,
    };

    ble_sts_init_t sts_init =
    {
        .p_init_temperature = &temp_init,
        .p_init_altitude = &alti_init,
        .p_init_accel = &accel_init,
        .p_init_config = &config_init,
        .evt_handler = ble_sts_evt_handler
    };
    err_code = ble_sts_init(&m_sts, &sts_init);
    APP_ERROR_CHECK(err_code);

    //Init Rocketry Service
    para_servo_config_t servo_config =
    {
        .position_open = 90,
        .position_closed = 0,
    };

    ble_srs_parachute_servo_t para_servo =
    {
        .p_config = &servo_config,
        .ctrl = PARA_SERVO_CLOSE
    };

    ble_srs_cap_volt_t cap_volt =
    {
        .integer = 0,
        .decimal = 0
    };

    ble_srs_cap_t super_cap =
    {
        .ctrl = BLE_SRS_CAP_IDLE,
        .p_voltage = &cap_volt
    };

    ble_srs_init_t srs_init =
    {
        .power_init = BLE_SRS_POWER_OFF,
        .p_cap_init = &super_cap,
        .ignition_init = BLE_SRS_IGNITION_OFF,
        .p_para_servo_init = &para_servo,
        .evt_handler = ble_srs_evt_handler
    };

    err_code = ble_srs_init(&m_srs, &srs_init);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Application's BLE stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            leds_set_rgb(leds_current_value_get() | 0x0000FF);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            leds_set_rgb(leds_current_value_get() ^ 0x0000FF);
            err_code = ignition_cap_adc_sample_end();
            APP_ERROR_CHECK(err_code);
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_sts_on_ble_evt(&m_sts, p_ble_evt);
    ble_srs_on_ble_evt(&m_srs, p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    ble_enable_params.common_enable_params.vs_uuid_count = 2;
    ble_enable_params.gatts_enable_params.attr_tab_size = 2048;

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    ble_gap_addr_t addr;

    err_code = sd_ble_gap_address_get(&addr);
    APP_ERROR_CHECK(err_code);
    err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &addr);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);

    //Start up HFCLK crystal to reduce servo jitter
    err_code = sd_clock_hfclk_request();
    APP_ERROR_CHECK(err_code);

}

static void radio_power_amp_init()
{
    drv_sky66112_init(CTX, CRX, PA_LNA_ANT2);
    drv_sky66112_tx_high_power();
    drv_sky66112_rx_lna();

//    drv_sky66112_bypass();
}

static void supercap_voltage_evt_handler( double result )
{
    ble_srs_cap_volt_t cap_volt;

    //RESULT = [V(P) – V(N) ] * GAIN/REFERENCE * 2^(RESOLUTION - m)
    //V(P) = result*reference*(1/gain)/(2^10)
    //Vsc = 2*V(P)  (voltage divider)

    cap_volt.integer = (uint8_t)(result);
    double d_decimal = result - cap_volt.integer;
    cap_volt.decimal = (uint8_t)(d_decimal*100);

    ret_code_t err_code;
    err_code = ble_srs_capacitor_voltage_set(&m_srs,&cap_volt);
    APP_ERROR_CHECK(err_code);
}

static void ble_sts_evt_handler(ble_sts_t        * p_sts,
                                ble_sts_evt_type_t evt_type,
                                uint8_t          * p_data,
                                uint16_t           length)
{
    switch (evt_type) {
        case BLE_STS_EVT_NOTIF_ALTITUDE:
            if (*p_data == 1)
            {
                ret_code_t err_code;
                err_code = strato_altitude_enable();
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                ret_code_t err_code;
                err_code = strato_altitude_disable();
                APP_ERROR_CHECK(err_code);
            }
            break;
        case BLE_STS_EVT_CONFIG_RECEIVED:
            {
                ret_code_t err_code;
                ble_sts_config_t * p_config = (ble_sts_config_t *)p_data;
                int16_t altitude_period = p_config->altitude_interval_ms;
                err_code = strato_altitude_sample_period_set(altitude_period);
                APP_ERROR_CHECK(err_code);
            }
            break;
    }
}

static void ble_srs_evt_handler(ble_srs_t        * p_srs,
                                ble_srs_evt_type_t evt_type,
                                uint8_t          * p_data,
                                uint16_t           length)
{
        switch (evt_type) {
        case BLE_SRS_EVT_POWER:
            power_5v_enable((bool)(*p_data));
            break;
        case BLE_SRS_EVT_IGNITION:
            switch ((ble_srs_ignition_ctrl_t)(*p_data)) {
                case BLE_SRS_IGNITION_OFF:
                    ignition_trigger_off(1);
                    ignition_trigger_off(2);
                    break;
                //TODO: check for invalid state which means voltage too low,
                //then throw some kind of error? or just let mobile side show the user
                case BLE_SRS_IGNITION1_ON:
                    ignition_trigger_on(1);
                    break;
                case BLE_SRS_IGNITION2_ON:
                    ignition_trigger_on(2);
                    break;
                case BLE_SRS_IGNITION_BOTH:
                    ignition_trigger_on(1);
                    ignition_trigger_on(2);
                    break;
            }
            break;
        case BLE_SRS_EVT_CAP_CTRL:
            ignition_dump_cap((bool)(*p_data));
            break;
        case BLE_SRS_EVT_CAP_VOLT:
            if (*p_data == 1)
            {
                ignition_cap_adc_sample_begin();
            }
            else
            {
                ignition_cap_adc_sample_end();
            }
            break;
        case BLE_SRS_EVT_PSERVO_CTRL:
            if ((para_servo_ctrl_t)(*p_data) == PARA_SERVO_OPEN)
            {
                parachute_hatch_open();
            }
            else if ((para_servo_ctrl_t)(*p_data) == PARA_SERVO_CLOSE)
            {
                parachute_hatch_close();
            }
            break;
        case BLE_SRS_EVT_PSERVO_CONFIG:
        {
            para_servo_config_t * p_config = (para_servo_config_t *)(p_data);
            parachute_end_values_set(p_config->position_open, p_config->position_closed);
            break;
        }
        case BLE_SRS_EVT_FIN_CTRL:
        {
            ble_srs_fin_ctrl_t * p_fin_ctrl = (ble_srs_fin_ctrl_t *)(p_data);
            fin_values_set((fin_degrees_t *)p_fin_ctrl);
            break;
        }
    }
}

void altitude_data_evt_handler(strato_altitude_data_t * p_data)
{

    ret_code_t err_code;
    SEGGER_RTT_printf(0, "Altitude: %d \r\n",p_data->current);
    SEGGER_RTT_printf(0, "Max Altitude: %d \r\n",p_data->max);
    SEGGER_RTT_printf(0, "Vertical Velocity: %d \r\n",p_data->vertical_velocity);
    err_code = ble_sts_altitude_set(&m_sts,(ble_sts_altitude_t *)p_data);
}

void accel_data_evt_handler(int16_t x, int16_t y, int16_t z)
{

}

static void strato_rocketry_system_init(void)
{
    ignition_init_t ignition_init_params =
    {
        .adc_evt_handler = supercap_voltage_evt_handler,
        .adc_sampling_period_ms = SUPERCAP_SAMPLE_PERIOD_MS
    };

    ignition_init(&ignition_init_params);
    parachute_fins_init();
}

static void strato_telemetry_system_init(void)
{
    strato_sensors_init(altitude_data_evt_handler, accel_data_evt_handler);
    strato_altitude_gnd_zero();
}

void strato_ble_ctrl_sys_init(void)
{
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();

    strato_rocketry_system_init();
    strato_telemetry_system_init();

    radio_power_amp_init();

    // Start execution.
    advertising_start();



}
