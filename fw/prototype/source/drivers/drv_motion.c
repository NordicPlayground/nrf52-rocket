#include "drv_motion.h"
#include "drv_mpu9250.h"
#include "nrf_error.h"
#include "app_timer.h"
#include "drv_io_ext.h"
#include "drv_io_ext_cfg.h"
#include "drv_acc.h"
#include "app_timer.h"
#include "app_scheduler.h"
#include "thingee_config.h"
#include "nrf_delay.h"

#include "nrf_drv_gpiote.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"

#define PEDO_READ_MS    (1000UL)
#define TEMP_READ_MS    (500UL)
#define COMPASS_READ_MS (200UL)
#define DEFAULT_MPU_HZ  (20UL)

/**@brief Check if the error code is equal to NRF_SUCCESS, if not return the error code.
 */
#define RETURN_IF_ERROR(PARAM)                                                                    \
        if ((PARAM) != NRF_SUCCESS)                                                               \
        {                                                                                         \
            return (PARAM);                                                                       \
        }

#define RETURN_IF_INV_ERROR(PARAM)                                                                \
        if ((PARAM) != INV_SUCCESS)                                                               \
        {                                                                                         \
            return NRF_ERROR_INTERNAL;                                                            \
        }
/**@brief Check if the input pointer is NULL, if so it returns NRF_ERROR_NULL.
 */
#define NULL_PARAM_CHECK(PARAM)                                                                   \
        if ((PARAM) == NULL)                                                                      \
        {                                                                                         \
            return NRF_ERROR_NULL;                                                                \
        }

#define MOTION_DEBUG

#ifdef MOTION_DEBUG
    #include "SEGGER_RTT.h"
    #define DEBUG_PRINTF SEGGER_RTT_printf
#else
    #define DEBUG_PRINTF(...)
#endif

unsigned char *mpl_key = (unsigned char*)"eMPL 5.1";

/* Platform-specific information. Kinda like a boardfile. */
typedef struct
{
    signed char orientation[9];
} platform_data_t;

static platform_data_t s_gyro_pdata =
{
    .orientation = { 1, 0, 0,
                     0, 1, 0,
                     0, 0, 1}
};

static platform_data_t s_compass_pdata =
{
    .orientation = { 0, 1, 0,
                     1, 0, 0,
                     0, 0, -1}
};

static struct
{
    bool                      lp_accel_mode;
    bool                      running;
    uint8_t                   sensors;
    bool                      dmp_on;
    uint16_t                  dmp_features;
    drv_motion_feature_mask_t features;
    drv_motion_evt_handler_t  evt_handler;
    bool                      do_temp;
    bool                      do_compass;
    bool                      do_pedo;
} m_motion;

APP_TIMER_DEF(m_temp_timer_id);
APP_TIMER_DEF(m_compass_timer_id);
APP_TIMER_DEF(m_pedo_timer_id);

static void mpulib_data_send(void)
{
    inv_time_t       timestamp;
    int8_t           accuracy;
    int32_t          data[9];
    drv_motion_evt_t evt;

    if (m_motion.features & DRV_MOTION_FEATURE_MASK_RAW)
    {
        bool valid_raw = false;
        if (m_motion.features & DRV_MOTION_FEATURE_MASK_RAW_ACCEL)
        {
            if (inv_get_sensor_type_accel((long *)&data[0], &accuracy, &timestamp))
            {
                // X, Y, and Z
                valid_raw = true;
            }
            else
            {
                data[0] = 0;
                data[1] = 0;
                data[2] = 0;
            }
        }

        if (m_motion.features & DRV_MOTION_FEATURE_MASK_RAW_GYRO)
        {
            if (inv_get_sensor_type_gyro((long *)&data[3], &accuracy, &timestamp))
            {
                // X, Y, and Z
                valid_raw = true;
            }
            else
            {
                data[3] = 0;
                data[4] = 0;
                data[5] = 0;
            }
        }

        if (m_motion.features & DRV_MOTION_FEATURE_MASK_RAW_COMPASS)
        {
            if (inv_get_sensor_type_compass((long *)&data[6], &accuracy, &timestamp))
            {
                // X, Y, and Z
                valid_raw = true;
            }
            else
            {
                data[6] = 0;
                data[7] = 0;
                data[8] = 0;
            }
        }

        if (valid_raw)
        {
            evt = DRV_MOTION_EVT_RAW;
            m_motion.evt_handler(&evt, data, sizeof(long) * 9);
        }
    }

    if (m_motion.features & DRV_MOTION_FEATURE_MASK_QUAT)
    {
        if (inv_get_sensor_type_quat((long *)data, &accuracy, &timestamp))
        {
            evt = DRV_MOTION_EVT_QUAT;
            // W, X, Y, and Z
            m_motion.evt_handler(&evt, data, sizeof(long) * 4);
        }
    }

    if (m_motion.features & DRV_MOTION_FEATURE_MASK_EULER)
    {
        if (inv_get_sensor_type_euler((long *)data, &accuracy, &timestamp))
        {
            evt = DRV_MOTION_EVT_EULER;
            // Pitch, roll and yaw
            m_motion.evt_handler(&evt, data, sizeof(long) * 3);
        }
    }

    if (m_motion.features & DRV_MOTION_FEATURE_MASK_ROT_MAT)
    {
        if (inv_get_sensor_type_rot_mat((long *)data, &accuracy, &timestamp))
        {
            evt = DRV_MOTION_EVT_ROT_MAT;
            m_motion.evt_handler(&evt, data, sizeof(long) * 9);
        }
    }

    if (m_motion.features & DRV_MOTION_FEATURE_MASK_HEADING)
    {
        if (inv_get_sensor_type_heading((long *)data, &accuracy, &timestamp))
        {
            evt = DRV_MOTION_EVT_HEADING;
            m_motion.evt_handler(&evt, data, sizeof(long));
        }
    }

    if (m_motion.features & DRV_MOTION_FEATURE_MASK_GRAVITY_VECTOR)
    {
        float gravity[3];

        if (inv_get_sensor_type_gravity(gravity, &accuracy, &timestamp))
        {
            evt = DRV_MOTION_EVT_GRAVITY;
            // x, y and z
            m_motion.evt_handler(&evt, data, sizeof(float) * 3);
        }
    }

    if (m_motion.features & DRV_MOTION_FEATURE_MASK_PEDOMETER && m_motion.do_pedo)
    {
        unsigned long pedometer[2]; //step_count, walk_time;
        dmp_get_pedometer_step_count(&pedometer[0]);
        dmp_get_pedometer_walk_time(&pedometer[1]);

        if (pedometer[0] > 0)
        {
            DEBUG_PRINTF(0, "drv_motion: Walked %ld steps over %ld milliseconds..\n", pedometer[0], pedometer[1]);

            evt = DRV_MOTION_EVT_PEDOMETER;
            // step_count and walk_time
            m_motion.evt_handler(&evt, data, sizeof(unsigned long) * 2);
        }
    }

}

static void mpulib_data_handler(void * p_event_data, uint16_t event_size)
{
    unsigned long sensor_timestamp;
    int new_data = 0;
    uint32_t err_code;

    if (m_motion.lp_accel_mode)
    {
        short accel_short[3];
        long accel[3];

        mpu_get_accel_reg(accel_short, &sensor_timestamp);

        accel[0] = (long)accel_short[0];
        accel[1] = (long)accel_short[1];
        accel[2] = (long)accel_short[2];

        inv_build_accel(accel, 0, sensor_timestamp);
        new_data = 1;
    }
    else if (m_motion.dmp_on)
    {
        short gyro[3], accel_short[3], sensors;
        unsigned char more;
        long accel[3], quat[4], temperature;
        /* This function gets new data from the FIFO when the DMP is in
         * use. The FIFO can contain any combination of gyro, accel,
         * quaternion, and gesture data. The sensors parameter tells the
         * caller which data fields were actually populated with new data.
         * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
         * the FIFO isn't being filled with accel data.
         * The driver parses the gesture data to determine if a gesture
         * event has occurred; on an event, the application will be notified
         * via a callback (assuming that a callback function was properly
         * registered). The more parameter is non-zero if there are
         * leftover packets in the FIFO.
         */
        dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);

        if (more)
        {
            err_code = app_sched_event_put(0, 0, mpulib_data_handler);
            APP_ERROR_CHECK(err_code);
        }

        if (sensors & INV_XYZ_GYRO) {
            /* Push the new data to the MPL. */
            inv_build_gyro(gyro, sensor_timestamp);
            new_data = 1;
            if (m_motion.do_temp) {
                m_motion.do_temp = 0;
                /* Temperature only used for gyro temp comp. */
                mpu_get_temperature(&temperature, &sensor_timestamp);
                inv_build_temp(temperature, sensor_timestamp);
            }
        }
        if (sensors & INV_XYZ_ACCEL) {
            accel[0] = (long)accel_short[0];
            accel[1] = (long)accel_short[1];
            accel[2] = (long)accel_short[2];
            inv_build_accel(accel, 0, sensor_timestamp);
            new_data = 1;
        }
        if (sensors & INV_WXYZ_QUAT) {
            inv_build_quat(quat, 0, sensor_timestamp);
            new_data = 1;
        }
    }
    else
    {
        short gyro[3], accel_short[3];
        unsigned char sensors, more;
        long accel[3], temperature;
        /* This function gets new data from the FIFO. The FIFO can contain
         * gyro, accel, both, or neither. The sensors parameter tells the
         * caller which data fields were actually populated with new data.
         * For example, if sensors == INV_XYZ_GYRO, then the FIFO isn't
         * being filled with accel data. The more parameter is non-zero if
         * there are leftover packets in the FIFO. The HAL can use this
         * information to increase the frequency at which this function is
         * called.
         */

        mpu_read_fifo(gyro, accel_short, &sensor_timestamp, &sensors, &more);

        if (more)
        {
            err_code = app_sched_event_put(0, 0, mpulib_data_handler);
            APP_ERROR_CHECK(err_code);
        }

        if (sensors & INV_XYZ_GYRO)
        {
            /* Push the new data to the MPL. */
            inv_build_gyro(gyro, sensor_timestamp);
            new_data = 1;
            if (m_motion.do_temp)
            {
                m_motion.do_temp = 0;
                /* Temperature only used for gyro temp comp. */
                mpu_get_temperature(&temperature, &sensor_timestamp);
                inv_build_temp(temperature, sensor_timestamp);
            }
        }
        if (sensors & INV_XYZ_ACCEL)
        {
            accel[0] = (long)accel_short[0];
            accel[1] = (long)accel_short[1];
            accel[2] = (long)accel_short[2];
            inv_build_accel(accel, 0, sensor_timestamp);
            new_data = 1;
        }
    }

    if (m_motion.do_compass)
    {
        short compass_short[3];
        long compass[3];
        m_motion.do_compass = 0;
        /* For any MPU device with an AKM on the auxiliary I2C bus, the raw
         * magnetometer registers are copied to special gyro registers.
         */
        if (!mpu_get_compass_reg(compass_short, &sensor_timestamp))
        {
            compass[0] = (long)compass_short[0];
            compass[1] = (long)compass_short[1];
            compass[2] = (long)compass_short[2];
            /* NOTE: If using a third-party compass calibration library,
             * pass in the compass data in uT * 2^16 and set the second
             * parameter to INV_CALIBRATED | acc, where acc is the
             * accuracy from 0 to 3.
             */
            inv_build_compass(compass, 0, sensor_timestamp);
        }
        new_data = 1;
    }

    if (new_data) {
        inv_execute_on_data();
        mpulib_data_send();
    }
}

static void mpulib_data_handler_cb(void)
{
    uint32_t err_code;

    err_code = app_sched_event_put(0, 0, mpulib_data_handler);
    APP_ERROR_CHECK(err_code);
}

static void mpulib_tap_cb(unsigned char direction, unsigned char count)
{
    if (m_motion.features & DRV_MOTION_FEATURE_MASK_TAP)
    {
        drv_motion_evt_t evt     = DRV_MOTION_EVT_TAP;
        uint8_t          data[2] = {direction, count};

        m_motion.evt_handler(&evt, &data, sizeof(data));
    }

#ifdef MOTION_DEBUG
    switch (direction)
    {
        case TAP_X_UP:
            DEBUG_PRINTF(0, "drv_motion: tap x+ ");
            break;
        case TAP_X_DOWN:
            DEBUG_PRINTF(0, "drv_motion: tap x- ");
            break;
        case TAP_Y_UP:
            DEBUG_PRINTF(0, "drv_motion: tap y+ ");
            break;
        case TAP_Y_DOWN:
            DEBUG_PRINTF(0, "drv_motion: tap y- ");
            break;
        case TAP_Z_UP:
            DEBUG_PRINTF(0, "drv_motion: tap z+ ");
            break;
        case TAP_Z_DOWN:
            DEBUG_PRINTF(0, "drv_motion: tap z- ");
            break;
        default:
            return;
    }

    DEBUG_PRINTF(0, "x%d\r\n", count);
#endif
}

static void mpulib_orient_cb(unsigned char orientation)
{
    if (m_motion.features & DRV_MOTION_FEATURE_MASK_ORIENTATION)
    {
        drv_motion_evt_t evt     = DRV_MOTION_EVT_ORIENTATION;

        m_motion.evt_handler(&evt, &orientation, 1);
    }

#ifdef MOTION_DEBUG
	switch (orientation)
    {
        case ANDROID_ORIENT_PORTRAIT:
            DEBUG_PRINTF(0, "Portrait\n");
            break;
        case ANDROID_ORIENT_LANDSCAPE:
            DEBUG_PRINTF(0, "Landscape\n");
            break;
        case ANDROID_ORIENT_REVERSE_PORTRAIT:
            DEBUG_PRINTF(0, "Reverse Portrait\n");
            break;
        case ANDROID_ORIENT_REVERSE_LANDSCAPE:
            DEBUG_PRINTF(0, "Reverse Landscape\n");
            break;
        default:
            return;
	}
#endif
}

static uint32_t mpulib_init(void)
{
    inv_error_t result;
    unsigned char accel_fsr;
    unsigned short gyro_rate;
    unsigned short gyro_fsr;
    unsigned short compass_fsr;
    struct int_param_s int_param;

    int_param.cb = mpulib_data_handler_cb;

    result = mpu_init(&int_param);
    RETURN_IF_INV_ERROR(result);

    /* Compute 6-axis and 9-axis quaternions. */
    inv_enable_quaternion();
    inv_enable_9x_sensor_fusion();
    /* Update gyro biases when not in motion. */
    inv_enable_fast_nomot();
    /* Update gyro biases when temperature changes. */
    inv_enable_gyro_tc();
    /* This algorithm updates the accel biases when in motion. A more accurate
     * bias measurement can be made when running the self-test */
    inv_enable_in_use_auto_calibration();
    /* Compass calibration algorithms. */
    inv_enable_vector_compass_cal();
    inv_enable_magnetic_disturbance();

    /* Allows use of the MPL APIs in read_from_mpl. */
    inv_enable_eMPL_outputs();

    result = inv_start_mpl();
    RETURN_IF_INV_ERROR(result);

    mpu_set_sensors(m_motion.sensors);

    /* Push both gyro, accel and compass data into the FIFO. */
    mpu_configure_fifo(m_motion.sensors);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);

    /* The compass sampling rate can be less than the gyro/accel sampling rate.
     * Use this function for proper power management.
     */
    mpu_set_compass_sample_rate(1000UL / COMPASS_READ_MS);

    /* Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);
    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);
    mpu_get_compass_fsr(&compass_fsr);

    /* Sync driver configuration with MPL. */
    /* Sample rate expected in microseconds. */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);

    /* The compass rate is independent of the gyro and accel rates. As long as
     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
     * fusion algorithm's compass correction gain will work properly.
     */
    inv_set_compass_sample_rate(COMPASS_READ_MS * 1000L);

    /* Set chip-to-body orientation matrix.
     * Set hardware units to dps/g's/degrees scaling factor.
     */
    inv_set_gyro_orientation_and_scale(
            inv_orientation_matrix_to_scalar(s_gyro_pdata.orientation),
            (long)gyro_fsr<<15);
    inv_set_accel_orientation_and_scale(
            inv_orientation_matrix_to_scalar(s_gyro_pdata.orientation),
            (long)accel_fsr<<15);
    inv_set_compass_orientation_and_scale(
            inv_orientation_matrix_to_scalar(s_compass_pdata.orientation),
            (long)compass_fsr<<15);

    /* Initialize DMP */
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(s_gyro_pdata.orientation));
    dmp_register_tap_cb(mpulib_tap_cb);
    dmp_register_android_orient_cb(mpulib_orient_cb);

    m_motion.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    dmp_enable_feature(m_motion.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    mpu_set_dmp_state(1);

    return NRF_SUCCESS;
}

static uint32_t mpu9250_power(bool enable)
{
    uint32_t err_code;

    if (enable)
    {
        err_code = drv_io_ext_pins_clr(IO_EXT_PIN_MPU_PWR_CTRL_PIN);
        RETURN_IF_ERROR(err_code);
    }
    else
    {
        err_code = drv_io_ext_pins_set(IO_EXT_PIN_MPU_PWR_CTRL_PIN);
        RETURN_IF_ERROR(err_code);
    }

    return NRF_SUCCESS;
}

/**@brief Function for handling temperature timer timeout event.
 */
static void temp_timeout_handler(void * p_context)
{
    m_motion.do_temp = true;
}

/**@brief Function for handling compass timer timeout event.
 */
static void compass_timeout_handler(void * p_context)
{
    m_motion.do_compass = true;
}

/**@brief Function for handling pedometer timer timeout event.
 */
static void pedo_timeout_handler(void * p_context)
{
    m_motion.do_pedo = true;
}

uint32_t drv_motion_enable(drv_motion_feature_mask_t feature_mask)
{
    uint32_t err_code;

    if ( (feature_mask & ~(DRV_MOTION_FEATURE_MASK)) ||
         (feature_mask == 0) )
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    /* Set features bits */
    m_motion.features |= feature_mask;
    /* Set enabled sensors */
    m_motion.sensors |= (INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS | INV_WXYZ_QUAT);

    if (!m_motion.running)
    {
        m_motion.running = true;

        err_code = mpu9250_power(true);
        RETURN_IF_ERROR(err_code);

        // TODO: 100ms probably not needed
        nrf_delay_ms(100);

        m_motion.dmp_on = true;
        err_code = mpulib_init();
        RETURN_IF_ERROR(err_code);

        err_code = app_timer_start(m_temp_timer_id,
                                   APP_TIMER_TICKS(TEMP_READ_MS, APP_TIMER_PRESCALER),
                                   NULL);
        RETURN_IF_ERROR(err_code);

        err_code = app_timer_start(m_compass_timer_id,
                                   APP_TIMER_TICKS(COMPASS_READ_MS, APP_TIMER_PRESCALER),
                                   NULL);
        RETURN_IF_ERROR(err_code);

        err_code = app_timer_start(m_pedo_timer_id,
                                   APP_TIMER_TICKS(PEDO_READ_MS, APP_TIMER_PRESCALER),
                                   NULL);
        RETURN_IF_ERROR(err_code);
    }

    return NRF_SUCCESS;
}

uint32_t drv_motion_disable(drv_motion_feature_mask_t feature_mask)
{
    uint32_t err_code;

    if ( (feature_mask & ~(DRV_MOTION_FEATURE_MASK)) ||
         (feature_mask == 0) )
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    /* Clear feature bits*/
    m_motion.features &= ~feature_mask;

    if (!m_motion.features)
    {
        nrf_drv_gpiote_in_event_disable(MPU_INT);
        nrf_drv_gpiote_in_uninit(MPU_INT);

        err_code = mpu9250_power(false);
        RETURN_IF_ERROR(err_code);

        m_motion.running = false;

        err_code = app_timer_stop(m_temp_timer_id);
        RETURN_IF_ERROR(err_code);
        err_code = app_timer_stop(m_compass_timer_id);
        RETURN_IF_ERROR(err_code);
        err_code = app_timer_stop(m_pedo_timer_id);
        RETURN_IF_ERROR(err_code);
    }

    return NRF_SUCCESS;

}

uint32_t drv_motion_init(drv_motion_init_t * p_params)
{
    uint32_t           err_code;
    drv_mpu9250_init_t init_params;
    drv_acc_cfg_t      lis_init_params;

    NULL_PARAM_CHECK(p_params);
    NULL_PARAM_CHECK(p_params->evt_handler);
    NULL_PARAM_CHECK(p_params->p_twi_instance);

    lis_init_params.p_twi_instance = p_params->p_twi_instance;
    lis_init_params.p_twi_cfg      = p_params->p_twi_cfg;
    lis_init_params.twi_addr       = 0x18;

    init_params.p_twi_instance     = p_params->p_twi_instance;
    init_params.p_twi_cfg          = p_params->p_twi_cfg;

    m_motion.evt_handler           = p_params->evt_handler;
    m_motion.features              = 0;
    m_motion.sensors               = 0;
    m_motion.dmp_features          = 0;
    m_motion.dmp_on                = false;
    m_motion.running               = false;
    m_motion.lp_accel_mode         = false;
    m_motion.do_temp               = false;
    m_motion.do_compass            = false;
    m_motion.do_pedo               = false;

    err_code = drv_io_ext_pins_inp_cfg(IO_EXT_PIN_LIS_INT2_PIN);
    RETURN_IF_ERROR(err_code);

    err_code = drv_acc_init(&lis_init_params);
    RETURN_IF_ERROR(err_code);

    err_code = drv_mpu9250_init(&init_params);
    RETURN_IF_ERROR(err_code);

    /* Init power pin and power off the mpu9250 chip */
    err_code = drv_io_ext_pins_out_cfg(IO_EXT_PIN_MPU_PWR_CTRL_PIN);
    RETURN_IF_ERROR(err_code);

    err_code = mpu9250_power(false);
    RETURN_IF_ERROR(err_code);

    /**@brief Init application timers */
    err_code = app_timer_create(&m_temp_timer_id, APP_TIMER_MODE_REPEATED, temp_timeout_handler);
    RETURN_IF_ERROR(err_code);

    err_code = app_timer_create(&m_compass_timer_id, APP_TIMER_MODE_REPEATED, compass_timeout_handler);
    RETURN_IF_ERROR(err_code);

    err_code = app_timer_create(&m_pedo_timer_id, APP_TIMER_MODE_REPEATED, pedo_timeout_handler);
    RETURN_IF_ERROR(err_code);


    return NRF_SUCCESS;
}
