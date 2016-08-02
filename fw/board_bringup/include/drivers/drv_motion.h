#ifndef __DRV_MOTION_H__
#define __DRV_MOTION_H__

#include <stdint.h>
#include <stddef.h>
#include "nrf_drv_twi.h"

typedef enum
{
    DRV_MOTION_FEATURE_RAW_ACCEL,
    DRV_MOTION_FEATURE_RAW_GYRO,
    DRV_MOTION_FEATURE_RAW_COMPASS,
    DRV_MOTION_FEATURE_QUAT,
    DRV_MOTION_FEATURE_EULER,
    DRV_MOTION_FEATURE_ROT_MAT,
    DRV_MOTION_FEATURE_HEADING,
    DRV_MOTION_FEATURE_GRAVITY_VECTOR,
    DRV_MOTION_FEATURE_TAP,
    DRV_MOTION_FEATURE_ORIENTATION,
    DRV_MOTION_FEATURE_PEDOMETER,
    DRV_MOTION_FEATURE_WAKE_ON_MOTION
}drv_motion_feature_t;

typedef uint32_t drv_motion_feature_mask_t;

#define DRV_MOTION_FEATURE_MASK_RAW               ((1UL << DRV_MOTION_FEATURE_RAW_ACCEL) | (1UL << DRV_MOTION_FEATURE_RAW_COMPASS) | (1UL << DRV_MOTION_FEATURE_RAW_GYRO))
#define DRV_MOTION_FEATURE_MASK_RAW_ACCEL         (1UL << DRV_MOTION_FEATURE_RAW_ACCEL)
#define DRV_MOTION_FEATURE_MASK_RAW_GYRO          (1UL << DRV_MOTION_FEATURE_RAW_GYRO)
#define DRV_MOTION_FEATURE_MASK_RAW_COMPASS       (1UL << DRV_MOTION_FEATURE_RAW_COMPASS)
#define DRV_MOTION_FEATURE_MASK_QUAT              (1UL << DRV_MOTION_FEATURE_QUAT)
#define DRV_MOTION_FEATURE_MASK_EULER             (1UL << DRV_MOTION_FEATURE_EULER)
#define DRV_MOTION_FEATURE_MASK_ROT_MAT           (1UL << DRV_MOTION_FEATURE_ROT_MAT)
#define DRV_MOTION_FEATURE_MASK_HEADING           (1UL << DRV_MOTION_FEATURE_HEADING)
#define DRV_MOTION_FEATURE_MASK_GRAVITY_VECTOR    (1UL << DRV_MOTION_FEATURE_GRAVITY_VECTOR)
#define DRV_MOTION_FEATURE_MASK_TAP               (1UL << DRV_MOTION_FEATURE_TAP)
#define DRV_MOTION_FEATURE_MASK_ORIENTATION       (1UL << DRV_MOTION_FEATURE_ORIENTATION)
#define DRV_MOTION_FEATURE_MASK_PEDOMETER         (1UL << DRV_MOTION_FEATURE_PEDOMETER)
#define DRV_MOTION_FEATURE_MASK_WAKE_ON_MOTION    (1UL << DRV_MOTION_FEATURE_WAKE_ON_MOTION)

#define DRV_MOTION_FEATURE_MASK                   (DRV_MOTION_FEATURE_MASK_RAW_ACCEL      |     \
                                                   DRV_MOTION_FEATURE_MASK_RAW_GYRO       |     \
                                                   DRV_MOTION_FEATURE_MASK_RAW_COMPASS    |     \
                                                   DRV_MOTION_FEATURE_MASK_QUAT           |     \
                                                   DRV_MOTION_FEATURE_MASK_EULER          |     \
                                                   DRV_MOTION_FEATURE_MASK_ROT_MAT        |     \
                                                   DRV_MOTION_FEATURE_MASK_HEADING        |     \
                                                   DRV_MOTION_FEATURE_MASK_GRAVITY_VECTOR |     \
                                                   DRV_MOTION_FEATURE_MASK_TAP            |     \
                                                   DRV_MOTION_FEATURE_MASK_ORIENTATION    |     \
                                                   DRV_MOTION_FEATURE_MASK_PEDOMETER      |     \
                                                   DRV_MOTION_FEATURE_MASK_WAKE_ON_MOTION)

#define DRV_MOTION_FEATURE_DMP_MASK               (DRV_MOTION_FEATURE_MASK_QUAT           |     \
                                                   DRV_MOTION_FEATURE_MASK_EULER          |     \
                                                   DRV_MOTION_FEATURE_MASK_ROT_MAT        |     \
                                                   DRV_MOTION_FEATURE_MASK_HEADING        |     \
                                                   DRV_MOTION_FEATURE_MASK_GRAVITY_VECTOR |     \
                                                   DRV_MOTION_FEATURE_MASK_TAP            |     \
                                                   DRV_MOTION_FEATURE_MASK_ORIENTATION    |     \
                                                   DRV_MOTION_FEATURE_MASK_PEDOMETER)

#define DRV_MOTION_FEATURE_GESTURE_MASK           (DRV_MOTION_FEATURE_MASK_TAP            |     \
                                                   DRV_MOTION_FEATURE_MASK_ORIENTATION    |     \
                                                   DRV_MOTION_FEATURE_MASK_PEDOMETER)

#define DRV_MOTION_FEATURE_CONTINUOS_MASK         (DRV_MOTION_FEATURE_MASK_RAW_ACCEL      |     \
                                                   DRV_MOTION_FEATURE_MASK_RAW_GYRO       |     \
                                                   DRV_MOTION_FEATURE_MASK_RAW_COMPASS    |     \
                                                   DRV_MOTION_FEATURE_MASK_QUAT           |     \
                                                   DRV_MOTION_FEATURE_MASK_EULER          |     \
                                                   DRV_MOTION_FEATURE_MASK_ROT_MAT        |     \
                                                   DRV_MOTION_FEATURE_MASK_HEADING        |     \
                                                   DRV_MOTION_FEATURE_MASK_GRAVITY_VECTOR)

typedef enum
{
    DRV_MOTION_EVT_RAW,
    DRV_MOTION_EVT_QUAT,
    DRV_MOTION_EVT_EULER,
    DRV_MOTION_EVT_ROT_MAT,
    DRV_MOTION_EVT_HEADING,
    DRV_MOTION_EVT_GRAVITY,
    DRV_MOTION_EVT_TAP,
    DRV_MOTION_EVT_ORIENTATION,
    DRV_MOTION_EVT_PEDOMETER
}drv_motion_evt_t;

/**
 * @brief Motion driver event handler callback type.
 */
typedef void (*drv_motion_evt_handler_t)(drv_motion_evt_t const * p_evt, void * p_data, uint32_t size);

typedef struct
{
    nrf_drv_twi_t         const * p_twi_instance;
    nrf_drv_twi_config_t  const * p_twi_cfg;
    drv_motion_evt_handler_t      evt_handler;
}drv_motion_init_t;


/**@brief Function for initializing the motion driver.
 *
 * @param[in] p_params      Pointer to the init paramter structure.
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_motion_init(drv_motion_init_t * p_params);

/**@brief Function to enable features in the motion driver.
 *
 * @param[in] feature_mask      Feature mask telling what features to enable.
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_motion_enable(drv_motion_feature_mask_t feature_mask);

/**@brief Function to disable features in the motion driver.
 *
 * @param[in] feature_mask      Feature mask telling what features to disable.
 *
 * @retval NRF_SUCCESS.
 */
uint32_t drv_motion_disable(drv_motion_feature_mask_t feature_mask);

#endif
