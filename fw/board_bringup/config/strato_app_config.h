#ifndef STRATO_APP_CONFIG_H
#define STRATO_APP_CONFIG_H

#include "app_timer.h"

//TIMER CONFIGS
#define APP_TIMER_PRESCALER                             0                                 /**< Value of the RTC1 PRESCALER register. 4095 = 125 ms every tick */
#define APP_TIMER_OP_QUEUE_SIZE                         30                                /**< Size of timer operation queues. */

//SCHEDULER CONFIGS
#define SCHED_MAX_EVENT_DATA_SIZE                       sizeof(app_timer_event_t)
#define SCHED_QUEUE_SIZE                                100

//BLE CONFIGS
#define DEVICE_NAME                                     "Strato"
#define IS_SRVC_CHANGED_CHARACT_PRESENT                 0                                 /**< Include the service changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define APP_ADV_INTERVAL                                64                                          /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS                      BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED       /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define CENTRAL_LINK_COUNT                              0                                 /**<number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT                           1                                 /**<number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define MIN_CONN_INTERVAL                               MSEC_TO_UNITS(50, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL                               MSEC_TO_UNITS(90, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY                  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY                   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT                    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

//IGNITION CONFIGS
#define MINIMUM_IGNITION_VOLTAGE                        (4.8f)
#define SUPERCAP_SAMPLE_PERIOD_MS                       (200)

//SENSOR DATA CONFIGS
#define TEMP_SAMPLE_PERIOD_MS                           (50)
#define ALTITUDE_SAMPLE_PERIOD_MS                       (520)
#define ACCEL_SAMPLE_PERIOD_MS                          (50)

//PARACHUTE CONFIGS
#define FAILSAFE_TIMER_DELAY_MS                         (5000)
#define CLOSED_DEGREE                                   (90)
#define OPEN_DEGREE                                     (CLOSED_DEGREE+88)
#endif /*STRATO_APP_CONFIG_H*/
