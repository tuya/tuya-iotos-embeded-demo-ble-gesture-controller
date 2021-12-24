/**
 * @file tuya_imu_daq.h
 * @author lifan
 * @brief IMU DAQ (Data Acquisition) module header file
 * @version 1.0.0
 * @date 2021-12-11
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#ifndef __TUYA_IMU_DAQ_H__
#define __TUYA_IMU_DAQ_H__

#include "tuya_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************
************************micro define************************
***********************************************************/

/***********************************************************
***********************typedef define***********************
***********************************************************/
typedef VOID_T (*DAQ_END_CB)();

/***********************************************************
***********************variable define**********************
***********************************************************/

/***********************************************************
***********************function define**********************
***********************************************************/
/**
 * @brief IMU DAQ module init
 * @param[in] daq_end_cb: data acquisition end callback
 * @return none
 */
VOID_T tuya_imu_daq_init(DAQ_END_CB daq_end_cb);

/**
 * @brief close IMU DAQ
 * @param[in] none
 * @return none
 */
VOID_T tuya_close_imu_daq(VOID_T);

/**
 * @brief get IMU data
 * @param[in] gyro: gyro data
 * @param[in] accel: accel data
 * @param[in] imu_angle: angle data from IMU
 * @return TRUE - success, FALSE - fail
 */
BOOL_T tuya_get_imu_data(FLOAT_T *gyro, FLOAT_T *accel, FLOAT_T *imu_angle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __TUYA_IMU_DAQ_H__ */
