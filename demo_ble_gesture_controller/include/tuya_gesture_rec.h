/**
 * @file tuya_gesture_rec.h
 * @author lifan
 * @brief gesture recognition module header file
 * @version 1.0.0
 * @date 2021-12-11
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#ifndef __TUYA_GESTURE_REC_H__
#define __TUYA_GESTURE_REC_H__

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
typedef BYTE_T GES_CODE_E;
#define GES_NONE            0x00
#define GES_SHAKE_UP        0x01
#define GES_SHAKE_DOWN      0x02
#define GES_SHAKE_LEFT      0x03
#define GES_SHAKE_RIGHT     0x04
#define GES_TURN_CW         0x05
#define GES_TURN_CCW        0x06

/***********************************************************
***********************variable define**********************
***********************************************************/

/***********************************************************
***********************function define**********************
***********************************************************/
/**
 * @brief gesture recognition reset
 * @param[in] none
 * @return none
 */
VOID_T tuya_gesture_rec_reset(VOID_T);

/**
 * @brief get the sum of the absolute value of the acceleration difference
 * @param[in] none
 * @return sg_accel_d_s
 */
FLOAT_T tuya_get_accel_diff_abs_sum(VOID_T);

/**
 * @brief recognize gesture
 * @param[in] gyro: gyro data
 * @param[in] accel: accel data
 * @param[in] angle: angle data
 * @return gesture code
 */
GES_CODE_E tuya_rec_gesture(FLOAT_T *gyro, FLOAT_T *accel, FLOAT_T *angle);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __TUYA_GESTURE_REC_H__ */
