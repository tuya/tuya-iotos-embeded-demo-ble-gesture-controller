/**
 * @file tuya_svc_angle_calc.h
 * @author lifan
 * @brief euler angle calculation service header file
 * @version 1.0.0
 * @date 2021-12-22
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#ifndef __TUYA_SVC_ANGLE_CALC_H__
#define __TUYA_SVC_ANGLE_CALC_H__

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

/***********************************************************
***********************variable define**********************
***********************************************************/

/***********************************************************
***********************function define**********************
***********************************************************/
/**
 * @brief get euler angles
 * @param[in] dt: smple time
 * @param[in] type: angle type
 * @param[in] gx: gyro data of X-axis
 * @param[in] gy: gyro data of Y-axis
 * @param[in] gz: gyro data of Z-axis
 * @param[in] ax: accel data of X-axis
 * @param[in] ay: accel data of Y-axis
 * @param[in] az: accel data of Z-axis
 * @param[out] roll: the angle rotated around the X-axis
 * @param[out] pitch: the angle rotated around the Y-axis
 * @param[out] yaw: the angle rotated around the Z-axis
 * @return none
 */
VOID_T tuya_calc_angles(_IN CONST FLOAT_T dt, _IN CONST BOOL_T type,
                        _IN FLOAT_T gx, _IN FLOAT_T gy, _IN FLOAT_T gz,
                        _IN CONST FLOAT_T ax, _IN CONST FLOAT_T ay, _IN CONST FLOAT_T az,
                        _OUT FLOAT_T *roll, _OUT FLOAT_T *pitch, _OUT FLOAT_T *yaw);

/**
 * @brief get euler angles by quaternion
 * @param[in] dt: smple time
 * @param[in] dps: gyro data unit (TRUE - dps, FALSE - rps)
 * @param[in] gx: gyro data of X-axis
 * @param[in] gy: gyro data of Y-axis
 * @param[in] gz: gyro data of Z-axis
 * @param[in] ax: accel data of X-axis
 * @param[in] ay: accel data of Y-axis
 * @param[in] az: accel data of Z-axis
 * @param[out] roll: the angle rotated around the X-axis
 * @param[out] pitch: the angle rotated around the Y-axis
 * @param[out] yaw: the angle rotated around the Z-axis
 * @return none
 */
VOID_T tuya_calc_angles_quat(_IN CONST FLOAT_T dt, _IN BOOL_T dps,
                             _IN FLOAT_T gx, _IN FLOAT_T gy, _IN FLOAT_T gz,
                             _IN FLOAT_T ax, _IN FLOAT_T ay, _IN FLOAT_T az,
                             _OUT FLOAT_T *roll, _OUT FLOAT_T *pitch, _OUT FLOAT_T *yaw);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __TUYA_SVC_ANGLE_CALC_H__ */
