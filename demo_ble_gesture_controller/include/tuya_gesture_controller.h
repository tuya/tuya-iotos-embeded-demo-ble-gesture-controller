/**
 * @file tuya_gesture_controller.h
 * @author lifan
 * @brief gesture controller management center
 * @version 1.0.0
 * @date 2021-12-22
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#ifndef __TUYA_GESTURE_CONTROLLER_H__
#define __TUYA_GESTURE_CONTROLLER_H__

#include "tuya_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************
************************micro define************************
***********************************************************/
#define GES_DATA_DEBUG_EN   1

/***********************************************************
***********************typedef define***********************
***********************************************************/

/***********************************************************
***********************variable define**********************
***********************************************************/

/***********************************************************
***********************function define**********************
***********************************************************/
#if GES_DATA_DEBUG_EN
/**
 * @brief send data to virtual oscilloscope
 * @param[in] data: data to be sent
 * @return none
 */
VOID_T tuya_send_data_to_vi(_IN SHORT_T data1, _IN SHORT_T data2, _IN SHORT_T data3, _IN SHORT_T data4);
#endif

/**
 * @brief gesture controller init
 * @param[in] none
 * @return none
 */
VOID_T tuya_gesture_controller_init(VOID_T);

/**
 * @brief gesture controller loop
 * @param[in] none
 * @return none
 */
VOID_T tuya_gesture_controller_loop(VOID_T);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __TUYA_GESTURE_CONTROLLER_H__ */
