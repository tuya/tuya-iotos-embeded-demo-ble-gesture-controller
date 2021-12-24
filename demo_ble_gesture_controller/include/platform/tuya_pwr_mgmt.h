/**
 * @file tuya_pwr_mgmt.h
 * @author lifan
 * @brief tuya power management for nRF52832
 * @version 1.0.0
 * @date 2021-11-05
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#ifndef __TUYA_PWR_MGMT_H__
#define __TUYA_PWR_MGMT_H__

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
 * @brief putting the chip into sleep mode
 * @param[in] wakeup_pin: wake-up pin
 * @param[in] cnt: wake-up pin number
 * @return none
 */
VOID_T tuya_enter_sleep_mode(_IN UINT_T *wakeup_pin, _IN CONST UCHAR_T cnt);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __TUYA_PWR_MGMT_H__ */
