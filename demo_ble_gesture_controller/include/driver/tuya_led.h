/**
 * @file tuya_led.h
 * @author lifan
 * @brief led driver header file
 * @version 1.0.0
 * @date 2021-09-23
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#ifndef __TUYA_LED_H__
#define __TUYA_LED_H__

#include "tuya_common.h"
#include "tuya_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************
************************micro define************************
***********************************************************/

/***********************************************************
***********************typedef define***********************
***********************************************************/
typedef BYTE_T LED_RET;
#define LED_OK                  0x00
#define LED_ERR_INVALID_PARM    0x01
#define LED_ERR_MALLOC_FAILED   0x02

typedef BYTE_T LED_FLASH_MODE_E;
#define LFM_SPEC_TIME           0x00    /* flash specified time */
#define LFM_SPEC_COUNT          0x01    /* flash specified count */
#define LFM_FOREVER             0x02    /* flash forever */

typedef BYTE_T LED_FLASH_TYPE_E;
#define LFT_STA_ON_END_ON       0x00    /* start at light on and end at light on */
#define LFT_STA_ON_END_OFF      0x01    /* start at light on and end at light off */
#define LFT_STA_OFF_END_ON      0x02    /* start at light off and end at light on */
#define LFT_STA_OFF_END_OFF     0x03    /* start at light off and end at light off */

typedef VOID_T *LED_HANDLE;
typedef VOID_T (*LED_CALLBACK)();

/***********************************************************
***********************variable define**********************
***********************************************************/

/***********************************************************
***********************function define**********************
***********************************************************/
/**
 * @brief tuya create led handle
 * @param[in] pin: led pin
 * @param[in] active_low: TURE - active low, FALSE - active high
 * @param[out] handle: led handle
 * @return LED_RET
 */
LED_RET tuya_create_led_handle(_IN CONST TY_GPIO_PORT_E pin, _IN CONST UCHAR_T active_low, _OUT LED_HANDLE *handle);

/**
 * @brief led reset
 * @param[in] none
 * @return LED_RET
 */
LED_RET tuya_led_reset(VOID_T);

/**
 * @brief set led light on or off
 * @param[in] handle: led handle
 * @param[in] on_off: TURE - light on, FALSE - light off
 * @return LED_RET
 */
LED_RET tuya_set_led_light(_IN CONST LED_HANDLE handle, _IN CONST BOOL_T on_off);

/**
 * @brief set led flash in time type and different flash interval
 * @param[in] handle: led handle
 * @param[in] mode: flash mode
 * @param[in] type: flash type
 * @param[in] on_time: light on time (ms)
 * @param[in] off_time: light off time (ms)
 * @param[in] total: it means flash total time (ms) when led mode is LFM_SPEC_TIME, or means flash total count when led mode is LFM_SPEC_COUNT
 * @param[in] flash_end_cb: flash end callback function
 * @return LED_RET
 */
LED_RET tuya_set_led_flash(_IN CONST LED_HANDLE handle, _IN CONST LED_FLASH_MODE_E mode, _IN CONST LED_FLASH_TYPE_E type, _IN CONST USHORT_T on_time, _IN CONST USHORT_T off_time, _IN CONST UINT_T total, _IN CONST LED_CALLBACK flash_end_cb);

#ifdef __cplusplus
}
#endif

#endif /* __TUYA_LED_H__ */
