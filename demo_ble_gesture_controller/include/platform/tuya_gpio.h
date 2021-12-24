/**
 * @file tuya_gpio.h
 * @author lifan
 * @brief tuya gpio header file
 * @version 1.0.0
 * @date 2021-11-02
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#ifndef __TUYA_GPIO_H__
#define __TUYA_GPIO_H__

#include "tuya_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************
************************micro define************************
***********************************************************/
typedef BYTE_T GPIO_RET;
#define GPIO_OK                 0x00
#define GPIO_ERR_INVALID_PARM   0x01
#define GPIO_ERR_MALLOC_FAILED  0x02
#define GPIO_ERR_CB_UNDEFINED   0x03

/***********************************************************
***********************typedef define***********************
***********************************************************/
typedef enum {
    TY_GPIO_0 = 0,
    TY_GPIO_1,
    TY_GPIO_2,
    TY_GPIO_3,
    TY_GPIO_4,
    TY_GPIO_5,
    TY_GPIO_6,
    TY_GPIO_7,
    TY_GPIO_8,
    TY_GPIO_9,
    TY_GPIO_10,
    TY_GPIO_11,
    TY_GPIO_12,
    TY_GPIO_13,
    TY_GPIO_14,
    TY_GPIO_15,
    TY_GPIO_16,
    TY_GPIO_17,
    TY_GPIO_18,
    TY_GPIO_19,
    TY_GPIO_20,
    TY_GPIO_21,
    TY_GPIO_22,
    TY_GPIO_23,
    TY_GPIO_24,
    TY_GPIO_25,
    TY_GPIO_26,
    TY_GPIO_27,
    TY_GPIO_28,
    TY_GPIO_29,
    TY_GPIO_30,
    TY_GPIO_31,
    TY_GPIO_MAX
} TY_GPIO_PORT_E;

typedef BYTE_T TY_GPIO_MODE_E;
#define TY_GPIO_PULLUP      0x00
#define TY_GPIO_PULLDOWN    0x01
#define TY_GPIO_FLOATING    0x02

typedef BYTE_T TY_GPIO_IRQ_TYPE_E;
#define TY_GPIO_IRQ_NONE    0x00
#define TY_GPIO_IRQ_RISING  0x01
#define TY_GPIO_IRQ_FALLING 0x02

typedef VOID_T (*TY_GPIO_IRQ_CB)();

/***********************************************************
***********************variable define**********************
***********************************************************/

/***********************************************************
***********************function define**********************
***********************************************************/
/**
 * @brief tuya gpio init
 * @param[in] port: gpio number
 * @param[in] in: TRUE - in, FALSE - out
 * @param[in] active_low: TRUE - active_low, FALSE - active high
 * @return GPIO_RET
 */
GPIO_RET tuya_gpio_init(_IN CONST TY_GPIO_PORT_E port, _IN CONST BOOL_T in, _IN CONST BOOL_T active_low);

/**
 * @brief tuya gpio write
 * @param[in] port: gpio number
 * @param[in] level: output level
 * @return GPIO_RET
 */
GPIO_RET tuya_gpio_write(_IN CONST TY_GPIO_PORT_E port, _IN CONST BOOL_T level);

/**
 * @brief tuya gpio read
 * @param[in] port: gpio number
 * @return TRUE - high level, false - low level
 */
BOOL_T tuya_gpio_read(_IN CONST TY_GPIO_PORT_E port);

/**
 * @brief tuya gpio interrupt init
 * @param[in] port: gpio number
 * @param[in] trig_type: trigger type
 * @param[in] irq_cb: interrupt callback function
 * @return GPIO_RET
 */
GPIO_RET tuya_gpio_irq_init(_IN CONST TY_GPIO_PORT_E port, _IN CONST TY_GPIO_IRQ_TYPE_E trig_type, _IN TY_GPIO_IRQ_CB irq_cb);

/**
 * @brief tuya gpio close
 * @param[in] port: gpio number
 * @return GPIO_RET
 */
GPIO_RET tuya_gpio_close(_IN CONST TY_GPIO_PORT_E port);

/**
 * @brief tuya gpio close all
 * @param[in] none
 * @return GPIO_RET
 */
GPIO_RET tuya_gpio_close_all(VOID_T);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __TUYA_GPIO_H__ */
