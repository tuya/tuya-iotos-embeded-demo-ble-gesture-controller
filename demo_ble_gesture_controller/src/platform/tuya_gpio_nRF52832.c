/**
 * @file tuya_gpio_nRF52832.c
 * @author lifan
 * @brief tuya gpio source file for nRF52832
 * @version 1.0.0
 * @date 2021-11-02
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#include "tuya_gpio.h"
#include "tuya_ble_mem.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"

/***********************************************************
************************micro define************************
***********************************************************/

/***********************************************************
***********************typedef define***********************
***********************************************************/
typedef struct ty_gpio_irq_mag_s {
    struct ty_gpio_irq_mag_s *next;
    TY_GPIO_PORT_E port;
    TY_GPIO_IRQ_CB irq_cb;
} TY_GPIO_IRQ_MAG_T;

/***********************************************************
***********************variable define**********************
***********************************************************/
STATIC UINT_T sg_pf_pin_list[] = {
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14,
    15,
    16,
    17,
    18,
    19,
    20,
    21,
    22,
    23,
    24,
    25,
    26,
    27,
    28,
    29,
    30,
    31
};

STATIC TY_GPIO_IRQ_MAG_T *sg_irq_mag_list = NULL;

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
GPIO_RET tuya_gpio_init(_IN CONST TY_GPIO_PORT_E port, _IN CONST BOOL_T in, _IN CONST BOOL_T active_low)
{
    if ((port >= TY_GPIO_MAX) ||
        (-1 == sg_pf_pin_list[port])) {
        return GPIO_ERR_INVALID_PARM;
    }

    if (in) {
        if (active_low) {
            nrf_gpio_cfg_input(sg_pf_pin_list[port], NRF_GPIO_PIN_PULLUP);
        } else {
            nrf_gpio_cfg_input(sg_pf_pin_list[port], NRF_GPIO_PIN_PULLDOWN);
        }
    } else {
        nrf_gpio_cfg_output(sg_pf_pin_list[port]);
        if (active_low) {
            nrf_gpio_pin_write(sg_pf_pin_list[port], TRUE);
        } else {
            nrf_gpio_pin_write(sg_pf_pin_list[port], FALSE);
        }
    }

    return GPIO_OK;
}

/**
 * @brief tuya gpio write
 * @param[in] port: gpio number
 * @param[in] level: output level
 * @return GPIO_RET
 */
GPIO_RET tuya_gpio_write(_IN CONST TY_GPIO_PORT_E port, _IN CONST BOOL_T level)
{
    if ((port >= TY_GPIO_MAX) ||
        (-1 == sg_pf_pin_list[port])) {
        return GPIO_ERR_INVALID_PARM;
    }

    nrf_gpio_pin_write(sg_pf_pin_list[port], level);

    return GPIO_OK;
}

/**
 * @brief tuya gpio read
 * @param[in] port: gpio number
 * @return TRUE - high level, false - low level
 */
BOOL_T tuya_gpio_read(_IN CONST TY_GPIO_PORT_E port)
{
    if ((port >= TY_GPIO_MAX) ||
        (-1 == sg_pf_pin_list[port])) {
        return GPIO_ERR_INVALID_PARM;
    }

    return nrf_gpio_pin_read(sg_pf_pin_list[port]);
}

/**
 * @brief gpio irq handler
 * @param[in] pin: pin number
 * @param[in] action: gpiote event polarity
 * @return none
 */
VOID_T __gpio_irq_handler(nrfx_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    TY_GPIO_IRQ_MAG_T *irq_mag_tmp = sg_irq_mag_list;
    while (irq_mag_tmp) {
        if (pin == sg_pf_pin_list[irq_mag_tmp->port]) {
            irq_mag_tmp->irq_cb();
        }
        irq_mag_tmp = irq_mag_tmp->next;
    }
}

/**
 * @brief tuya gpio interrupt init
 * @param[in] port: gpio number
 * @param[in] trig_type: trigger type
 * @param[in] irq_cb: interrupt callback function
 * @return GPIO_RET
 */
GPIO_RET tuya_gpio_irq_init(_IN CONST TY_GPIO_PORT_E port, _IN CONST TY_GPIO_IRQ_TYPE_E trig_type, _IN TY_GPIO_IRQ_CB irq_cb)
{
    if ((port >= TY_GPIO_MAX) ||
        (-1 == sg_pf_pin_list[port]) ||
        (trig_type > TY_GPIO_IRQ_FALLING) ||
        (irq_cb == NULL)) {
        return GPIO_ERR_INVALID_PARM;
    }

    ret_code_t errCode;
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    TY_GPIO_IRQ_MAG_T *irq_mag_tmp = (TY_GPIO_IRQ_MAG_T *)tuya_ble_malloc(SIZEOF(TY_GPIO_IRQ_MAG_T));
    if (NULL == irq_mag_tmp) {
        return GPIO_ERR_MALLOC_FAILED;
    }

    irq_mag_tmp->port = port;
    irq_mag_tmp->irq_cb = irq_cb;
    if (sg_irq_mag_list) {
        irq_mag_tmp->next = sg_irq_mag_list;
    } else {
        errCode = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(errCode);
    }
    sg_irq_mag_list = irq_mag_tmp;

    switch (trig_type) {
    case TY_GPIO_IRQ_NONE:
        break;
    case TY_GPIO_IRQ_RISING:
        in_config.pull = NRF_GPIO_PIN_PULLDOWN;
	    in_config.sense = NRF_GPIOTE_POLARITY_LOTOHI;
        break;
    case TY_GPIO_IRQ_FALLING:
        in_config.pull = NRF_GPIO_PIN_PULLUP;
	    in_config.sense = NRF_GPIOTE_POLARITY_HITOLO;
        break;
    default:
        break;
    }

    errCode = nrf_drv_gpiote_in_init(sg_pf_pin_list[port], &in_config, __gpio_irq_handler);
    APP_ERROR_CHECK(errCode);
    nrf_drv_gpiote_in_event_enable(sg_pf_pin_list[port], TRUE);

    return GPIO_OK;
}

/**
 * @brief tuya gpio close
 * @param[in] port: gpio number
 * @return GPIO_RET
 */
GPIO_RET tuya_gpio_close(_IN CONST TY_GPIO_PORT_E port)
{
    if ((port >= TY_GPIO_MAX) ||
        (-1 == sg_pf_pin_list[port])) {
        return GPIO_ERR_INVALID_PARM;
    }
    nrf_gpio_cfg_default(sg_pf_pin_list[port]);
    return GPIO_OK;
}

/**
 * @brief tuya gpio close all
 * @param[in] none
 * @return GPIO_RET
 */
GPIO_RET tuya_gpio_close_all(VOID_T)
{
    UINT_T pin;
    for (pin = 0; pin < NUMBER_OF_PINS; pin++) {
        nrf_gpio_cfg_default(pin);
        nrf_drv_gpiote_uninit();
        nrf_drv_gpiote_in_event_disable(pin);
    }
    return GPIO_OK;
}
