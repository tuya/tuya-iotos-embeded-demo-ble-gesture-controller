/**
 * @file tuya_pwr_mgmt_nRF52832.c
 * @author lifan
 * @brief tuya power management for nRF52832
 * @version 1.0.0
 * @date 2021-11-05
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#include "tuya_pwr_mgmt.h"
#include "nrf_gpio.h"
#include "nrf_pwr_mgmt.h"
//#include "board.h"
//#include "app_timer.h"
//#include "app_error.h"
//#include "nrf_soc.h"
//#include "nrf_sdm.h"
//#include "bsp_btn_ble.h"

/***********************************************************
************************micro define************************
***********************************************************/
#if 0
/* modify the wake-up button configuration in pca10040.h,
   refer to the following code !!! */
#define BUTTONS_NUMBER 1//4

#define BUTTON_START   5//13
#define BUTTON_1       5//13
#define BUTTON_2       14
#define BUTTON_3       15
#define BUTTON_4       16
#define BUTTON_STOP    16
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BUTTONS_ACTIVE_STATE 0

#define BUTTONS_LIST { BUTTON_1 }//, BUTTON_2, BUTTON_3, BUTTON_4 }

#define BSP_BUTTON_0   BUTTON_1
//#define BSP_BUTTON_1   BUTTON_2
//#define BSP_BUTTON_2   BUTTON_3
//#define BSP_BUTTON_3   BUTTON_4
#endif

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
VOID_T tuya_enter_sleep_mode(UINT_T *wakeup_pin, UCHAR_T cnt)
{
    //ret_code_t err_code;

    /* prepare wakeup buttons */
    //err_code = bsp_btn_ble_sleep_mode_prepare();
    //APP_ERROR_CHECK(err_code);
    for (UCHAR_T i = 0; i < cnt; i++) {
        nrf_gpio_cfg_input(wakeup_pin[i], NRF_GPIO_PIN_PULLUP);
        nrf_gpio_pin_sense_t sense = NRF_GPIO_PIN_SENSE_LOW;
        nrf_gpio_cfg_sense_set(wakeup_pin[i], sense);
    }

    /* go to system-off mode (this function does not return; wakeup causes a reset) */
    //err_code = sd_power_system_off();
    //APP_ERROR_CHECK(err_code);
    //sd_softdevice_disable();
    //NRF_POWER->SYSTEMOFF = TRUE;
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
}
