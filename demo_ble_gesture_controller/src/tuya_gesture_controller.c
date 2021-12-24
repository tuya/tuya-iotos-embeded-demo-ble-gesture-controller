/**
 * @file tuya_gesture_controller.c
 * @author lifan
 * @brief gesture controller management center
 * @version 1.0.0
 * @date 2021-12-22
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#include "tuya_gesture_controller.h"
#include "tuya_imu_daq.h"
#include "tuya_svc_angle_calc.h"
#include "tuya_gesture_rec.h"
#include "tuya_net_proc.h"
#include "tuya_pwr_mgmt.h"
#include "tuya_ble_log.h"
#include "tuya_ble_api.h"
#include "tuya_gpio.h"
#include "ty_uart.h"
#include "ty_rtc.h"
#include "ty_ble.h"
#include <string.h>

#if GES_DATA_DEBUG_EN
#include "tuya_ble_utils.h"
#endif

/***********************************************************
************************micro define************************
***********************************************************/
#define ANGLE_CALC_BY_QUAT  0

#define REC_KEY_PIN         TY_GPIO_4
#define DELTA_T             0.005f
#define UNUSED_THR          5
#define UNUSED_TIME         12000    /* 5ms * 12000 = 60s */

/***********************************************************
***********************typedef define***********************
***********************************************************/
#if GES_DATA_DEBUG_EN
typedef BYTE_T DBG_DATA_TYPE_E;
#define DBG_GYRO            0x00
#define DBG_ACCEL           0x01
#define DBG_ANGLE           0x02
#endif

typedef struct {
    FLOAT_T gyro[3];
    FLOAT_T accel[3];
    FLOAT_T angle[3];
} GES_DATA_T;

/***********************************************************
***********************variable define**********************
***********************************************************/
STATIC BOOL_T sg_new_data_ready = CLR;
STATIC GES_DATA_T sg_ges_data = {
    .gyro = {0.0f, 0.0f, 0.0f},
    .accel = {0.0f, 0.0f, 0.0f},
    .angle = {0.0f, 0.0f, 0.0f}
};
STATIC UINT_T sg_wakeup_pin[2] = {TY_GPIO_4, TY_GPIO_5};

/***********************************************************
***********************function define**********************
***********************************************************/
#if GES_DATA_DEBUG_EN
/**
 * @brief send data to virtual oscilloscope
 * @param[in] data: data to be sent
 * @return none
 */
STATIC VOID_T __send_data_to_vi(_IN SHORT_T data1, _IN SHORT_T data2, _IN SHORT_T data3, _IN SHORT_T data4)
{
    USHORT_T crc16 = 0xFFFF;
    UCHAR_T send_buf[10];

    send_buf[0] = (UCHAR_T)((UINT_T)((INT_T)data1) % 256);
    send_buf[1] = (UCHAR_T)((UINT_T)((INT_T)data1) / 256);
    send_buf[2] = (UCHAR_T)((UINT_T)((INT_T)data2) % 256);
    send_buf[3] = (UCHAR_T)((UINT_T)((INT_T)data2) / 256);
    send_buf[4] = (UCHAR_T)((UINT_T)((INT_T)data3) % 256);
    send_buf[5] = (UCHAR_T)((UINT_T)((INT_T)data3) / 256);
    send_buf[6] = (UCHAR_T)((UINT_T)((INT_T)data4) % 256);
    send_buf[7] = (UCHAR_T)((UINT_T)((INT_T)data4) / 256);
    crc16 = tuya_ble_crc16_compute(send_buf, 8, NULL);
    send_buf[8] = (UCHAR_T)(crc16 % 256);
    send_buf[9] = (UCHAR_T)(crc16 / 256);
    ty_uart_send(send_buf, 10);
}

/**
 * @brief send data to virtual oscilloscope
 * @param[in] data: data to be sent
 * @return none
 */
VOID_T tuya_send_data_to_vi(_IN SHORT_T data1, _IN SHORT_T data2, _IN SHORT_T data3, _IN SHORT_T data4)
{
    __send_data_to_vi(data1, data2, data3, data4);
}

/**
 * @brief debug
 * @param[in] type: data type
 * @return none
 */
STATIC VOID_T __debug_ges_data(_IN CONST DBG_DATA_TYPE_E type)
{
    switch (type) {
    case DBG_GYRO:
        __send_data_to_vi((SHORT_T)sg_ges_data.gyro[0], (SHORT_T)sg_ges_data.gyro[1], (SHORT_T)sg_ges_data.gyro[2], (SHORT_T)tuya_get_accel_diff_abs_sum());
        break;
    case DBG_ACCEL:
        __send_data_to_vi((SHORT_T)sg_ges_data.accel[0], (SHORT_T)sg_ges_data.accel[1], (SHORT_T)sg_ges_data.accel[2], (SHORT_T)tuya_get_accel_diff_abs_sum());
        break;
    case DBG_ANGLE:
        __send_data_to_vi((SHORT_T)sg_ges_data.angle[0], (SHORT_T)sg_ges_data.angle[1], (SHORT_T)sg_ges_data.angle[2], 0);
        break;
    default:
        break;
    }
}
#endif

/**
 * @brief close all peripheral
 * @param[in] none
 * @return none
 */
STATIC VOID_T __close_peripheral(VOID_T)
{
    tuya_gpio_close_all();
    ty_rtc_uninit();
    ty_uart_uninit();
}

/**
 * @brief device sleep
 * @param[in] none
 * @return none
 */
STATIC VOID_T __gesture_controller_sleep(VOID_T)
{
    if (!tuya_ble_sleep_allowed_check()) {
        return;
    }
    tuya_close_imu_daq();
    tuya_net_proc_before_sleep();
    __close_peripheral();
    tuya_enter_sleep_mode(sg_wakeup_pin, 2);
}

/**
 * @brief DAQ end callback
 * @param[in] none
 * @return none
 */
STATIC VOID_T __gesture_daq_end_cb(VOID_T)
{
    sg_new_data_ready = SET;
}

/**
 * @brief gesture controller init
 * @param[in] none
 * @return none
 */
VOID_T tuya_gesture_controller_init(VOID_T)
{
    tuya_gpio_init(REC_KEY_PIN, TRUE, TRUE);
    tuya_net_proc_init();
    tuya_imu_daq_init(__gesture_daq_end_cb);
}

/**
 * @brief is recognition function open
 * @param[in] none
 * @return TRUE - open, FALSE - close
 */
STATIC BOOL_T __is_rec_func_open(VOID_T)
{
    STATIC BOOL_T ret = FALSE;
    if (tuya_gpio_read(REC_KEY_PIN)) {
        ret = FALSE;
    } else {
        if (FALSE == ret) {
            tuya_gesture_rec_reset();
        }
        ret = TRUE;
    }
    return ret;
}

/**
 * @brief is device unused
 * @param[in] none
 * @return TRUE - unused, FALSE - using
 */
STATIC BOOL_T __is_device_unused(VOID_T)
{
    STATIC USHORT_T s_tm = 0;
    if (tuya_is_wait_bind()) {
        return FALSE;
    }
    if ((sg_ges_data.gyro[0] < UNUSED_THR) &&
        (sg_ges_data.gyro[1] < UNUSED_THR) &&
        (sg_ges_data.gyro[2] < UNUSED_THR)) {
        s_tm++;
        if (s_tm >= UNUSED_TIME) {
            return TRUE;
        }
    }
    return FALSE;
}

/**
 * @brief gesture controller loop
 * @param[in] none
 * @return none
 */
VOID_T tuya_gesture_controller_loop(VOID_T)
{
    GES_CODE_E gesture = GES_NONE;

    if (sg_new_data_ready) {
        sg_new_data_ready = CLR;
        if (!tuya_get_imu_data(sg_ges_data.gyro, sg_ges_data.accel, sg_ges_data.angle)) {
            return;
        }
        if (__is_device_unused()) {
            __gesture_controller_sleep();
        }
#if (INV_MOTION_DRIVER == 0)
#if (ANGLE_CALC_BY_QUAT == 0)
        tuya_calc_angles(DELTA_T, TRUE,
                         sg_ges_data.gyro[0], sg_ges_data.gyro[1], sg_ges_data.gyro[2],
                         sg_ges_data.accel[0], sg_ges_data.accel[1], sg_ges_data.accel[2],
                         &sg_ges_data.angle[0], &sg_ges_data.angle[1], &sg_ges_data.angle[2]);
#else
        tuya_calc_angles_quat(DELTA_T, TRUE,
                              sg_ges_data.gyro[0], sg_ges_data.gyro[1], sg_ges_data.gyro[2],
                              sg_ges_data.accel[0], sg_ges_data.accel[1], sg_ges_data.accel[2],
                              &sg_ges_data.angle[0], &sg_ges_data.angle[1], &sg_ges_data.angle[2]);
#endif
#endif
        if (__is_rec_func_open()) {
            gesture = tuya_rec_gesture(sg_ges_data.gyro, sg_ges_data.accel, sg_ges_data.angle);
            if (GES_NONE != gesture) {
                tuya_report_gesture(gesture);
            }
        }
#if GES_DATA_DEBUG_EN
        __debug_ges_data(DBG_ANGLE);
#endif
    }
}
