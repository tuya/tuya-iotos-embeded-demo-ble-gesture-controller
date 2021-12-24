/**
 * @file tuya_net_proc.c
 * @author lifan
 * @brief network process module source file
 * @version 1.0.0
 * @date 2021-12-11
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#include "tuya_net_proc.h"
#include "tuya_ble_log.h"
#include "tuya_ble_port.h"
#include "tuya_ble_api.h"
#include "tuya_ble_mutli_tsf_protocol.h"
#include "tuya_key.h"
#include "tuya_led.h"
#include "ty_ble.h"
#include "tuya_app_adv.h"

/***********************************************************
************************micro define************************
***********************************************************/
/* DP ID */
#define DP_ID_GESTURE               101
/* DP data index */
#define DP_DATA_INDEX_OFFSET_ID     0
#define DP_DATA_INDEX_OFFSET_TYPE   1
#define DP_DATA_INDEX_OFFSET_LEN_H  2
#define DP_DATA_INDEX_OFFSET_LEN_L  3
#define DP_DATA_INDEX_OFFSET_DATA   4
/* network peripherals */
#define NET_KEY_PIN                 TY_GPIO_5
#define NET_LED_PIN                 TY_GPIO_12
#define NET_LED_FLASH_INTV_MS       300

#define WAIT_BIND_TIME_MS           (1*60*1000) /* 1min */

#define F_BLE_BOUND                 sg_net_proc_flag.bit0
#define F_WAIT_BINDING              sg_net_proc_flag.bit1

#if TUYA_BLE_BEACON_KEY_ENABLE
#define UPDATE_TIME_MS              1000        /* 1s */
#endif

/***********************************************************
***********************typedef define***********************
***********************************************************/
typedef BYTE_T NET_LED_STAT;
#define NET_LED_OFF                 0x00
#define NET_LED_ON                  0x01
#define NET_LED_FLASH_QUICK         0x02

/***********************************************************
***********************variable define**********************
***********************************************************/
STATIC FLAG_BIT sg_net_proc_flag;
STATIC LED_HANDLE sg_net_led_handle = NULL;
STATIC NET_LED_STAT sg_net_led_status = NET_LED_OFF;
STATIC NET_LED_STAT sg_net_led_status_last = NET_LED_OFF;
STATIC KEY_DEF_T sg_key_def_s;
STATIC tuya_ble_timer_t wait_bind_timer;
STATIC tuya_ble_timer_t beacon_data_update_timer;
STATIC UCHAR_T sg_repo_array[255+3];
STATIC UCHAR_T sg_dp_gesture = 0;
extern UINT_T g_sn;

/***********************************************************
***********************function define**********************
***********************************************************/
STATIC VOID_T __net_key_cb(KEY_PRESS_TYPE_E type);
STATIC VOID_T __wait_bind_timer_cb(VOID_T);
#if TUYA_BLE_BEACON_KEY_ENABLE
STATIC VOID_T __beacon_timer_cb(VOID_T);
#endif

/**
 * @brief network key init
 * @param[in] none
 * @return none
 */
VOID_T __net_key_init(VOID_T)
{
    KEY_RET ret;

    sg_key_def_s.port = NET_KEY_PIN;
    sg_key_def_s.active_low = TRUE;
    sg_key_def_s.long_press_time1 = 3000;
    sg_key_def_s.long_press_time2 = 0;
    sg_key_def_s.key_cb = __net_key_cb;

    ret = tuya_reg_key(&sg_key_def_s);
    if (KEY_OK != ret) {
        TUYA_APP_LOG_ERROR("Network key init error: %d.", ret);
    }
}

/**
 * @brief network led init
 * @param[in] none
 * @return none
 */
VOID_T __net_led_init(VOID_T)
{
    LED_RET ret = tuya_create_led_handle(NET_LED_PIN, TRUE, &sg_net_led_handle);
    if (LED_OK != ret) {
        TUYA_APP_LOG_ERROR("Network led init err:%d.", ret);
    }
}

/**
 * @brief set network led status
 * @param[in] status: network led status
 * @return none
 */
STATIC VOID_T __set_net_led_status(NET_LED_STAT status)
{
    if (NET_LED_FLASH_QUICK == status) {
        sg_net_led_status_last = sg_net_led_status;
    }
    sg_net_led_status = status;

    switch (status) {
    case NET_LED_OFF:
        TUYA_APP_LOG_DEBUG("Net led is light off.");
        tuya_set_led_light(sg_net_led_handle, FALSE);
        break;
    case NET_LED_ON:
        TUYA_APP_LOG_DEBUG("Net led is light on.");
        tuya_set_led_light(sg_net_led_handle, TRUE);
        break;
    case NET_LED_FLASH_QUICK:
        TUYA_APP_LOG_DEBUG("Net led is flashing.");
        tuya_set_led_flash(sg_net_led_handle, LFM_FOREVER, LFT_STA_ON_END_ON, NET_LED_FLASH_INTV_MS, NET_LED_FLASH_INTV_MS, 0, NULL);
        break;
    default:
        break;
    }
}

/**
 * @brief is waiting for binding
 * @param[in] none
 * @return F_WAIT_BINDING
 */
BOOL_T tuya_is_wait_bind(VOID_T)
{
    return F_WAIT_BINDING;
}

/**
 * @brief network process init
 * @param[in] none
 * @return none
 */
VOID_T tuya_net_proc_init(VOID_T)
{
    tuya_ble_connect_status_t ble_conn_sta;
    ble_conn_sta = tuya_ble_connect_status_get();
    TUYA_APP_LOG_DEBUG("BLE connect status: %d.", ble_conn_sta);

    __net_key_init();
    __net_led_init();

    tuya_ble_timer_create(&wait_bind_timer, WAIT_BIND_TIME_MS, TUYA_BLE_TIMER_SINGLE_SHOT, __wait_bind_timer_cb);
#if TUYA_BLE_BEACON_KEY_ENABLE
    tuya_ble_timer_create(&beacon_data_update_timer, UPDATE_TIME_MS, TUYA_BLE_TIMER_SINGLE_SHOT, __beacon_timer_cb);
#endif

    if ((ble_conn_sta == BONDING_UNCONN) ||
        (ble_conn_sta == BONDING_CONN)   ||
        (ble_conn_sta == BONDING_UNAUTH_CONN)) {
        F_BLE_BOUND = SET;
        F_WAIT_BINDING = CLR;
        __set_net_led_status(NET_LED_OFF);
    } else {
        F_BLE_BOUND = CLR;
        F_WAIT_BINDING = SET;
        __set_net_led_status(NET_LED_FLASH_QUICK);
        tuya_ble_timer_start(wait_bind_timer);
    }
}

/**
 * @brief network process before sleep
 * @param[in] none
 * @return none
 */
VOID_T tuya_net_proc_before_sleep(VOID_T)
{
    ty_ble_stop_adv();
    tuya_ble_timer_stop(wait_bind_timer);

    tuya_ble_connect_status_t ble_conn_sta = tuya_ble_connect_status_get();
    if ((ble_conn_sta == UNBONDING_UNCONN) ||
        (ble_conn_sta == BONDING_UNCONN)) {
        return;
    }
    tuya_ble_gap_disconnect();
    while (1) {
        if ((ble_conn_sta == UNBONDING_UNCONN) ||
            (ble_conn_sta == BONDING_UNCONN)) {
            break;
        }
    }
}

/**
 * @brief allow users to bind
 * @param[in] none
 * @return none
 */
VOID_T __allow_binding(VOID_T)
{
    if (F_WAIT_BINDING) {
        return;
    }
    if (F_BLE_BOUND) {
        tuya_ble_device_unbind();
    }
    F_WAIT_BINDING = SET;
    ty_ble_start_adv();
    __set_net_led_status(NET_LED_FLASH_QUICK);
    tuya_ble_timer_start(wait_bind_timer);
}

/**
 * @brief prohibit users to bind
 * @param[in] none
 * @return none
 */
VOID_T __prohibit_binding(VOID_T)
{
    F_WAIT_BINDING = CLR;
    ty_ble_stop_adv();
    __set_net_led_status(sg_net_led_status_last);
}

/**
 * @brief report one dp data
 * @param[in] dp_id: DP ID
 * @param[in] dp_type: DP type
 * @param[in] dp_len: DP length
 * @param[in] dp_data: DP data address
 * @return none
 */
STATIC VOID_T __report_one_dp_data(_IN CONST UCHAR_T dp_id, _IN CONST UCHAR_T dp_type, _IN CONST USHORT_T dp_len, _IN CONST UCHAR_T *dp_data)
{
    USHORT_T i;
    sg_repo_array[DP_DATA_INDEX_OFFSET_ID] = dp_id;
    sg_repo_array[DP_DATA_INDEX_OFFSET_TYPE] = dp_type;
    sg_repo_array[DP_DATA_INDEX_OFFSET_LEN_H] = (UCHAR_T)(dp_len >> 8);
    sg_repo_array[DP_DATA_INDEX_OFFSET_LEN_L] = (UCHAR_T)dp_len;
    for (i = 0; i < dp_len; i++) {
        sg_repo_array[DP_DATA_INDEX_OFFSET_DATA + i] = *(dp_data + (dp_len-i-1));
    }
    tuya_ble_dp_data_send(g_sn++, DP_SEND_TYPE_ACTIVE, DP_SEND_FOR_CLOUD_PANEL, DP_SEND_WITHOUT_RESPONSE, sg_repo_array, dp_len + DP_DATA_INDEX_OFFSET_DATA);
}

/**
 * @brief add one dp data
 * @param[in] dp_id: DP ID
 * @param[in] dp_type: DP type
 * @param[in] dp_len: DP length
 * @param[in] dp_data: DP data address
 * @param[in] addr: DP report address
 * @return total length
 */
STATIC UCHAR_T __add_one_dp_data(_IN CONST UCHAR_T dp_id, _IN CONST UCHAR_T dp_type, _IN CONST USHORT_T dp_len, _IN CONST UCHAR_T *dp_data, _IN UCHAR_T *addr)
{
    USHORT_T i;
    *(addr + DP_DATA_INDEX_OFFSET_ID) = dp_id;
    *(addr + DP_DATA_INDEX_OFFSET_TYPE) = dp_type;
    *(addr + DP_DATA_INDEX_OFFSET_LEN_H) = (UCHAR_T)(dp_len >> 8);
    *(addr + DP_DATA_INDEX_OFFSET_LEN_L) = (UCHAR_T)dp_len;
    for (i = 0; i < dp_len; i++) {
        *(addr + DP_DATA_INDEX_OFFSET_DATA + i) = *(dp_data + (dp_len-i-1));
    }
    return (dp_len + DP_DATA_INDEX_OFFSET_DATA);
}

/**
 * @brief report all dp data
 * @param[in] none
 * @return none
 */
STATIC VOID_T __report_all_dp_data(VOID_T)
{
#if (TUYA_BLE_BEACON_KEY_ENABLE == 0)
    UINT_T total_len = 0;
    total_len += __add_one_dp_data(DP_ID_GESTURE, DT_ENUM, 1, &sg_dp_gesture, sg_repo_array);
    tuya_ble_dp_data_send(g_sn++, DP_SEND_TYPE_ACTIVE, DP_SEND_FOR_CLOUD_PANEL, DP_SEND_WITHOUT_RESPONSE, sg_repo_array, total_len);
#else
    tuya_beacon_data_update(g_sn++, DP_ID_GESTURE, ((DT_ENUM << 4) | 0x01), (UINT_T)sg_dp_gesture);
    tuya_ble_timer_start(beacon_data_update_timer);
#endif
}

/**
 * @brief report gesture result
 * @param[in] gesture: gesture code
 * @return none
 */
VOID_T tuya_report_gesture(UCHAR_T gesture)
{
    sg_dp_gesture = gesture;
#if (TUYA_BLE_BEACON_KEY_ENABLE == 0)
    __report_one_dp_data(DP_ID_GESTURE, DT_ENUM, 1, &sg_dp_gesture);
#else
    tuya_beacon_data_update(g_sn++, DP_ID_GESTURE, ((DT_ENUM << 4) | 0x01), (UINT_T)sg_dp_gesture);
    tuya_ble_timer_start(beacon_data_update_timer);
#endif
}

/**
 * @brief report all dp data
 * @param[in] none
 * @return none
 */
VOID_T tuya_report_all_dp_data(VOID_T)
{
    __report_all_dp_data();
}

/**
 * @brief received DP data process
 * @param[in] dp_data: dp data array
 * @return none
 */
VOID_T tuya_net_proc_dp_recv(_IN UCHAR_T *dp_data)
{
    switch (dp_data[0]) {
    default:
        break;
    }
}

/**
 * @brief ble connected process
 * @param[in] none
 * @return none
 */
VOID_T tuya_net_proc_ble_conn(VOID_T)
{
    if (F_WAIT_BINDING == SET) {
        F_BLE_BOUND = SET;
        F_WAIT_BINDING = CLR;
        tuya_ble_timer_stop(wait_bind_timer);
        __set_net_led_status(sg_net_led_status_last);
    }
}

/**
 * @brief ble unbound process
 * @param[in] none
 * @return none
 */
VOID_T tuya_net_proc_ble_unbound(VOID_T)
{
    F_BLE_BOUND = CLR;
    ty_ble_stop_adv();
}

/**
 * @brief DP query process
 * @param[in] none
 * @return none
 */
VOID_T tuya_net_proc_dp_query(VOID_T)
{
    /* report initial value */
    __report_all_dp_data();
}

/**
 * @brief set net led turn on or off
 * @param[in] none
 * @return none
 */
VOID_T __set_net_led_power(VOID_T)
{
    if (NET_LED_FLASH_QUICK == sg_net_led_status) {
        return;
    }
    if (NET_LED_OFF == sg_net_led_status) {
        __set_net_led_status(NET_LED_ON);
    } else {
        __set_net_led_status(NET_LED_OFF);
    }
}

/**
 * @brief network key callback
 * @param[in] type: key event type
 * @return none
 */
STATIC VOID_T __net_key_cb(KEY_PRESS_TYPE_E type)
{
    switch (type) {
    case SHORT_PRESS:
        __set_net_led_power();
        break;
    case LONG_PRESS_FOR_TIME1:
        __allow_binding();
        break;
    case LONG_PRESS_FOR_TIME2:
        break;
    default:
        break;
    }
}

/**
 * @brief wait for binding timer callback
 * @param[in] none
 * @return none
 */
STATIC VOID_T __wait_bind_timer_cb(VOID_T)
{
    __prohibit_binding();
}

#if TUYA_BLE_BEACON_KEY_ENABLE
/**
 * @brief beacon data update
 * @param[in] none
 * @return none
 */
STATIC VOID_T __beacon_timer_cb(VOID_T)
{
    tuya_adv_data_update();
}
#endif
