/**
 * @file tuya_net_proc.h
 * @author lifan
 * @brief network process module header file
 * @version 1.0.0
 * @date 2021-12-11
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#ifndef __TUYA_NET_PROC_H__
#define __TUYA_NET_PROC_H__

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
 * @brief is waiting for binding
 * @param[in] none
 * @return F_WAIT_BINDING
 */
BOOL_T tuya_is_wait_bind(VOID_T);

/**
 * @brief network process init
 * @param[in] none
 * @return none
 */
VOID_T tuya_net_proc_init(VOID_T);

/**
 * @brief network process before sleep
 * @param[in] none
 * @return none
 */
VOID_T tuya_net_proc_before_sleep(VOID_T);

/**
 * @brief report gesture result
 * @param[in] gesture: gesture code
 * @return none
 */
VOID_T tuya_report_gesture(UCHAR_T gesture);

/**
 * @brief report all dp data
 * @param[in] none
 * @return none
 */
VOID_T tuya_report_all_dp_data(VOID_T);

/**
 * @brief received DP data process
 * @param[in] dp_data: dp data array
 * @return none
 */
VOID_T tuya_net_proc_dp_recv(_IN UCHAR_T *dp_data);

/**
 * @brief ble connected process
 * @param[in] none
 * @return none
 */
VOID_T tuya_net_proc_ble_conn(VOID_T);

/**
 * @brief ble unbound process
 * @param[in] none
 * @return none
 */
VOID_T tuya_net_proc_ble_unbound(VOID_T);

/**
 * @brief DP query process
 * @param[in] none
 * @return none
 */
VOID_T tuya_net_proc_dp_query(VOID_T);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __TUYA_NET_PROC_H__ */
