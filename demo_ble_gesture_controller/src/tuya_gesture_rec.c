/**
 * @file tuya_gesture_rec.c
 * @author lifan
 * @brief gesture action recognition module source file
 * @version 1.0.0
 * @date 2021-12-23
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#include "tuya_gesture_rec.h"
#include "tuya_ble_log.h"
#include <string.h>

/***********************************************************
************************micro define************************
***********************************************************/
#define GYRO_AXIS_NUM       3
#define ACCEL_AXIS_NUM      3
#define EULER_ANGLE_NUM     3
#define DATA_SMP_NUM        8
#define BUFFER_SIZE         100
#define GES_DATA_VALID_THR  5.0f
#define GES_SHAKE_THR       2000.0f
#define GES_TURN_THR        800.0f
#define GES_SHAKE_LEN_THR   20

/***********************************************************
***********************typedef define***********************
***********************************************************/
typedef BYTE_T GES_TYPE_E;
#define GES_TYPE_NONE       0x00
#define GES_TYPE_SHAKE      0x01
#define GES_TYPE_TURN       0x02

/***********************************************************
***********************variable define**********************
***********************************************************/
STATIC FLOAT_T accel_last[ACCEL_AXIS_NUM];
STATIC FLOAT_T sg_accel_diff_sum[DATA_SMP_NUM];
STATIC FLOAT_T sg_gyro_buf[BUFFER_SIZE][GYRO_AXIS_NUM];
STATIC FLOAT_T sg_accel_buf[BUFFER_SIZE][ACCEL_AXIS_NUM];
STATIC FLOAT_T sg_roll_buf[BUFFER_SIZE];
STATIC FLOAT_T sg_pitch_buf[BUFFER_SIZE];
STATIC FLOAT_T sg_yaw_buf[BUFFER_SIZE];

STATIC BOOL_T sg_ges_valid = FALSE;
STATIC FLOAT_T sg_accel_d_s = 0;
STATIC UCHAR_T sg_data_index = 0;
STATIC BOOL_T sg_x_cw = FALSE;

/***********************************************************
***********************function define**********************
***********************************************************/
/**
 * @brief gesture recognition reset
 * @param[in] none
 * @return none
 */
VOID_T tuya_gesture_rec_reset(VOID_T)
{
    sg_ges_valid = FALSE;
}

/**
 * @brief calculate the sum of the absolute value of the acceleration difference
 * @param[in] accel_cur: current acceleration
 * @return the calculation result
 */
FLOAT_T __calc_accel_diff_abs_sum(FLOAT_T *accel_cur)
{
    UCHAR_T i;
    FLOAT_T diff = 0.0f;
    FLOAT_T diff_sum = 0.0f;
    FLOAT_T ret = 0.0f;

    for (i = 0; i < ACCEL_AXIS_NUM; i++) {
        diff = accel_cur[i] - accel_last[i];
        diff_sum += ((diff > 0) ? diff : (-diff));
        accel_last[i] = accel_cur[i];
    }

    for (i = 0; i < DATA_SMP_NUM-1; i++) {
        sg_accel_diff_sum[i] = sg_accel_diff_sum[i+1];
        ret += sg_accel_diff_sum[i];
    }
    sg_accel_diff_sum[DATA_SMP_NUM-1] = diff_sum;
    ret = (ret + sg_accel_diff_sum[DATA_SMP_NUM-1]) / DATA_SMP_NUM;

    return ret;
}

/**
 * @brief get the sum of the absolute value of the acceleration difference
 * @param[in] none
 * @return sg_accel_d_s
 */
FLOAT_T tuya_get_accel_diff_abs_sum(VOID_T)
{
    return sg_accel_d_s;
}

/**
 * @brief get gesture length
 * @param[in] none
 * @return gesture length
 */
UCHAR_T __get_ges_len(VOID_T)
{
    return sg_data_index;
}

/**
 * @brief calculate the total amount of change in acceleration data
 * @param[in] none
 * @return the calculation result
 */
FLOAT_T __calc_accel_total_change(VOID_T)
{
    UCHAR_T i, j;
    FLOAT_T diff = 0.0f;
    FLOAT_T diff_sum = 0.0f;

    for (i = 1; i < sg_data_index; i++) {
        for (j = 0; j < ACCEL_AXIS_NUM; j++) {
            diff = sg_accel_buf[i][j] - sg_accel_buf[0][j];
            diff_sum += ((diff > 0) ? diff : (-diff));
        }
    }

    return diff_sum;
}

/**
 * @brief get the peak value of the gyroscope's x-axis data
 * @param[in] none
 * @return the calculation result
 */
FLOAT_T __get_gyro_x_max(VOID_T)
{
    UCHAR_T i;
    FLOAT_T max = sg_gyro_buf[0][0];
    FLOAT_T min = sg_gyro_buf[0][0];

    for (i = 1; i < sg_data_index; i++) {
        if (max < sg_gyro_buf[i][0]) {
            max = sg_gyro_buf[i][0];
        }
        if (min > sg_gyro_buf[i][0]) {
            min = sg_gyro_buf[i][0];
        }
    }
    max = (max > 0) ? (max) : (-max);
    min = (min > 0) ? (min) : (-min);

    if (max >= min) {
        sg_x_cw = TRUE;
    } else {
        max = min;
        sg_x_cw = FALSE;
    }

    return max;
}

/**
 * @brief judge the type of gesture
 * @param[in] none
 * @return gesture type
 */
GES_TYPE_E __judge_ges_type(VOID_T)
{
    GES_TYPE_E type = GES_TYPE_NONE;

    UCHAR_T len = __get_ges_len();
    TUYA_APP_LOG_DEBUG("lenth:%d", len);

    FLOAT_T accel_change = __calc_accel_total_change();
    TUYA_APP_LOG_DEBUG("accel_change:%.1f", accel_change);

    if ((accel_change >= GES_SHAKE_THR) &&
        (len >= GES_SHAKE_LEN_THR)) {
        type = GES_TYPE_SHAKE;
    } else {
        if ((len < GES_SHAKE_LEN_THR) &&
            (__get_gyro_x_max() >= GES_TURN_THR)) {
            type = GES_TYPE_TURN;
        }
    }
    return type;
}

/**
 * @brief calculate angle diff abs sum
 * @param[in] angle: angle data
 * @param[in] len: gesture length
 * @return diff_abs_sum
 */
FLOAT_T __calc_angle_diff_abs_sum(FLOAT_T *angle, UCHAR_T len)
{
    UCHAR_T i;
    FLOAT_T diff = 0.0f;
    FLOAT_T diff_abs = 0.0f;
    FLOAT_T diff_abs_sum = 0.0f;

    for (i = 1; i < len; i++) {
        diff = angle[i] - angle[i-1];
        diff_abs = (diff > 0) ? diff : (-diff);
        if (diff_abs < 90) {
            diff_abs_sum += diff_abs;
        }
    }
    return diff_abs_sum;
}

/**
 * @brief get shake direction feature
 * @param[in] angle: angle data
 * @param[in] len: gesture length
 * @return direction feature
 */
FLOAT_T __get_angle_dir_feat(FLOAT_T *angle, UCHAR_T len)
{
    UCHAR_T i;
    FLOAT_T diff = 0.0f;
    FLOAT_T dir_feat = 0.0f;

    for (i = 1; i < len; i++) {
        diff = angle[i] - angle[i-1];
        if ((diff > 300) || (diff <- 300)) {
            dir_feat += ((diff > 0) ? (diff - 360) : (diff + 360));
        } else {
            dir_feat += diff;
        }
    }
    return dir_feat;
}

/**
 * @brief recognize the direction of the shaking gesture
 * @param[in] none
 * @return gesture code
 */
GES_CODE_E __rec_shake_gesture(VOID_T)
{
    GES_CODE_E ret = GES_NONE;
    FLOAT_T pitch_d_a_s = __calc_angle_diff_abs_sum(sg_pitch_buf, sg_data_index);
    FLOAT_T yaw_d_a_s = __calc_angle_diff_abs_sum(sg_yaw_buf, sg_data_index);
    TUYA_APP_LOG_DEBUG("pitch_change:%.1f, yaw_change:%.1f", pitch_d_a_s, yaw_d_a_s);

    if (pitch_d_a_s > yaw_d_a_s) {
        if (__get_angle_dir_feat(sg_pitch_buf, sg_data_index) < 0) {
            ret = GES_SHAKE_UP;
            TUYA_APP_LOG_DEBUG("Gesture: up");
        } else {
            ret = GES_SHAKE_DOWN;
            TUYA_APP_LOG_DEBUG("Gesture: down");
        }
    } else {
        if (__get_angle_dir_feat(sg_yaw_buf, sg_data_index) > 0) {
            ret = GES_SHAKE_LEFT;
            TUYA_APP_LOG_DEBUG("Gesture: left");
        } else {
            ret = GES_SHAKE_RIGHT;
            TUYA_APP_LOG_DEBUG("Gesture: right");
        }
    }

    return ret;
}

/**
 * @brief recognize the direction of the turning gesture
 * @param[in] none
 * @return gesture code
 */
GES_CODE_E __rec_turn_gesture(VOID_T)
{
    GES_CODE_E ret = GES_NONE;

    if (sg_x_cw) {
        ret = GES_TURN_CW;
        TUYA_APP_LOG_DEBUG("Gesture: cw");
    } else {
        ret = GES_TURN_CCW;
        TUYA_APP_LOG_DEBUG("Gesture: ccw");
    }

    return ret;
}

/**
 * @brief recognize gesture
 * @param[in] none
 * @return gesture code
 */
GES_CODE_E __rec_gesture(VOID_T)
{
    GES_CODE_E ret = GES_NONE;
    switch (__judge_ges_type()) {
        case GES_TYPE_SHAKE:
            ret = __rec_shake_gesture();
            break;
        case GES_TYPE_TURN:
            ret = __rec_turn_gesture();
            break;
        default:
            break;
    }
    return ret;
}

/**
 * @brief recognize gesture
 * @param[in] gyro: gyro data
 * @param[in] accel: accel data
 * @param[in] angle: angle data
 * @return gesture code
 */
GES_CODE_E tuya_rec_gesture(FLOAT_T *gyro, FLOAT_T *accel, FLOAT_T *angle)
{
    GES_CODE_E ret = GES_NONE;
    UCHAR_T i;

    sg_accel_d_s = __calc_accel_diff_abs_sum(accel);

    if (!sg_ges_valid) {
        if (sg_accel_d_s >= GES_DATA_VALID_THR) {
            sg_ges_valid = TRUE;
            sg_data_index = 0;
        }
    } else {
        if (sg_accel_d_s < GES_DATA_VALID_THR) {
            sg_ges_valid = FALSE;
            ret = __rec_gesture();
        }
    }

    if (sg_ges_valid) {
        for (i = 0; i < 3; i++) {
            sg_gyro_buf[sg_data_index][i] = gyro[i];
            sg_accel_buf[sg_data_index][i] = accel[i];
        }
        sg_roll_buf[sg_data_index] = angle[0];
        sg_pitch_buf[sg_data_index] = angle[1];
        sg_yaw_buf[sg_data_index] = angle[2];
        sg_data_index++;
        TUYA_APP_LOG_DEBUG("roll:%.1f, pitch:%.1f, yaw:%.1f", angle[0], angle[1], angle[2]);
        if (sg_data_index >= BUFFER_SIZE) {
            sg_data_index = 0;
        }
    }
    return ret;
}
