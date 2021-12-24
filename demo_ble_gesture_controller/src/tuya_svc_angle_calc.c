/**
 * @file tuya_svc_angle_calc.c
 * @author lifan
 * @brief euler angle calculation service source file
 * @version 1.0.0
 * @date 2021-12-22
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */
#include "tuya_svc_angle_calc.h"
#include "tuya_ble_log.h"
#include <math.h>

/***********************************************************
************************micro define************************
***********************************************************/
#define PI                  3.1416f
#define RAD_TO_DEG          57.3f
#define ERR_COV_Q_ANGLE     0.001f  /* Q - angle */
#define ERR_COV_Q_GYRO	    0.003f  /* Q - gyro_m */
#define ERR_COV_R_ACC_ANG	0.5f    /* R - acc_ang */
#define KF_CH_NUM           2       /* angle channel */
#define KP                  0.8f
#define KI                  0.0003f

/***********************************************************
***********************typedef define***********************
***********************************************************/
typedef BYTE_T KF_CH_E;
#define KF_CH_ROLL          0x00
#define KF_CH_PITCH         0x01

/* covariance matrix */
typedef struct {
    FLOAT_T a;          /* P[0][0] */
    FLOAT_T b;          /* P[0][1] */
    FLOAT_T c;          /* P[1][0] */
    FLOAT_T d;          /* P[1][1] */
} KF_COV_MT_T;

typedef struct {
    FLOAT_T angle;      /* x0 - angle */
    FLOAT_T err_gyro;   /* x1 - angle rate error */
    FLOAT_T k0;         /* K0 - for x0 */
    FLOAT_T k1;         /* K1 - for x1 */
    KF_COV_MT_T err_cov;/* P - estimate error covariance matrix */
} ANGLE_KF_T;

/***********************************************************
***********************variable define**********************
***********************************************************/
STATIC ANGLE_KF_T sg_angle_kf[KF_CH_NUM] = {
    { 0.0f, 0.0f, 0.0f, 0.0f, {1.0f, 0.0f, 0.0f, 1.0f} },
    { 0.0f, 0.0f, 0.0f, 0.0f, {1.0f, 0.0f, 0.0f, 1.0f} }
};

STATIC FLOAT_T sg_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
STATIC FLOAT_T sg_err_int_x = 0.0f, sg_err_int_y = 0.0f, sg_err_int_z = 0.0f;

/***********************************************************
***********************function define**********************
***********************************************************/
/**
 * @brief Kalman filter for attitude calculation
 * @param[in] dt: sample time
 * @param[in] acc_ang_m: angle calculated from acceleration measurement
 * @param[in] gyro_m: angular velocity calculated from gyroscope measurement
 * @param[in] ch: euler angle channel
 * @return none
 */
STATIC VOID_T __angle_calc_kalman_filter(_IN CONST FLOAT_T dt, _IN CONST FLOAT_T acc_ang_m, _IN CONST FLOAT_T gyro_m, _IN CONST KF_CH_E ch)
{
    KF_COV_MT_T mt_tmp;
    FLOAT_T err_acc_ang;

    /* 1. predict state estimate */
    sg_angle_kf[ch].angle += (gyro_m - sg_angle_kf[ch].err_gyro) * dt;

    /* 2. predict state estimate covariance */
    mt_tmp.a = sg_angle_kf[ch].err_cov.a;
    mt_tmp.b = sg_angle_kf[ch].err_cov.b;
    mt_tmp.c = sg_angle_kf[ch].err_cov.c;
    mt_tmp.d = sg_angle_kf[ch].err_cov.d;

    sg_angle_kf[ch].err_cov.a += ERR_COV_Q_ANGLE - (mt_tmp.b + mt_tmp.c) * dt;
    sg_angle_kf[ch].err_cov.b -= mt_tmp.d * dt;
    sg_angle_kf[ch].err_cov.c -= mt_tmp.d * dt;
    sg_angle_kf[ch].err_cov.d += ERR_COV_Q_GYRO;

    /* 3. calculate optimal Kalman gain */
    sg_angle_kf[ch].k0 = sg_angle_kf[ch].err_cov.a / (sg_angle_kf[ch].err_cov.a + ERR_COV_R_ACC_ANG);
    sg_angle_kf[ch].k1 = sg_angle_kf[ch].err_cov.c / (sg_angle_kf[ch].err_cov.a + ERR_COV_R_ACC_ANG);

    /* 4. update state estimate */
    err_acc_ang = acc_ang_m - sg_angle_kf[ch].angle;
    sg_angle_kf[ch].angle += sg_angle_kf[ch].k0 * err_acc_ang;
    sg_angle_kf[ch].err_gyro += sg_angle_kf[ch].k1 * err_acc_ang;

    /* 5. update state estimate covariance */
    mt_tmp.a = sg_angle_kf[ch].err_cov.a;
    mt_tmp.b = sg_angle_kf[ch].err_cov.b;
    sg_angle_kf[ch].err_cov.a -= sg_angle_kf[ch].k0 * mt_tmp.a;
    sg_angle_kf[ch].err_cov.b -= sg_angle_kf[ch].k0 * mt_tmp.b;
    sg_angle_kf[ch].err_cov.c -= sg_angle_kf[ch].k1 * mt_tmp.a;
    sg_angle_kf[ch].err_cov.d -= sg_angle_kf[ch].k1 * mt_tmp.b;
}

/**
 * @brief convert gyro data (intrinsic rotation to extrinsic rotation)
 * @param[inout] gx: gyro data of X-axis
 * @param[inout] gy: gyro data of Y-axis
 * @param[inout] gz: gyro data of Z-axis
 * @param[in] roll: the angle rotated around the X-axis
 * @param[in] pitch: the angle rotated around the Y-axis
 * @param[in] unit: angle unit (TRUE - degree, FALSE - radian)
 * @return none
 */
STATIC VOID_T __conv_gyro_intr_to_extr(_INOUT FLOAT_T *gx, _INOUT FLOAT_T *gy, _INOUT FLOAT_T *gz,
                                       _IN FLOAT_T roll, _IN FLOAT_T pitch, _IN CONST BOOL_T unit)
{
    FLOAT_T omega_x = *gx;
    FLOAT_T omega_y = *gy;
    FLOAT_T omega_z = *gz;

    if (unit) {
        roll /= RAD_TO_DEG;
        pitch /= RAD_TO_DEG;
    }

    *gx = omega_x + sin(roll) * tan(pitch) * omega_y + cos(roll) * tan(pitch) * omega_z;
    *gy = cos(roll) * omega_y - sin(roll) * omega_z;
    *gz = sin(roll) / cos(pitch) * omega_y + cos(roll) / cos(pitch) * omega_z;
}

/**
 * @brief get euler angles
 * @param[in] dt: smple time
 * @param[in] type: angle type
 * @param[in] gx: gyro data of X-axis
 * @param[in] gy: gyro data of Y-axis
 * @param[in] gz: gyro data of Z-axis
 * @param[in] ax: accel data of X-axis
 * @param[in] ay: accel data of Y-axis
 * @param[in] az: accel data of Z-axis
 * @param[out] roll: the angle rotated around the X-axis
 * @param[out] pitch: the angle rotated around the Y-axis
 * @param[out] yaw: the angle rotated around the Z-axis
 * @return none
 */
VOID_T tuya_calc_angles(_IN CONST FLOAT_T dt, _IN CONST BOOL_T type,
                        _IN FLOAT_T gx, _IN FLOAT_T gy, _IN FLOAT_T gz,
                        _IN CONST FLOAT_T ax, _IN CONST FLOAT_T ay, _IN CONST FLOAT_T az,
                        _OUT FLOAT_T *roll, _OUT FLOAT_T *pitch, _OUT FLOAT_T *yaw)
{
    FLOAT_T acc_roll_m, acc_pitch_m, tmp_yaw;

    if (!type) {
        acc_roll_m = atan2(-ay, az) * RAD_TO_DEG;
        acc_pitch_m = atan2(ax, az) * RAD_TO_DEG;
    } else {
        acc_roll_m = atan2(ay, az) * RAD_TO_DEG;
        acc_pitch_m = atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;
        __conv_gyro_intr_to_extr(&gx, &gy, &gz, *roll, *pitch, TRUE);
    }

    __angle_calc_kalman_filter(dt, acc_roll_m, gx, KF_CH_ROLL);
	*roll = sg_angle_kf[KF_CH_ROLL].angle;
    __angle_calc_kalman_filter(dt, acc_pitch_m, gy, KF_CH_PITCH);
	*pitch = sg_angle_kf[KF_CH_PITCH].angle;

    tmp_yaw = *yaw;
    tmp_yaw += (gz * dt);
    if (tmp_yaw > 180) {
        tmp_yaw -= 360;
    } else if (tmp_yaw <= -180) {
        tmp_yaw += 360;
    } else {
        ;
    }
    *yaw = tmp_yaw;
}

/**
 * @brief fast inverse square root
 * @param[in] number: number
 * @return inverse square root
 */
STATIC FLOAT_T __fast_invsqrt(_IN CONST FLOAT_T number)
{
    LONG_T i;
    FLOAT_T x2, y;
    CONST FLOAT_T threehalfs = 1.5f;

    x2 = number * 0.5f;
    y = number;
    i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(FLOAT_T *)&i;
    y = y * (threehalfs - (x2 * y * y) );      /* 1st iteration */
    /* y = y * (threehalfs - (x2 * y * y));*/  /* 2nd iteration, this can be removed */

    return y;
}

/**
 * @brief get euler angles by quaternion
 * @param[in] dt: smple time
 * @param[in] dps: gyro data unit (TRUE - dps, FALSE - rps)
 * @param[in] gx: gyro data of X-axis
 * @param[in] gy: gyro data of Y-axis
 * @param[in] gz: gyro data of Z-axis
 * @param[in] ax: accel data of X-axis
 * @param[in] ay: accel data of Y-axis
 * @param[in] az: accel data of Z-axis
 * @param[out] roll: the angle rotated around the X-axis
 * @param[out] pitch: the angle rotated around the Y-axis
 * @param[out] yaw: the angle rotated around the Z-axis
 * @return none
 */
VOID_T tuya_calc_angles_quat(_IN CONST FLOAT_T dt, _IN BOOL_T dps,
                             _IN FLOAT_T gx, _IN FLOAT_T gy, _IN FLOAT_T gz,
                             _IN FLOAT_T ax, _IN FLOAT_T ay, _IN FLOAT_T az,
                             _OUT FLOAT_T *roll, _OUT FLOAT_T *pitch, _OUT FLOAT_T *yaw)
{
    FLOAT_T norm;
    FLOAT_T ag_x, ag_y, ag_z;
    FLOAT_T err_x, err_y, err_z;
    FLOAT_T quat_tmp[4];

    /* unit conversion */
    if (dps) {
        gx /= RAD_TO_DEG;
        gy /= RAD_TO_DEG;
        gz /= RAD_TO_DEG;
    }

    /* acceleration normalization */
    norm = __fast_invsqrt(ax*ax + ay*ay + az*az);
    ax *= norm;
    ay *= norm;
    az *= norm;

    /* extract the gravity component in the equivalent rotation matrix of the quaternion */
    ag_x = 2 * (sg_quat[1]*sg_quat[3] - sg_quat[0]*sg_quat[2]);
    ag_y = 2 * (sg_quat[0]*sg_quat[1] + sg_quat[2]*sg_quat[3]);
    ag_z = 1 - 2 * (sg_quat[1]*sg_quat[1] + sg_quat[2]*sg_quat[2]);

    /* calculate the vector product to get the attitude error */
    err_x = (ay*ag_z - az*ag_y);
    err_y = (az*ag_x - ax*ag_z);
    err_z = (ax*ag_y - ay*ag_x);

    /* use complementary filter to correct angular velocity */
    sg_err_int_x += (err_x * KI);
    sg_err_int_y += (err_y * KI);
    sg_err_int_z += (err_z * KI);

    gx += (KP * err_x + sg_err_int_x);
    gy += (KP * err_y + sg_err_int_y);
    gz += (KP * err_z + sg_err_int_z);

    /* update the quaternion */
    quat_tmp[0] = (-sg_quat[1]*gx - sg_quat[2]*gy - sg_quat[3]*gz) * (dt/2);
    quat_tmp[1] = ( sg_quat[0]*gx - sg_quat[3]*gy + sg_quat[2]*gz) * (dt/2);
    quat_tmp[2] = ( sg_quat[3]*gx + sg_quat[0]*gy - sg_quat[1]*gz) * (dt/2);
    quat_tmp[3] = (-sg_quat[2]*gx + sg_quat[1]*gy + sg_quat[0]*gz) * (dt/2);

    sg_quat[0] += quat_tmp[0];
    sg_quat[1] += quat_tmp[1];
    sg_quat[2] += quat_tmp[2];
    sg_quat[3] += quat_tmp[3];

    /* quaternion normalization */
    norm = __fast_invsqrt(sg_quat[0]*sg_quat[0] + sg_quat[1]*sg_quat[1] + sg_quat[2]*sg_quat[2] + sg_quat[3]*sg_quat[3]);
    sg_quat[0] *= norm;
    sg_quat[1] *= norm;
    sg_quat[3] *= norm;
    sg_quat[2] *= norm;

    /* calculate the angles */
    *pitch = asin(-2 * sg_quat[1] * sg_quat[3] + 2 * sg_quat[0]* sg_quat[2]) * RAD_TO_DEG;
    *roll = atan2(2 * sg_quat[2] * sg_quat[3] + 2 * sg_quat[0] * sg_quat[1], -2 * sg_quat[1] * sg_quat[1] - 2 * sg_quat[2]* sg_quat[2] + 1) * RAD_TO_DEG;
    *yaw = atan2(2 * (sg_quat[1]*sg_quat[2] + sg_quat[0]*sg_quat[3]), sg_quat[0]*sg_quat[0]+sg_quat[1]*sg_quat[1]-sg_quat[2]*sg_quat[2]-sg_quat[3]*sg_quat[3]) * RAD_TO_DEG;
}
