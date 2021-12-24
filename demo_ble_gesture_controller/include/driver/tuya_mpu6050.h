/**
 * @file tuya_mpu6050.h
 * @author lifan
 * @brief MPU6050 sensor driver header file
 * @version 1.0.0
 * @date 2021-11-30
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#ifndef __TUYA_MPU6050_H__
#define __TUYA_MPU6050_H__

#include "tuya_common.h"
#include "tuya_gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/***********************************************************
************************micro define************************
***********************************************************/
#define INV_MOTION_DRIVER           0

/***********************************************************
***********************typedef define***********************
***********************************************************/
/* MPU6050 operate result */
typedef BYTE_T MPU_RET;
#define MPU_OK                      0x00
#define MPU_ERR_UNCONN              0x01
#define MPU_ERR_INIT_FAILED         0x02
#define MPU_ERR_SENS_TURN_ON_FAILED 0x03
#define MPU_ERR_CFG_FIFO_FAILED     0x04
#define MPU_ERR_SET_SMPRT_FAILED    0x05
#define MPU_ERR_SET_GFSR_FAILED     0x06
#define MPU_ERR_SET_AFSR_FAILED     0x07
#define MPU_ERR_LOAD_IMG_FAILED     0x08
#define MPU_ERR_SET_ORIEN_FAILED    0x09
#define MPU_ERR_EN_FEAT_FAILED      0x0A
#define MPU_ERR_SET_FIFO_RT_FAILED  0x0B
#define MPU_ERR_SELF_TEST_FAILED    0x0C
#define MPU_ERR_EN_DMP_FAILED       0x0D
#define MPU_ERR_IRQ_INIT_FAILED     0x0E

/* MPU6050 Gyro data type */
typedef BYTE_T MPU_GYRO_DT_E;
#define MPU_GDT_RAW                 0x00    /* raw data */
#define MPU_GDT_DPS                 0x01    /* unit: dps */
#define MPU_GDT_RPS                 0x02    /* unit: rps */
/* MPU6050 Accel data type */
typedef BYTE_T MPU_ACCEL_DT_E;
#define MPU_ADT_RAW                 0x00    /* raw data */
#define MPU_ADT_G                   0x01    /* unit: g */
#define MPU_ADT_MPS2                0x02    /* unit: m/s^2 */

/* MPU6050 Gyro full-scale range */
typedef BYTE_T MPU_GYRO_FSR_E;
#define MPU_GYRO_FS_250             0x00    /* 250dps */
#define MPU_GYRO_FS_500             0x01    /* 500dps */
#define MPU_GYRO_FS_1000            0x02    /* 1000dps */
#define MPU_GYRO_FS_2000            0x03    /* 2000dps */
/* MPU6050 Accel full-scale range */
typedef BYTE_T MPU_ACCEL_FSR_E;
#define MPU_ACCEL_FS_2              0x00    /* 2g */
#define MPU_ACCEL_FS_4              0x01    /* 4g */
#define MPU_ACCEL_FS_8              0x02    /* 8g */
#define MPU_ACCEL_FS_16             0x03    /* 16g */

/* MPU6050 sensors select */
typedef BYTE_T MPU_WORK_SENSOR_E;
#define MPU_GYRO_WORK               0x00
#define MPU_ACCEL_WORK              0x01
#define MPU_BOTH_WORK               0x02

/* MPU6050 clock source */
typedef BYTE_T MPU_CLK_E;
#define MPU_CLK_INTERNAL            0x00
#define MPU_CLK_PLL_XGYRO           0x01
#define MPU_CLK_PLL_YGYRO           0x02
#define MPU_CLK_PLL_ZGYRO           0x03
#define MPU_CLK_PLL_EXT32K          0x04
#define MPU_CLK_PLL_EXT19M          0x05
#define MPU_CLK_KEEP_RESET          0x07

/***********************************************************
***********************variable define**********************
***********************************************************/

/***********************************************************
***********************function define**********************
***********************************************************/
/**
 * @brief set MPU6050 power on
 * @param[in] pin: VLOGIC pin number
 * @param[in] active_low: TRUE - active low, FALSE - active high
 * @return none
 */
VOID_T tuya_mpu6050_power_on(_IN CONST TY_GPIO_PORT_E pin, _IN CONST BOOL_T active_low);

/**
 * @brief set MPU6050 power off
 * @param[in] pin: VLOGIC pin number
 * @param[in] active_low: TRUE - active low, FALSE - active high
 * @return none
 */
VOID_T tuya_mpu6050_power_off(_IN CONST TY_GPIO_PORT_E pin, _IN CONST BOOL_T active_low);

/**
 * @brief enable or disable sleep mode
 * @param[in] enabled: TRUE - sleep, FALSE - work
 * @return none
 */
VOID_T tuya_mpu6050_set_sleep_mode(_IN CONST BOOL_T enabled);

/**
 * @brief MPU6050 sensor driver init
 * @param[in] clk: clock source
 * @param[in] g_fsr: gyroscope's full-scale range
 * @param[in] a_fsr: accelerometer's full-scale range
 * @param[in] smp_rt: sample rate
 * @param[in] pin: interrupt pin
 * @param[in] type: interrupt type
 * @param[in] int_cb: interrupt callback function
 * @return operation result
 */
MPU_RET tuya_mpu6050_init(_IN CONST MPU_CLK_E clk, _IN CONST MPU_GYRO_FSR_E g_fsr, _IN CONST MPU_ACCEL_FSR_E a_fsr,
                          _IN CONST USHORT_T smp_rt, _IN CONST TY_GPIO_PORT_E pin, _IN CONST TY_GPIO_IRQ_TYPE_E type,
                          _IN TY_GPIO_IRQ_CB int_cb);

/**
 * @brief read accelerometer data from MPU6050 (raw data)
 * @param[out] a_x: accelerometer data of X-axis
 * @param[out] a_y: accelerometer data of Y-axis
 * @param[out] a_z: accelerometer data of Z-axis
 * @return none
 */
VOID_T tuya_mpu6050_read_accel_raw(_OUT SHORT_T *a_x, _OUT SHORT_T *a_y, _OUT SHORT_T *a_z);

/**
 * @brief read accelerometer data from MPU6050 (specified unit)
 * @param[out] a_x: output data of X-axis
 * @param[out] a_y: output data of Y-axis
 * @param[out] a_z: output data of Z-axis
 * @param[in] unit: accelerometer unit
 * @return none
 */
VOID_T tuya_mpu6050_read_accel_spec_unit(_OUT FLOAT_T *a_x, _OUT FLOAT_T *a_y, _OUT FLOAT_T *a_z, _IN CONST MPU_ACCEL_DT_E unit);

/**
 * @brief read gyroscope data from MPU6050 (raw data)
 * @param[out] g_x: gyroscope data of X-axis
 * @param[out] g_y: gyroscope data of Y-axis
 * @param[out] g_z: gyroscope data of Z-axis
 * @return none
 */
VOID_T tuya_mpu6050_read_gyro_raw(_OUT SHORT_T *g_x, _OUT SHORT_T *g_y, _OUT SHORT_T *g_z);

/**
 * @brief read gyroscope data from MPU6050 (specified unit)
 * @param[out] g_x: output data of X-axis
 * @param[out] g_y: output data of Y-axis
 * @param[out] g_z: output data of Z-axis
 * @param[in] unit: gyroscope unit
 * @return none
 */
VOID_T tuya_mpu6050_read_gyro_spec_unit(_OUT FLOAT_T *g_x, _OUT FLOAT_T *g_y, _OUT FLOAT_T *g_z, _IN CONST MPU_GYRO_DT_E unit);

/**
 * @brief read temperature data from MPU6050 (Celsius)
 * @param[in] none
 * @return temperature data in Celsius
 */
FLOAT_T tuya_mpu6050_read_temp(VOID_T);

#if INV_MOTION_DRIVER
/**
 * @brief MPU6050 built-in DMP init
 * @param[in] g_fsr: gyroscope's full-scale range
 * @param[in] a_fsr: accelerometer's full-scale range
 * @param[in] pin: interrupt pin
 * @param[in] type: interrupt type
 * @param[in] int_cb: interrupt callback function
 * @return MPU_RET
 */
MPU_RET tuya_mpu6050_dmp_init(_IN CONST MPU_GYRO_FSR_E g_fsr, _IN CONST MPU_ACCEL_FSR_E a_fsr,
                              _IN CONST TY_GPIO_PORT_E pin, _IN CONST TY_GPIO_IRQ_TYPE_E type, _IN TY_GPIO_IRQ_CB int_cb);

/**
 * @brief read data of MPU6050 built-in DMP (raw data)
 * @param[out] gyro: gyroscope data of 3-axis
 * @param[out] accel: accelerometer data of 3-axis
 * @param[out] roll: the angle rotated around the X-axis
 * @param[out] pitch: the angle rotated around the Y-axis
 * @param[out] yaw: the angle rotated around the Z-axis
 * @return 0 means success
 */
INT_T tuya_mpu6050_read_dmp_raw(_OUT SHORT_T *gyro, _OUT SHORT_T *accel,
                                _OUT FLOAT_T *roll, _OUT FLOAT_T *pitch, _OUT FLOAT_T *yaw);

/**
 * @brief read data of MPU6050 built-in DMP (specified unit)
 * @param[in] g_unit: gyroscope unit
 * @param[out] gyro: gyroscope data of 3-axis
 * @param[in] a_unit: accelerometer unit
 * @param[out] accel: accelerometer data of 3-axis
 * @param[out] roll: the angle rotated around the X-axis
 * @param[out] pitch: the angle rotated around the Y-axis
 * @param[out] yaw: the angle rotated around the Z-axis
 * @return 0 means success
 */
INT_T tuya_mpu6050_read_dmp_spec_unit(_IN CONST MPU_GYRO_DT_E g_unit, _OUT FLOAT_T *gyro,
                                      _IN CONST MPU_ACCEL_DT_E a_unit, _OUT FLOAT_T *accel,
                                      _OUT FLOAT_T *roll, _OUT FLOAT_T *pitch, _OUT FLOAT_T *yaw);

/**
 * @brief read data of MPU6050 (called in inv_mpu.c and inv_mpu_dmp_motion_driver.c)
 * @param[in] dev_addr: device address
 * @param[in] reg_addr: register address
 * @param[in] len: data length
 * @param[out] data: read data
 * @return default: 0
 */
INT_T tuya_mpu6050_i2c_read(_IN CONST UCHAR_T dev_addr, _IN UCHAR_T reg_addr, _IN CONST UCHAR_T len, _OUT UCHAR_T *data);

/**
 * @brief write data of MPU6050 (called in inv_mpu.c and inv_mpu_dmp_motion_driver.c)
 * @param[in] dev_addr: device address
 * @param[in] reg_addr: register address
 * @param[in] len: data length
 * @param[in] data: write data
 * @return default: 0
 */
INT_T tuya_mpu6050_i2c_write(_IN CONST UCHAR_T dev_addr, _IN UCHAR_T reg_addr, _IN CONST UCHAR_T len, _IN UCHAR_T *data);

/**
 * @brief get ms timestamp (called in inv_mpu.c and inv_mpu_dmp_motion_driver.c)
 * @param[out] timestamp: ms timestamp
 * @return none
 */
VOID_T tuya_ble_get_ms(_OUT ULONG_T *timestamp);
#endif /* INV_MOTION_DRIVER */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __TUYA_MPU6050_H__ */
