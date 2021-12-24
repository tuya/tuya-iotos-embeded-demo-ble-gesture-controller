/**
 * @file tuya_mpu6050.c
 * @author lifan
 * @brief MPU6050 sensor driver source file
 * @version 1.0.0
 * @date 2021-11-30
 *
 * @copyright Copyright (c) tuya.inc 2021
 *
 */

#include "tuya_mpu6050.h"
#include "ty_i2c.h"
#include "tuya_ble_log.h"
#include "tuya_ble_port.h"

#if INV_MOTION_DRIVER
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>
#endif

/***********************************************************
************************micro define************************
***********************************************************/
/* device address */
#define MPU6050_DEV_ADDR_AD0_LOW        0x68
#define MPU6050_DEV_ADDR_AD0_HIGH       0x69
#define MPU6050_DEV_ADDR     		    MPU6050_DEV_ADDR_AD0_LOW
#define MPU6050_DEV_ID                  0x68

/* I2C R/W command */
#define I2C_CMD_WRITE                   0
#define I2C_CMD_READ                    1
#define MPU6050_ADDR_CMD_WRITE          ((MPU6050_DEV_ADDR << 1) | I2C_CMD_WRITE)
#define MPU6050_ADDR_CMD_READ           ((MPU6050_DEV_ADDR << 1) | I2C_CMD_READ)

/* register map */
#define MPU6050_RA_XG_OFFS_TC           0x00
#define MPU6050_RA_YG_OFFS_TC           0x01
#define MPU6050_RA_ZG_OFFS_TC           0x02
#define MPU6050_RA_X_FINE_GAIN          0x03
#define MPU6050_RA_Y_FINE_GAIN          0x04
#define MPU6050_RA_Z_FINE_GAIN          0x05
#define MPU6050_RA_XA_OFFS_H            0x06
#define MPU6050_RA_XA_OFFS_L_TC         0x07
#define MPU6050_RA_YA_OFFS_H            0x08
#define MPU6050_RA_YA_OFFS_L_TC         0x09
#define MPU6050_RA_ZA_OFFS_H            0x0A
#define MPU6050_RA_ZA_OFFS_L_TC         0x0B
#define MPU6050_RA_XG_OFFS_USRH         0x13
#define MPU6050_RA_XG_OFFS_USRL         0x14
#define MPU6050_RA_YG_OFFS_USRH         0x15
#define MPU6050_RA_YG_OFFS_USRL         0x16
#define MPU6050_RA_ZG_OFFS_USRH         0x17
#define MPU6050_RA_ZG_OFFS_USRL         0x18
#define MPU6050_RA_SMPRT_DIV            0x19
#define MPU6050_RA_CONFIG               0x1A
#define MPU6050_RA_GYRO_CONFIG          0x1B
#define MPU6050_RA_ACCEL_CONFIG         0x1C
#define MPU6050_RA_FF_THR               0x1D
#define MPU6050_RA_FF_DUR               0x1E
#define MPU6050_RA_MOT_THR              0x1F
#define MPU6050_RA_MOT_DUR              0x20
#define MPU6050_RA_ZRMOT_THR            0x21
#define MPU6050_RA_ZRMOT_DUR            0x22
#define MPU6050_RA_FIFO_EN              0x23
#define MPU6050_RA_I2C_MST_CTRL         0x24
#define MPU6050_RA_I2C_SLV0_ADDR        0x25
#define MPU6050_RA_I2C_SLV0_REG         0x26
#define MPU6050_RA_I2C_SLV0_CTRL        0x27
#define MPU6050_RA_I2C_SLV1_ADDR        0x28
#define MPU6050_RA_I2C_SLV1_REG         0x29
#define MPU6050_RA_I2C_SLV1_CTRL        0x2A
#define MPU6050_RA_I2C_SLV2_ADDR        0x2B
#define MPU6050_RA_I2C_SLV2_REG         0x2C
#define MPU6050_RA_I2C_SLV2_CTRL        0x2D
#define MPU6050_RA_I2C_SLV3_ADDR        0x2E
#define MPU6050_RA_I2C_SLV3_REG         0x2F
#define MPU6050_RA_I2C_SLV3_CTRL        0x30
#define MPU6050_RA_I2C_SLV4_ADDR        0x31
#define MPU6050_RA_I2C_SLV4_REG         0x32
#define MPU6050_RA_I2C_SLV4_DO          0x33
#define MPU6050_RA_I2C_SLV4_CTRL        0x34
#define MPU6050_RA_I2C_SLV4_DI          0x35
#define MPU6050_RA_I2C_MST_STATUS       0x36
#define MPU6050_RA_INT_PIN_CFG          0x37
#define MPU6050_RA_INT_ENABLE           0x38
#define MPU6050_RA_DMP_INT_STATUS       0x39
#define MPU6050_RA_INT_STATUS           0x3A
#define MPU6050_RA_ACCEL_XOUT_H         0x3B
#define MPU6050_RA_ACCEL_XOUT_L         0x3C
#define MPU6050_RA_ACCEL_YOUT_H         0x3D
#define MPU6050_RA_ACCEL_YOUT_L         0x3E
#define MPU6050_RA_ACCEL_ZOUT_H         0x3F
#define MPU6050_RA_ACCEL_ZOUT_L         0x40
#define MPU6050_RA_TEMP_OUT_H           0x41
#define MPU6050_RA_TEMP_OUT_L           0x42
#define MPU6050_RA_GYRO_XOUT_H          0x43
#define MPU6050_RA_GYRO_XOUT_L          0x44
#define MPU6050_RA_GYRO_YOUT_H          0x45
#define MPU6050_RA_GYRO_YOUT_L          0x46
#define MPU6050_RA_GYRO_ZOUT_H          0x47
#define MPU6050_RA_GYRO_ZOUT_L          0x48
#define MPU6050_RA_EXT_SENS_DATA_00     0x49
#define MPU6050_RA_EXT_SENS_DATA_01     0x4A
#define MPU6050_RA_EXT_SENS_DATA_02     0x4B
#define MPU6050_RA_EXT_SENS_DATA_03     0x4C
#define MPU6050_RA_EXT_SENS_DATA_04     0x4D
#define MPU6050_RA_EXT_SENS_DATA_05     0x4E
#define MPU6050_RA_EXT_SENS_DATA_06     0x4F
#define MPU6050_RA_EXT_SENS_DATA_07     0x50
#define MPU6050_RA_EXT_SENS_DATA_08     0x51
#define MPU6050_RA_EXT_SENS_DATA_09     0x52
#define MPU6050_RA_EXT_SENS_DATA_10     0x53
#define MPU6050_RA_EXT_SENS_DATA_11     0x54
#define MPU6050_RA_EXT_SENS_DATA_12     0x55
#define MPU6050_RA_EXT_SENS_DATA_13     0x56
#define MPU6050_RA_EXT_SENS_DATA_14     0x57
#define MPU6050_RA_EXT_SENS_DATA_15     0x58
#define MPU6050_RA_EXT_SENS_DATA_16     0x59
#define MPU6050_RA_EXT_SENS_DATA_17     0x5A
#define MPU6050_RA_EXT_SENS_DATA_18     0x5B
#define MPU6050_RA_EXT_SENS_DATA_19     0x5C
#define MPU6050_RA_EXT_SENS_DATA_20     0x5D
#define MPU6050_RA_EXT_SENS_DATA_21     0x5E
#define MPU6050_RA_EXT_SENS_DATA_22     0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 	0x60
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_I2C_SLV0_DO      	0x63
#define MPU6050_RA_I2C_SLV1_DO      	0x64
#define MPU6050_RA_I2C_SLV2_DO      	0x65
#define MPU6050_RA_I2C_SLV3_DO      	0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL            0x6A
#define MPU6050_RA_PWR_MGMT_1           0x6B
#define MPU6050_RA_PWR_MGMT_2           0x6C
#define MPU6050_RA_BANK_SEL             0x6D
#define MPU6050_RA_MEM_START_ADDR       0x6E
#define MPU6050_RA_MEM_R_W              0x6F
#define MPU6050_RA_DMP_CFG_1            0x70
#define MPU6050_RA_DMP_CFG_2            0x71
#define MPU6050_RA_FIFO_COUNTH          0x72
#define MPU6050_RA_FIFO_COUNTL          0x73
#define MPU6050_RA_FIFO_R_W             0x74
#define MPU6050_RA_WHO_AM_I             0x75

/* register bits */
#define MPU_RA_BIT_DEVICE_RESET         (1<<7)
#define MPU_RA_BIT_SLEEP                (1<<6)
#define MPU_RA_BIT_CLKSEL               ((1<<2) | (1<<1) | (1<<0))
#define MPU_RA_BIT_STBY_XYZA            ((1<<5) | (1<<4) | (1<<3))
#define MPU_RA_BIT_STBY_XYZG            ((1<<2) | (1<<1) | (1<<0))
#define MPU_RA_BIT_FIFO_EN_XYZG         ((1<<6) | (1<<5) | (1<<4))
#define MPU_RA_BIT_FIFO_EN_XYZA         (1<<3)
#define MPU_RA_BIT_XYZG_ST              ((1<<7) | (1<<6) | (1<<5))
#define MPU_RA_BIT_XYZA_ST              ((1<<7) | (1<<6) | (1<<5))
#define MPU_RA_BIT_FS_SEL               ((1<<4) | (1<<3))
#define MPU_RA_BIT_AFS_SEL              ((1<<4) | (1<<3))

/* DLPF */
#define MPU_DLPF_BW_CFG_0               260
#define MPU_DLPF_BW_CFG_1               184
#define MPU_DLPF_BW_CFG_2               94
#define MPU_DLPF_BW_CFG_3               44
#define MPU_DLPF_BW_CFG_4               21
#define MPU_DLPF_BW_CFG_5               10
#define MPU_DLPF_BW_CFG_6               5

#define MPU_GYRO_OUTPUT_RATE            1000
#define MPU_SMPRT_DIV_MAX               255

/* unit conversion parameters */
#define ACCEL_OF_G                      9.8f
#define RPS_TO_DPS                      57.3f

#if INV_MOTION_DRIVER
#define MPU_SMPRT_DEFAULT               200
#define Q30                             1073741824.0f
#endif

/***********************************************************
***********************typedef define***********************
***********************************************************/

/***********************************************************
***********************variable define**********************
***********************************************************/
STATIC BOOL_T sg_pwr_pin_used = FALSE;
STATIC FLOAT_T sg_gyro_sens = 0.0f;
STATIC USHORT_T sg_accel_sens = 0;

#if INV_MOTION_DRIVER
STATIC CHAR_T gyro_orientation[9] = { 0,-1, 0,
                                      1, 0, 0,
                                      0, 0, 1 };
#endif

/***********************************************************
***********************function define**********************
***********************************************************/
/**
 * @brief read data of MPU6050
 * @param[in] reg_addr: register address
 * @param[in] len: data length
 * @param[out] data: data buffer
 * @return none
 */
STATIC VOID_T __mpu6050_read_data(_IN UCHAR_T reg_addr, _IN CONST UCHAR_T len, _OUT UCHAR_T *data)
{
    i2c_start();
    i2c_send_bytes(MPU6050_ADDR_CMD_WRITE, &reg_addr, 1);
    i2c_start();
    i2c_rcv_bytes(MPU6050_ADDR_CMD_READ, data, len);
    i2c_stop();
}

/**
 * @brief read register of MPU6050
 * @param[in] reg_addr: register address
 * @return register value
 */
STATIC UCHAR_T __mpu6050_read_register(_IN UCHAR_T reg_addr)
{
    UCHAR_T reg_val;
    i2c_start();
    i2c_send_bytes(MPU6050_ADDR_CMD_WRITE, &reg_addr, 1);
    i2c_start();
    i2c_rcv_bytes(MPU6050_ADDR_CMD_READ, &reg_val, 1);
    i2c_stop();
    return reg_val;
}

/**
 * @brief write register of MPU6050
 * @param[in] reg_addr: register address
 * @param[in] reg_val: value to be written
 * @return none
 */
STATIC VOID_T __mpu6050_write_register(_IN CONST UCHAR_T reg_addr, _IN UCHAR_T reg_val)
{
    i2c_soft_cfg(MPU6050_ADDR_CMD_WRITE, reg_addr, reg_val);
}

/**
 * @brief write register of MPU6050
 * @param[in] reg_addr: register address
 * @param[in] data: data to be written
 * @param[in] valid_bit: the code of valid bits
 * @return none
 */
STATIC VOID_T __mpu6050_write_register_bit(_IN CONST UCHAR_T reg_addr, _IN CONST UCHAR_T data, _IN CONST UCHAR_T valid_bit)
{
    UCHAR_T reg_val;
    if (valid_bit == 0xFF) {
        reg_val = data;
    } else {
        reg_val = __mpu6050_read_register(reg_addr);
        reg_val = (reg_val & (~valid_bit)) | (data & valid_bit);
    }
    i2c_soft_cfg(MPU6050_ADDR_CMD_WRITE, reg_addr, reg_val);
}

#if 0
/**
 * @brief get valid bits code when the bits is continuous
 * @param[in] high_bit: the highest bit
 * @param[in] num: the number of bits
 * @return valid bits
 */
STATIC UCHAR_T __get_valid_bit(_IN CONST UCHAR_T high_bit, _IN CONST UCHAR_T num)
{
    if ((num < 1) || (num > 8) || (high_bit < (num - 1))) {
        return 0x00;
    }
    return ((0xFF >> (8 - num)) << (high_bit - (num - 1)));
}
#endif

/**
 * @brief set MPU6050 power on
 * @param[in] pin: VLOGIC pin number
 * @param[in] active_low: TRUE - active low, FALSE - active high
 * @return none
 */
VOID_T tuya_mpu6050_power_on(_IN CONST TY_GPIO_PORT_E pin, _IN CONST BOOL_T active_low)
{
    if (!sg_pwr_pin_used) {
        tuya_gpio_init(pin, FALSE, active_low);
        sg_pwr_pin_used = TRUE;
    }
    tuya_gpio_write(pin, !active_low);
    tuya_ble_device_delay_ms(100);
}

/**
 * @brief set MPU6050 power off
 * @param[in] pin: VLOGIC pin number
 * @param[in] active_low: TRUE - active low, FALSE - active high
 * @return none
 */
VOID_T tuya_mpu6050_power_off(_IN CONST TY_GPIO_PORT_E pin, _IN CONST BOOL_T active_low)
{
    if (!sg_pwr_pin_used){
        tuya_gpio_init(pin, FALSE, active_low);
        sg_pwr_pin_used = TRUE;
    }
    tuya_gpio_write(pin, active_low);
}

/**
 * @brief reset MPU6050
 * @param[in] none
 * @return none
 */
STATIC VOID_T __mpu6050_reset(VOID_T)
{
    __mpu6050_write_register_bit(MPU6050_RA_PWR_MGMT_1, MPU_RA_BIT_DEVICE_RESET, MPU_RA_BIT_DEVICE_RESET);
    tuya_ble_device_delay_ms(100);
}

/**
 * @brief get the identity of the device (default: 0x68)
 * @param[in] none
 * @return device id
 */
STATIC UCHAR_T __mpu6050_get_device_id(VOID_T)
{
    return __mpu6050_read_register(MPU6050_RA_WHO_AM_I);
}

/**
 * @brief check if MPU6050 is connected
 * @param[in] none
 * @return TRUE - connected, FALSE - unconnected
 */
STATIC BOOL_T __mpu6050_is_connected(VOID_T)
{
    if (__mpu6050_get_device_id() == MPU6050_DEV_ID) {
        return TRUE;
    } else {
        return FALSE;
    }
}

/**
 * @brief enable or disable sleep mode
 * @param[in] enabled: TRUE - sleep, FALSE - work
 * @return none
 */
STATIC VOID_T __mpu6050_set_sleep_mode(_IN CONST BOOL_T enabled)
{
    if (enabled) {
        __mpu6050_write_register_bit(MPU6050_RA_PWR_MGMT_1, MPU_RA_BIT_SLEEP, MPU_RA_BIT_SLEEP);
    } else {
        __mpu6050_write_register_bit(MPU6050_RA_PWR_MGMT_1, ~MPU_RA_BIT_SLEEP, MPU_RA_BIT_SLEEP);
    }
}

/**
 * @brief enable or disable sleep mode
 * @param[in] enabled: TRUE - sleep, FALSE - work
 * @return none
 */
VOID_T tuya_mpu6050_set_sleep_mode(_IN CONST BOOL_T enabled)
{
    __mpu6050_set_sleep_mode(enabled);
}

/**
 * @brief set clock source
 * @param[in] src: clock source
 * @return none
 */
STATIC VOID_T __mpu6050_set_clk_src(UCHAR_T src)
{
    __mpu6050_write_register_bit(MPU6050_RA_PWR_MGMT_1, src, MPU_RA_BIT_CLKSEL);
}

/**
 * @brief set gyroscope's full-scale range
 * @param[in] range: gyroscope's full-scale range value
 * @return none
 */
STATIC VOID_T __mpu6050_set_gyro_fsr(_IN CONST MPU_GYRO_FSR_E range)
{
    __mpu6050_write_register_bit(MPU6050_RA_GYRO_CONFIG, range<<3, MPU_RA_BIT_FS_SEL);
}

/**
 * @brief set accelerometer's full-scale range
 * @param[in] range: new full-scale accelerometer range value
 * @return none
 */
STATIC VOID_T __mpu6050_set_accel_fsr(_IN CONST MPU_ACCEL_FSR_E range)
{
    __mpu6050_write_register_bit(MPU6050_RA_ACCEL_CONFIG, range<<3, MPU_RA_BIT_AFS_SEL);
}

/**
 * @brief set MPU6050's sample rate
 * @param[in] sr: sample rate gyroscope output rate divider value
 * @return none
 */
STATIC VOID_T __mpu6050_set_sample_rate(_IN USHORT_T sr)
{
    UCHAR_T div;
    if (sr > MPU_GYRO_OUTPUT_RATE) {
        sr = MPU_GYRO_OUTPUT_RATE;
    }
    if (sr < (MPU_GYRO_OUTPUT_RATE/MPU_SMPRT_DIV_MAX)) {
        sr = (MPU_GYRO_OUTPUT_RATE/MPU_SMPRT_DIV_MAX);
    }
    div = MPU_GYRO_OUTPUT_RATE / sr - 1;
    __mpu6050_write_register(MPU6050_RA_SMPRT_DIV, div);
}

/**
 * @brief set MPU6050's DLPF
 * @param[in] bw: baud width
 * @return none
 */
STATIC VOID_T __mpu6050_set_dlpf(_IN CONST USHORT_T bw)
{
    UCHAR_T cfg = 0;
    if (bw >= MPU_DLPF_BW_CFG_1) {
        cfg = 1;
    } else if (bw >= MPU_DLPF_BW_CFG_2) {
        cfg = 2;
    } else if (bw >= MPU_DLPF_BW_CFG_3) {
        cfg = 3;
    } else if (bw >= MPU_DLPF_BW_CFG_4) {
        cfg = 4;
    } else if (bw >= MPU_DLPF_BW_CFG_5) {
        cfg = 5;
    } else {
        cfg = 6;
    }
    __mpu6050_write_register(MPU6050_RA_CONFIG, cfg);
}

#if 0
/**
 * @brief wake up sensors
 * @param[in] sensor: working sensors
 * @return none
 */
STATIC VOID_T __mpu6050_set_sensors(_IN CONST MPU_WORK_SENSOR_E sensor)
{
    UCHAR_T reg_value = 0;
    switch (sensor) {
    case MPU_GYRO_WORK:
        reg_value = ~MPU_RA_BIT_STBY_XYZG;
        break;
    case MPU_ACCEL_WORK:
        reg_value = ~MPU_RA_BIT_STBY_XYZA;
        break;
    case MPU_BOTH_WORK:
        reg_value = ~(MPU_RA_BIT_STBY_XYZG | MPU_RA_BIT_STBY_XYZA);
        break;
    default:
        break;
    }
    __mpu6050_write_register_bit(MPU6050_RA_PWR_MGMT_2, reg_value, MPU_RA_BIT_STBY_XYZG|MPU_RA_BIT_STBY_XYZA);
}

/**
 * @brief set fifo
 * @param[in] sensor: working sensors
 * @return none
 */
STATIC VOID_T __mpu6050_set_fifo(_IN CONST MPU_WORK_SENSOR_E sensor)
{
    UCHAR_T reg_value = 0;
    switch (sensor) {
    case MPU_GYRO_WORK:
        reg_value = MPU_RA_BIT_FIFO_EN_XYZG;
        break;
    case MPU_ACCEL_WORK:
        reg_value = MPU_RA_BIT_FIFO_EN_XYZA;
        break;
    case MPU_BOTH_WORK:
        reg_value = MPU_RA_BIT_FIFO_EN_XYZG | MPU_RA_BIT_FIFO_EN_XYZA;
        break;
    default:
        break;
    }
    __mpu6050_write_register_bit(MPU6050_RA_PWR_MGMT_2, reg_value, MPU_RA_BIT_FIFO_EN_XYZG|MPU_RA_BIT_FIFO_EN_XYZA);
}
#endif

/**
 * @brief set intterupt
 * @param[in] active_low: TRUE - active low, FALSE - active high
 * @return none
 */
STATIC VOID_T __mpu6050_set_int(BOOL_T active_low)
{
    UCHAR_T reg_value = 0;
    if (active_low) {
        __mpu6050_write_register(MPU6050_RA_INT_PIN_CFG, 0x90);
    } else {
        __mpu6050_write_register(MPU6050_RA_INT_PIN_CFG, 0x50);
    }
    __mpu6050_write_register(MPU6050_RA_INT_ENABLE, 0x01);
}

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
                          _IN TY_GPIO_IRQ_CB int_cb)
{
    /* I2C init */
    i2c_soft_gpio_init();
    /* reset MPU6050 */
    __mpu6050_reset();
    /* check communication */
    if (!__mpu6050_is_connected()) {
        return MPU_ERR_UNCONN;
    }

    /* MPU6050 init */
    __mpu6050_set_sleep_mode(FALSE);    /* wakeup MPU6050 */
    __mpu6050_set_clk_src(clk);         /* set clock source */
    __mpu6050_set_gyro_fsr(g_fsr);      /* set gyroscope's full-scale range */
    __mpu6050_set_accel_fsr(a_fsr);     /* set accelerometer's full-scale range */
    __mpu6050_set_sample_rate(smp_rt);  /* set sample rate */
    __mpu6050_set_dlpf(smp_rt/2);       /* set DLPF */

    /* save sensitivity scale factor */
    sg_gyro_sens = 32768.0 / ((1 << g_fsr) * 250);
    sg_accel_sens = 32768.0 / ((1 << a_fsr) * 2);

    /* interrupt init */
    if (int_cb != NULL) {
        if (tuya_gpio_irq_init(pin, type, int_cb)) {
            return MPU_ERR_IRQ_INIT_FAILED;
        }
        if (TY_GPIO_IRQ_FALLING == type) {
            __mpu6050_set_int(TRUE);
        } else {
            __mpu6050_set_int(FALSE);
        }
    }

    return MPU_OK;
}

/**
 * @brief read accelerometer data (raw data)
 * @param[out] a_x: accelerometer data of X-axis
 * @param[out] a_y: accelerometer data of Y-axis
 * @param[out] a_z: accelerometer data of Z-axis
 * @return none
 */
STATIC VOID_T __read_accel_raw(_OUT SHORT_T *a_x, _OUT SHORT_T *a_y, _OUT SHORT_T *a_z)
{
    UCHAR_T tmp_buf[6];

    /* read data from MPU6050 */
    __mpu6050_read_data(MPU6050_RA_ACCEL_XOUT_H, 6, tmp_buf);
    /* get acceleration */
    *a_x = ((SHORT_T)tmp_buf[0] << 8) | tmp_buf[1];
    *a_y = ((SHORT_T)tmp_buf[2] << 8) | tmp_buf[3];
    *a_z = ((SHORT_T)tmp_buf[4] << 8) | tmp_buf[5];
}

/**
 * @brief read accelerometer data from MPU6050 (raw data)
 * @param[out] a_x: accelerometer data of X-axis
 * @param[out] a_y: accelerometer data of Y-axis
 * @param[out] a_z: accelerometer data of Z-axis
 * @return none
 */
VOID_T tuya_mpu6050_read_accel_raw(_OUT SHORT_T *a_x, _OUT SHORT_T *a_y, _OUT SHORT_T *a_z)
{
    __read_accel_raw(a_x, a_y, a_z);
}

/**
 * @brief convert accelerometer data's unit to g
 * @param[in] data: accelerometer data
 * @return data in g
 */
STATIC FLOAT_T __accel_cnv_unit_to_g(_IN CONST SHORT_T data)
{
    FLOAT_T new_data;
    new_data = (FLOAT_T)data / sg_accel_sens;
    return new_data;
}

/**
 * @brief convert accelerometer data's unit to m/s^2
 * @param[in] data: accelerometer data
 * @return data in mps2
 */
STATIC FLOAT_T __accel_cnv_unit_to_mps2(_IN CONST SHORT_T data)
{
    FLOAT_T new_data;
    new_data = data * ACCEL_OF_G / sg_accel_sens;
    return new_data;
}

/**
 * @brief convert accelerometer data's unit
 * @param[in] ax: raw data of X-axis
 * @param[in] ay: raw data of Y-axis
 * @param[in] az: raw data of Z-axis
 * @param[out] a_x: converted data of X-axis
 * @param[out] a_y: converted data of Y-axis
 * @param[out] a_z: converted data of Z-axis
 * @param[in] unit: accelerometer unit
 * @return none
 */
STATIC VOID_T __cnv_accel_unit(_IN CONST SHORT_T ax, _IN CONST SHORT_T ay, _IN CONST SHORT_T az,
                               _OUT FLOAT_T *a_x, _OUT FLOAT_T *a_y, _OUT FLOAT_T *a_z, _IN CONST MPU_ACCEL_DT_E unit)
{
    if (unit == MPU_ADT_G) {
        *a_x = __accel_cnv_unit_to_g(ax);
        *a_y = __accel_cnv_unit_to_g(ay);
        *a_z = __accel_cnv_unit_to_g(az);
    } else {
        *a_x = __accel_cnv_unit_to_mps2(ax);
        *a_y = __accel_cnv_unit_to_mps2(ay);
        *a_z = __accel_cnv_unit_to_mps2(az);
    }
}

/**
 * @brief read accelerometer data from MPU6050 (specified unit)
 * @param[out] a_x: output data of X-axis
 * @param[out] a_y: output data of Y-axis
 * @param[out] a_z: output data of Z-axis
 * @param[in] unit: accelerometer unit
 * @return none
 */
VOID_T tuya_mpu6050_read_accel_spec_unit(_OUT FLOAT_T *a_x, _OUT FLOAT_T *a_y, _OUT FLOAT_T *a_z, _IN CONST MPU_ACCEL_DT_E unit)
{
    SHORT_T ax, ay, az;
    __read_accel_raw(&ax, &ay, &az);
    __cnv_accel_unit(ax, ay, az, a_x, a_y, a_z, unit);
}

/**
 * @brief read gyroscope data (raw data)
 * @param[out] g_x: gyroscope data of X-axis
 * @param[out] g_y: gyroscope data of Y-axis
 * @param[out] g_z: gyroscope data of Z-axis
 * @return none
 */
STATIC VOID_T __read_gyro_raw(_OUT SHORT_T *g_x, _OUT SHORT_T *g_y, _OUT SHORT_T *g_z)
{
    UCHAR_T tmp_buf[6];

    /* read data from MPU6050 */
    __mpu6050_read_data(MPU6050_RA_GYRO_XOUT_H, 6, tmp_buf);
    /* get angular rate */
    *g_x = ((SHORT_T)tmp_buf[0] << 8) | tmp_buf[1];
    *g_y = ((SHORT_T)tmp_buf[2] << 8) | tmp_buf[3];
    *g_z = ((SHORT_T)tmp_buf[4] << 8) | tmp_buf[5];
}

/**
 * @brief read gyroscope data from MPU6050 (raw data)
 * @param[out] g_x: gyroscope data of X-axis
 * @param[out] g_y: gyroscope data of Y-axis
 * @param[out] g_z: gyroscope data of Z-axis
 * @return none
 */
VOID_T tuya_mpu6050_read_gyro_raw(_OUT SHORT_T *g_x, _OUT SHORT_T *g_y, _OUT SHORT_T *g_z)
{
    __read_gyro_raw(g_x, g_y, g_z);
}

/**
 * @brief convert gyroscope data's unit to dps
 * @param[in] data: gyroscope data
 * @return data in dps
 */
STATIC FLOAT_T __gyro_cnv_unit_to_dps(_IN CONST SHORT_T data)
{
    FLOAT_T new_data;
    new_data = data / sg_gyro_sens;
    return new_data;
}

/**
 * @brief convert gyroscope data's unit to rps
 * @param[in] data: gyroscope data
 * @return data in rps
 */
STATIC FLOAT_T __gyro_cnv_unit_to_rps(_IN CONST SHORT_T data)
{
    FLOAT_T new_data;
    new_data = data / sg_gyro_sens / RPS_TO_DPS;
    return new_data;
}

/**
 * @brief convert gyroscope data's unit
 * @param[in] gx: raw data of X-axis
 * @param[in] gy: raw data of Y-axis
 * @param[in] gz: raw data of Z-axis
 * @param[out] g_x: converted data of X-axis
 * @param[out] g_y: converted data of Y-axis
 * @param[out] g_z: converted data of Z-axis
 * @param[in] unit: gyroscope unit
 * @return none
 */
STATIC VOID_T __cnv_gyro_unit(_IN CONST SHORT_T gx, _IN CONST SHORT_T gy, _IN CONST SHORT_T gz,
                              _OUT FLOAT_T *g_x, _OUT FLOAT_T *g_y, _OUT FLOAT_T *g_z, _IN CONST MPU_GYRO_DT_E unit)
{
    if (unit == MPU_GDT_DPS) {
        *g_x = __gyro_cnv_unit_to_dps(gx);
        *g_y = __gyro_cnv_unit_to_dps(gy);
        *g_z = __gyro_cnv_unit_to_dps(gz);
    } else {
        *g_x = __gyro_cnv_unit_to_rps(gx);
        *g_y = __gyro_cnv_unit_to_rps(gy);
        *g_z = __gyro_cnv_unit_to_rps(gz);
    }
}

/**
 * @brief read gyroscope data from MPU6050 (specified unit)
 * @param[out] g_x: output data of X-axis
 * @param[out] g_y: output data of Y-axis
 * @param[out] g_z: output data of Z-axis
 * @param[in] unit: gyroscope unit
 * @return none
 */
VOID_T tuya_mpu6050_read_gyro_spec_unit(_OUT FLOAT_T *g_x, _OUT FLOAT_T *g_y, _OUT FLOAT_T *g_z, _IN CONST MPU_GYRO_DT_E unit)
{
    SHORT_T gx, gy, gz;
    __read_gyro_raw(&gx, &gy, &gz);
    __cnv_gyro_unit(gx, gy, gz, g_x, g_y, g_z, unit);
}

/**
 * @brief read temperature data from MPU6050 (Celsius)
 * @param[in] none
 * @return temperature data in Celsius
 */
FLOAT_T tuya_mpu6050_read_temp(VOID_T)
{
    UCHAR_T tmp_buf[2];
    FLOAT_T temp;
    /* read data from MPU6050 */
    __mpu6050_read_data(MPU6050_RA_TEMP_OUT_H, 2, tmp_buf);
    /* get temperature */
    temp = ((SHORT_T)tmp_buf[0] << 8) | tmp_buf[1];
    /* unit conversion */
    temp = (36.53f + temp / 340.0f) * 10.0f;
    return temp;
}

#if INV_MOTION_DRIVER
/**
 * @brief row to scale
 * @param[in] row: row data
 * @return scale
 */
STATIC USHORT_T __inv_row_2_scale(_IN CONST CHAR_T *row)
{
    USHORT_T b;

    if (row[0] > 0) {
        b = 0;
    } else if (row[0] < 0) {
        b = 4;
    } else if (row[1] > 0) {
        b = 1;
    } else if (row[1] < 0) {
        b = 5;
    } else if (row[2] > 0) {
        b = 2;
    } else if (row[2] < 0) {
        b = 6;
    } else {
        b = 7;  /* error */
    }
    return b;
}

/**
 * @brief orientation matrix to scalar
 * @param[in] mtx: orientation matrix
 * @return scalar
 */
STATIC USHORT_T __inv_orientation_matrix_to_scalar(_IN CONST CHAR_T *mtx)
{
    USHORT_T scalar;
    scalar = __inv_row_2_scale(mtx);
    scalar |= __inv_row_2_scale(mtx + 3) << 3;
    scalar |= __inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

/**
 * @brief run self test
 * @param[in] none
 * @return none
 */
STATIC INT_T __run_self_test(VOID_T)
{
    INT_T result;
    LONG_T gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        FLOAT_T gyro_sens;
        USHORT_T accel_sens;

        mpu_get_gyro_sens(&gyro_sens);
        gyro[0] = (LONG_T)(gyro[0] * gyro_sens);
        gyro[1] = (LONG_T)(gyro[1] * gyro_sens);
        gyro[2] = (LONG_T)(gyro[2] * gyro_sens);
        dmp_set_gyro_bias(gyro);

        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    } else {
        return -1;
    }
    return 0;
}

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
                              _IN CONST TY_GPIO_PORT_E pin, _IN CONST TY_GPIO_IRQ_TYPE_E type, _IN TY_GPIO_IRQ_CB int_cb)
{
    /* I2C init */
    i2c_soft_gpio_init();
    /* reset MPU6050 */
    __mpu6050_reset();
    /* check communication */
    if (!__mpu6050_is_connected()) {
        return MPU_ERR_UNCONN;
    }
    /* MPU6050 init */
	if (mpu_init()) {
        return MPU_ERR_INIT_FAILED;
    }
    /* config interrupt */
    if (int_cb != NULL) {
        if (tuya_gpio_irq_init(pin, type, int_cb)) {
            return MPU_ERR_IRQ_INIT_FAILED;
        }
    }
    /* wake up all sensors */
    if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {
        return MPU_ERR_SENS_TURN_ON_FAILED;
    }
    /* push both gyro and accel data into the FIFO */
    if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {
        return MPU_ERR_CFG_FIFO_FAILED;
    }
    /* set sample rate */
    if (mpu_set_sample_rate(MPU_SMPRT_DEFAULT)) {
        return MPU_ERR_SET_SMPRT_FAILED;
    }
    /* set gyro full-scale range */
    if (mpu_set_gyro_fsr((1 << g_fsr) * 250)) {
        return MPU_ERR_SET_GFSR_FAILED;
    }
    mpu_get_gyro_sens(&sg_gyro_sens);
    /* set accel full-scale range */
    if (mpu_set_accel_fsr((1 << a_fsr) * 2)) {
        return MPU_ERR_SET_AFSR_FAILED;
    }
    mpu_get_accel_sens(&sg_accel_sens);
    /* push the DMP image in inv_mpu_dmp_motion_driver.h into the MPU memory */
    if (dmp_load_motion_driver_firmware()) {
        return MPU_ERR_LOAD_IMG_FAILED;
    }
    /* push gyro and accel orientation matrix to DMP */
    if (dmp_set_orientation(__inv_orientation_matrix_to_scalar(gyro_orientation))) {
        return MPU_ERR_SET_ORIEN_FAILED;
    }
    /* enable DMP features */
    if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL)) {
        return MPU_ERR_EN_FEAT_FAILED;
    }
    /* set DMP output rate */
    if (dmp_set_fifo_rate(MPU_SMPRT_DEFAULT)) {
        return MPU_ERR_SET_FIFO_RT_FAILED;
    }
    /* run self test */
    if (__run_self_test()) {
        return MPU_ERR_SELF_TEST_FAILED;
    }
    /* enable/disable DMP support */
	if (mpu_set_dmp_state(TRUE)) {
        return MPU_ERR_EN_DMP_FAILED;
    }
    return MPU_OK;
}

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
                                _OUT FLOAT_T *roll, _OUT FLOAT_T *pitch, _OUT FLOAT_T *yaw)
{
	UCHAR_T more;
	SHORT_T sensors;
	LONG_T quat[4];
    ULONG_T timestamp;
    FLOAT_T q0, q1, q2, q3;

    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more)) {
        return -1;
    }
    if ((sensors & (INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_WXYZ_QUAT)) == 0) {
        return -1;
    }
    q0 = quat[0] / Q30;
    q1 = quat[1] / Q30;
    q2 = quat[2] / Q30;
    q3 = quat[3] / Q30;
    *pitch = asin(2 * (q0*q2 - q1*q3)) * RPS_TO_DPS;
    *roll = atan2(2 * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3) * RPS_TO_DPS;
    *yaw = atan2(2 * (q0*q3 + q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3) * RPS_TO_DPS;

    return 0;
}

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
                                      _OUT FLOAT_T *roll, _OUT FLOAT_T *pitch, _OUT FLOAT_T *yaw)
{
	UCHAR_T more;
	SHORT_T g_tmp[3], a_tmp[3], sensors;
	LONG_T quat[4];
    ULONG_T timestamp;
    FLOAT_T q0, q1, q2, q3;

    if (dmp_read_fifo(g_tmp, a_tmp, quat, &timestamp, &sensors, &more)) {
        return -1;
    }
    if ((sensors & (INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_WXYZ_QUAT)) == 0) {
        return -1;
    }

    __cnv_gyro_unit(g_tmp[0], g_tmp[1], g_tmp[2], gyro, gyro+1, gyro+2, g_unit);
    __cnv_accel_unit(a_tmp[0], a_tmp[1], a_tmp[2], accel, accel+1, accel+2, a_unit);

    q0 = quat[0] / Q30;
    q1 = quat[1] / Q30;
    q2 = quat[2] / Q30;
    q3 = quat[3] / Q30;
    *roll = atan2(2 * (q0*q1 + q2*q3), q0*q0 - q1*q1 - q2*q2 + q3*q3) * RPS_TO_DPS;
    *pitch = asin(2 * (q0*q2 - q1*q3)) * RPS_TO_DPS;
    *yaw = atan2(2 * (q0*q3 + q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3) * RPS_TO_DPS;

    return 0;
}

/**
 * @brief read data of MPU6050 (called in inv_mpu.c and inv_mpu_dmp_motion_driver.c)
 * @param[in] dev_addr: device address
 * @param[in] reg_addr: register address
 * @param[in] len: data length
 * @param[out] data: read data
 * @return default: 0
 */
INT_T tuya_mpu6050_i2c_read(_IN CONST UCHAR_T dev_addr, _IN UCHAR_T reg_addr, _IN CONST UCHAR_T len, _OUT UCHAR_T *data)
{
    i2c_start();
    i2c_send_bytes(MPU6050_ADDR_CMD_WRITE, &reg_addr, 1);
    i2c_start();
    i2c_rcv_bytes(MPU6050_ADDR_CMD_READ, data, len);
    i2c_stop();
    return 0;
}

/**
 * @brief write data of MPU6050 (called in inv_mpu.c and inv_mpu_dmp_motion_driver.c)
 * @param[in] dev_addr: device address
 * @param[in] reg_addr: register address
 * @param[in] len: data length
 * @param[in] data: write data
 * @return default: 0
 */
INT_T tuya_mpu6050_i2c_write(_IN CONST UCHAR_T dev_addr, _IN UCHAR_T reg_addr, _IN CONST UCHAR_T len, _IN UCHAR_T *data)
{
    i2c_start();
    i2c_send_bytes(MPU6050_ADDR_CMD_WRITE, &reg_addr, 1);
    i2c_send_bytes(data[0], data+1, len-1);
    i2c_stop();
    return 0;
}

/**
 * @brief get ms timestamp (called in inv_mpu.c and inv_mpu_dmp_motion_driver.c)
 * @param[out] timestamp: ms timestamp
 * @return none
 */
VOID_T tuya_ble_get_ms(_OUT ULONG_T *timestamp)
{
    INT_T timezone = 0;
    tuya_ble_rtc_get_timestamp((UINT_T *)timestamp, &timezone);
}
#endif /* INV_MOTION_DRIVER */
