# TuyaOS Embedded Bluetooth Low Energy Intuitive Gesture Remote Control

[English](./README.md) | [中文](./README_zh.md)

<br>

## Overview

In this demo, we will show you how to make a smart gesture-based remote control. Based on the [Tuya IoT Platform](https://iot.tuya.com/), we use Tuya's Bluetooth Low Energy module, SDK, and the Smart Life app to connect this thing to the cloud. This gesture remote control consists of one button for pairing, one button for turning on/off gesture recognition, one LED indicator, and one MPU6050 accelerometer and gyroscope sensor. This smart wand allows you to use gestures to control your device: roll clockwise, roll counter-clockwise, flick up or down, and flick left or right. You can also use the Smart Life app and Bluetooth gateway to achieve remote control of your smart home.

<br>

## Get started

### Set up IDE

- Install the integrated development environment (IDE) as per your chip platform.
- To get the SDK, you need to create a product on the [Tuya IoT Platform](https://iot.tuya.com/). Then, go to the Hardware Development step, select a connection method, and then download the SDK.

<br>

### Compilation and flashing

- Edit code

   1. In `tuya_ble_sdk_demo.h`, populate the PID, MAC address, UUID, and authKey with the value you obtained from the [Tuya IoT Platform](https://iot.tuya.com/).

      ```c
      #define TY_DEVICE_NAME        "demo"
      #define TY_DEVICE_PID         "xxxxxxxx" /* PID */
      #define TY_DEVICE_MAC         "xxxxxxxx" /* mac */
      #define TY_DEVICE_DID         "xxxxxxxx" /* uuid */
      #define TY_DEVICE_AUTH_KEY    "xxxxxxxx" /* authkey */
      ```

      Replace `xxxxxxxx` with the value you obtained.

   2. In `tuya_ble_sdk_demo.c`, change the values of `use_ext_license_key` and `device_id_len` to 1 and 16 respectively to make the license effective.

      ```c
      static tuya_ble_device_param_t tuya_ble_device_param = {
          .use_ext_license_key = 1,	 // 1-info in tuya_ble_sdk_demo.h, 0-auth info
          .device_id_len       = 16,	 // DEVICE_ID_LEN,
          .p_type              = TUYA_BLE_PRODUCT_ID_TYPE_PID,
          .product_id_len      = 8,
          .adv_local_name_len  = 4,
          .firmware_version    = TY_DEVICE_FVER_NUM,
          .hardware_version    = TY_DEVICE_HVER_NUM,
      };
      ```

- Compile code

   Compile the edited code, download the code to the hardware, and run it. You may need to download the stack and bootloader depending on your chip models. Check the logs and use the third-party Bluetooth debugging app (such as LightBlue for iOS) to verify the Bluetooth broadcast.

<br>

### File introduction
```
├── include
|    ├── common
|    |    └── tuya_common.h                /* Common types and macros */
|    ├── driver
|    |    ├── tuya_key.h                   /* Key driver */
|    |    ├── tuya_led.h                   /* LED driver */
|    |    └── tuya_mpu6050.h               /* MPU6050 driver */
|    ├── platform
|    |    ├── tuya_gpio.h                  /* GPIO driver */
|    |    └── tuya_pwr_mgmt.h              /* Low power mode processing */
|    ├── sdk
|    |    ├── tuya_ble_bulk_data_demo.h    /* Bulk data transmission */
|    |    ├── tuya_ble_product_test_demo.h /* Routine for end product testing */
|    |    └── tuya_ble_sdk_test.h          /* UART commands for testing */
|    ├── tuya_ble_sdk_demo.h               /* Entry to the main application. SDK initialization. */
|    ├── tuya_imu_daq.h                    /* Sensor data collection */
|    ├── tuya_gesture_controller.h         /* Gesture control management */
|    ├── tuya_gesture_rec.h                /* Gesture recognition */
|    ├── tuya_net_proc.h                   /* Network connection */
|    └── tuya_svc_angle_calc.h             /* Attitude calculation */
└── src
     ├── driver
     |    ├── tuya_key.c                   /* Key driver */
     |    ├── tuya_led.c                   /* LED driver */
     |    └── tuya_mpu6050.c               /* MPU6050 driver */
     ├── platform
     |    ├── tuya_gpio_nRF52832.c         /* nRF52832 GPIO driver */
     |    └── tuya_pwr_mgmt_nRF52832.c     /* nRF52832 low power mode processing */
     ├── sdk
     |    ├── tuya_ble_bulk_data_demo.c    /* Bulk data transmission */
     |    ├── tuya_ble_product_test_demo.c /* Routine for end product testing */
     |    └── tuya_ble_sdk_test.c          /* Program for testing SDK */
     ├── tuya_ble_sdk_demo.c               /* Entry to the main application. SDK initialization. */
     ├── tuya_gesture_controller.c         /* Gesture control management */
     ├── tuya_imu_daq.c                    /* Sensor data collection  */
     ├── tuya_gesture_rec.c                /* Gesture recognition */
     ├── tuya_net_proc.c                   /* Network connection */
     └── tuya_svc_angle_calc.c             /* Attitude calculation */
```

<br>

### Entry to application

Entry file: `tuya_ble_sdk_demo.c` and `tuya_ble_main.c`

+ `tuya_ble_sdk_demo_init()` is run to initialize the SDK. This function is run only once. `tuya_gesture_controller_init()` is run to initialize the gesture control program.

     ```c
     void tuya_ble_sdk_demo_init(void)
     {
         ...
         tuya_gesture_controller_init();
     }
     ```

+ `tuya_gesture_controller_loop()` is used to loop the application code of the gesture control. Call this loop function in `tuya_ble_main_tasks_exec()`.

     ```c
     void tuya_ble_main_tasks_exec(void)
     {
         tuya_gesture_controller_loop();
         tuya_sched_execute();
     }
     ```

<br>

### Data point (DP)

| Function | tuya_ble_dp_data_send |
| :---: | :--- |
| Function prototype | tuya_ble_status_t tuya_ble_dp_data_send(<br/>uint32_t sn,<br/>tuya_ble_dp_data_send_type_t type,<br/>tuya_ble_dp_data_send_mode_t mode,<br/>tuya_ble_dp_data_send_ack_t ack,<br/>uint8_t *p_dp_data,<br/>uint32_t dp_data_len<br/>) ; |
| Feature overview | Send data point (DP) data to the cloud. |
| Parameters | `sn[in]`: the sequence number. <br/>`type[in]`: the type of data sending, which can be a proactive notification or a follow-up response. <br/>`mode[in]`: the delivery mode. <br/>`ack[in]`: whether an ACK message is required. <br/>`p_dp_data[in]`: the DP data. <br/>`dp_data_len[in]`: the length of data, no more than `TUYA_BLE_SEND_MAX_DATA_LEN-7`. `TUYA_BLE_SEND_MAX_DATA_LEN` is configurable. |
| Return value | `TUYA_BLE_SUCCESS`: sent successfully. <br/>`TUYA_BLE_ERR_INVALID_PARAM`: invalid parameter. <br/>`TUYA_BLE_ERR_INVALID_STATE`failed to send data due to the current Bluetooth connection, such as Bluetooth disconnected. <br/>`TUYA_BLE_ERR_NO_MEM`: failed to request memory allocation. <br/>`TUYA_BLE_ERR_INVALID_LENGTH`: data length error.<br/>`TUYA_BLE_ERR_NO_EVENT`: other errors. |
| Notes | The application calls this function to send DP data to the mobile app. |

**`p_dp_data`** parameter description:

The [Tuya IoT Platform](https://iot.tuya.com/) manages data through DPs. Each product feature defined on the platform is described as a DP. The DP data consists of four parts. For more information, see [Custom Functions](https://developer.tuya.com/en/docs/iot/custom-functions?id=K937y38137c64).

- `Dp_id`: The 1-byte parameter indicates the ID of a DP defined on the Tuya IoT Development Platform.

- `Dp_type`: The 1-byte parameter indicates the data type.

   - `#define DT_RAW  0`: raw type.
   - `#define DT_BOOL  1`: Boolean type.
   - `#define DT_VALUE 2`: value type. The value range is specified when a DP of value type is created on the Tuya IoT Development Platform.
   - `#define DT_STRING 3`: string type.
   - `#define DT_ENUM  4`: enum type.
   - `#define DT_BITMAP 5`: bitmap type.

- `Dp_len`: The 2-byte parameter indicates the length of a DP. The maximum length of a DP is specified when you define a DP on the Tuya IoT Development Platform.

   - If `Dp_typ`e is 1, `Dp_len` must be 1.
   - If `Dp_type` is 2, `Dp_len` can be 1, 2, or 4.
   - If `Dp_type` is 4, `Dp_len` must be 1.
   - If `Dp_type` is 5, `Dp_len` can be 1, 2, and 4.
   - If `Dp_type` is 0 or 3, `Dp_len` can be customized but must not exceed the maximum length defined on the Tuya IoT Development Platform.

- `Dp_data`: the DP data with the length of `dp_len`.

The data that the parameter `Dp_data` points to must be packaged in the following format for reporting.

| DP 1 data |  |  |  | — | DP n data |  |  |  |
| :---: | :---: | :---: | :---: | :--- | :---: | :---: | :---: | :---: |
| 1 | 2 | 3 to 4 | 5 and greater | — | n | n+1 | n+2 to n+3 | n+4 and greater |
| Dp_id | Dp_type | Dp_len | Dp_data | — | Dp_id | Dp_type | Dp_len | Dp_data |

You can send the data of multiple DPs at a time. Make sure the total length does not exceed `TUYA_BLE_SEND_MAX_DATA_LEN-7`. The parameter `TUYA_BLE_SEND_MAX_DATA_LEN` is configurable.

<br>

### Pin configuration

| Peripherals | I/O | Peripherals | I/O |
| :---: | :---: | :---: | :---: |
| Function key | IO4 | Digital I/O power supply of MPU6050 (VLOGIC) | IO16_D |
| Device pairing key | IO5 | Interrupt output of MPU6050 (INT) | IO2_A |
| LED indicator | IO12 | SCL of the MPU6050's I2C | IO14_D |
| TXD/RXD for UART communication | IO18_D/IO20_D | SDA of the MPU6050's I2C | IO11 |

<br>

## Reference

- [Bluetooth SDK Development](https://developer.tuya.com/en/docs/iot-device-dev/BLE-SDK?id=Kalgco5r2mr0h)
- [Tuya Project Hub](https://developer.tuya.com/demo)

<br>

## Technical support

You can get support from Tuya with the following methods:

+ [Tuya Developer Platform](https://developer.tuya.com/en/)
+ [Help Center](https://support.tuya.com/en/help)
+ [Service & Support](https://service.console.tuya.com)

<br>
