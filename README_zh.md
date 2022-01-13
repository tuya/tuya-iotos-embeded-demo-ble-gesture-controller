# TuyaOS Embedded Bluetooth Low Energy Intuitive Gesture Remote Control

[English](./README.md) | [中文](./README_zh.md)

<br>

## 简介

本 demo 基于 [涂鸦IoT平台](https://iot.tuya.com/) 、涂鸦智能APP、IoTOS Embeded BLE SDK，使用涂鸦 BLE 系列模组快速组建一个手势控制器应用程序。该手势控制器主要由 1 个配网键、1 个识别功能开/关键、1 个指示灯和 1个惯性传感器 (MPU6050) 组成，实现了基础的甩动手势 (上、下、左、右) 和翻转手势 (顺时针、逆时针) 识别，并可通过智慧生活 APP 和蓝牙网关实现对其他设备的控制。

<br>

## 快速上手

### 开发环境搭建

- IDE 根据芯片原厂 SDK 要求进行安装。
- Tuya BLE SDK Demo Project 的下载需先在 [涂鸦IoT平台](https://iot.tuya.com/) 创建智能产品。在【硬件开发】页面选择好【云端接入硬件】后，可在左下角的【开发资料】中下载对应的 SDK。

<br>

### 编译烧录

- 代码修改

  1. 在 `tuya_ble_sdk_demo.h` 中填入在 [涂鸦IoT平台](https://iot.tuya.com/) 创建的智能产品 PID、mac 地址、uuid 和 authkey。

     ```c
     #define TY_DEVICE_NAME        "demo"
     #define TY_DEVICE_PID         "xxxxxxxx" /* PID */
     #define TY_DEVICE_MAC         "xxxxxxxx" /* mac */
     #define TY_DEVICE_DID         "xxxxxxxx" /* uuid */
     #define TY_DEVICE_AUTH_KEY    "xxxxxxxx" /* authkey */
     ```

     按注释中的提示将对应的 `xxxxxxxx` 进行替换。

  2. 在 `tuya_ble_sdk_demo.c` 中将初始化参数 `use_ext_license_key` 和`device_id_len` 的值分别修改为 1 和 16，以使激活码生效。

     ```c
     static tuya_ble_device_param_t tuya_ble_device_param = {
         .use_ext_license_key = 1,	// 1-info in tuya_ble_sdk_demo.h, 0-auth info
         .device_id_len       = 16,	// DEVICE_ID_LEN,
         .p_type              = TUYA_BLE_PRODUCT_ID_TYPE_PID,
         .product_id_len      = 8,
         .adv_local_name_len  = 4,
         .firmware_version    = TY_DEVICE_FVER_NUM,
         .hardware_version    = TY_DEVICE_HVER_NUM,
     };
     ```

- 编译运行Demo代码

  修改好后编译代码，并下载固件至硬件运行（根据所选的芯片型号有可能还需要下载 stack 和 bootloader），观察 log 日志 ，并用第三方蓝牙调试 App（例如 IOS 下的 lightBlue）扫描确认设备有没有正常广播。

<br>

### 文件介绍
```
├── include
|    ├── common
|    |    └── tuya_common.h                /* 通用类型和宏定义 */
|    ├── driver
|    |    ├── tuya_key.h                   /* 按键驱动 */
|    |    ├── tuya_led.h                   /* LED驱动 */
|    |    └── tuya_mpu6050.h               /* MPU6050驱动 */
|    ├── platform
|    |    ├── tuya_gpio.h                  /* 平台关联GPIO驱动 */
|    |    └── tuya_pwr_mgmt.h              /* 平台关联低功耗模式处理 */
|    ├── sdk
|    |    ├── tuya_ble_bulk_data_demo.h    /* 大数据通道例程 */
|    |    ├── tuya_ble_product_test_demo.h /* 整机产测例程 */
|    |    └── tuya_ble_sdk_test.h          /* 实现tuya_ble_sdk测试的串口指令 */
|    ├── tuya_ble_sdk_demo.h               /* 实现tuya_ble_sdk的初始化，应用程序入口 */
|    ├── tuya_imu_daq.h                    /* 传感数据采集 */
|    ├── tuya_gesture_controller.h         /* 手势控制器管理中心 */
|    ├── tuya_gesture_rec.h                /* 手势动作识别 */
|    ├── tuya_net_proc.h                   /* 设备联网处理 */
|    └── tuya_svc_angle_calc.h             /* 姿态解算服务 */
└── src
     ├── driver
     |    ├── tuya_key.c                   /* 按键驱动 */
     |    ├── tuya_led.c                   /* LED驱动 */
     |    └── tuya_mpu6050.c               /* MPU6050驱动 */
     ├── platform
     |    ├── tuya_gpio_nRF52832.c         /* nRF52832平台关联GPIO驱动 */
     |    └── tuya_pwr_mgmt_nRF52832.c     /* nRF52832平台关联低功耗模式处理 */
     ├── sdk
     |    ├── tuya_ble_bulk_data_demo.c    /* 大数据通道例程 */
     |    ├── tuya_ble_product_test_demo.c /* 整机产测例程 */
     |    └── tuya_ble_sdk_test.c          /* SDK测试程序 */
     ├── tuya_ble_sdk_demo.c               /* 实现tuya_ble_sdk的初始化，应用程序入口 */
     ├── tuya_gesture_controller.c         /* 手势控制器管理中心 */
     ├── tuya_imu_daq.c                    /* 传感数据采集 */
     ├── tuya_gesture_rec.c                /* 手势动作识别 */
     ├── tuya_net_proc.c                   /* 设备联网处理 */
     └── tuya_svc_angle_calc.c             /* 姿态解算服务 */
```

<br>

### 应用入口

入口文件：`tuya_ble_sdk_demo.c` 和 `tuya_ble_main.c`

+ `tuya_ble_sdk_demo_init()` 对 Tuya IoTOS Embeded Ble SDK 进行一些必要的初始化，该函数只执行一次。`tuya_gesture_controller_init()` 对手势控制器应用程序进行一些必要的初始化。

     ```c
     void tuya_ble_sdk_demo_init(void)
     {
         ...
         tuya_gesture_controller_init();
     }
     ```

+ `tuya_gesture_controller_loop()` 用来循环执行手势控制器的应用代码，需在如下函数中调用：

     ```c
     void tuya_ble_main_tasks_exec(void)
     {
         tuya_gesture_controller_loop();
         tuya_sched_execute();
     }
     ```

<br>

### DP点相关

| 函数名 | tuya_ble_dp_data_send |
| :---: | :--- |
| 函数原型 | tuya_ble_status_t tuya_ble_dp_data_send(<br/>uint32_t sn,<br/>tuya_ble_dp_data_send_type_t type,<br/>tuya_ble_dp_data_send_mode_t mode,<br/>tuya_ble_dp_data_send_ack_t ack,<br/>uint8_t *p_dp_data,<br/>uint32_t dp_data_len<br/>) ; |
| 功能概述 | 发送 DP 数据。 |
| 参数 | sn[in]：发送序号<br/>type[in]：发送类型，分为主动发送和应答查询发送<br/>mode[in]：发送模式<br/>ack[in]：是否需要应答标志<br/>p_dp_data [in]：DP 数据<br/>dp_data_len[in]：数据长度，最大不能超过 `TUYA_BLE_SEND_MAX_DATA_LEN-7`。其中 `TUYA_BLE_SEND_MAX_DATA_LEN `可配置 |
| 返回值 | TUYA_BLE_SUCCESS ：发送成功；<br/>TUYA_BLE_ERR_INVALID_PARAM：参数无效；<br/>TUYA_BLE_ERR_INVALID_STATE：当前状态不支持发送，例如蓝牙断开；<br/>TUYA_BLE_ERR_NO_MEM：内存申请失败；<br/>TUYA_BLE_ERR_INVALID_LENGTH：数据长度错误；<br/>TUYA_BLE_ERR_NO_EVENT：其他错误。 |
| 备注 | 应用程序通过调用该函数上报 DP 数据到 App。 |

**p_dp_data** 参数说明：

[涂鸦IoT平台](https://iot.tuya.com/) 是以 DP 模型管理数据。任何设备产生的数据都需要抽象为 DP 数据形式，一个 DP 数据由四部分组成。更多详情，请参考 [自定义功能](https://developer.tuya.com/cn/docs/iot/custom-functions?id=K937y38137c64)。

- Dp_id：1个字节，在涂鸦 IoT 平台注册的 dp_id 序号。

- Dp_type：1 个字节，DP 类型，根据上报的 DP 类型来选择。

  - `#define DT_RAW 0`：Raw类型；
  - `#define DT_BOOL 1` ：布尔类型；
  - `#define DT_VALUE 2`：数值类型，其范围在iot平台注册时指定。
  - `#define DT_STRING 3`：字符串类型；
  - `#define DT_ENUM 4 `：枚举类型；
  - `#define DT_BITMAP 5`：位映射类型；

- Dp_len：DP 数据长度，两个字节，每个 DP 数据类型的最大数据长度在涂鸦 IoT平台定义时指定。

  - 若 `Dp_type` = 1, 则 `Dp_len` 必须为 1。
  - 若 `Dp_type` = 2, 则 `Dp_len` 可以为1、2、4。
  - 若 `Dp_type` = 4, 则 `Dp_len` 必须为 1。
  - 若 `Dp_type` = 5, 则 `Dp_len` 可以为1、2、4。
  - 若 `Dp_type` = 0 或 3, 则 `Dp_len` 数值自定义，但必须小于在涂鸦 IoT平台定义时的最大长度。

- Dp_data：数据，dp_len 个字节数据。

该 DP 上报函数的参数 Dp_data 指向的数据必须以下表格式组装上报：

| Dp点1的数据 |  |  |  | ~  | Dp点n的数据 |  |  |  |
| :---: | :---: | :---: | :---: | :--- | :---: | :---: | :---: | :---: |
| 1 | 2 | 3-4 | 5~ | ~ | n| n+1 | n+2-n+3 | n+4~ |
| Dp_id | Dp_type | Dp_len | Dp_data | ~ | Dp_id | Dp_type | Dp_len | Dp_data |

一次可发送多个dp数据，只要总长度不超过限制即可，最大长度为
`TUYA_BLE_SEND_MAX_DATA_LEN-7` ,其中 `TUYA_BLE_SEND_MAX_DATA_LEN` 可配置。

<br>

### I/O 列表

| 外设 | I/O | 外设 | I/O |
| :---: | :---: | :---: | :---: |
| 功能键 | IO4 | MPU6050 数字I/O供电 VLOGIC | IO16_D |
| 配网键 | IO5 | MPU6050 中断输出 INT | IO2_A |
| 指示灯 | IO12 | MPU6050 I2C 通信 SCL | IO14_D |
| 串口通信 TXD/RXD | IO18_D/IO20_D | MPU6050 I2C 通信 SDA | IO11 |

<br>

## 相关文档

- [蓝牙 SDK 开发](https://developer.tuya.com/cn/docs/iot-device-dev/BLE-SDK?id=Kalgco5r2mr0h)
- [涂鸦 Demo 中心](https://developer.tuya.com/demo)

<br>

## 技术支持

您可以通过以下方法获得涂鸦的支持:

- [涂鸦 AI+IoT 开发者平台](https://developer.tuya.com)
- [帮助中心](https://support.tuya.com/help)
- [服务与支持](https://service.console.tuya.com)
