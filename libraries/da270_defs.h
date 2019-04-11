/**
 * Copyright (C) 2017 - 201 MiraMEMS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file	   da270_defs.h
 * @date       3 Apr 2019
 * @version    0.0.1
 * @brief
 *
 */
/*! \file da270_defs.h */
/*!
 * @defgroup DA270 SENSOR API
 * @brief
 * @{
 */
#ifndef DA270_DEFS_H_
#define DA270_DEFS_H_
/*********************************************************************/
/**\ header files */
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif
/*********************************************************************/
/** \name		Common macros				     */
/*********************************************************************/

#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)       S8_C(x)
#define UINT8_C(x)      U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)      S16_C(x)
#define UINT16_C(x)     U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)      S32_C(x)
#define UINT32_C(x)     U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)      S64_C(x)
#define UINT64_C(x)     U64_C(x)
#endif
/**\name C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL   0
#else
#define NULL   ((void *) 0)
#endif
#endif

#ifndef TRUE
#define TRUE     UINT8_C(1)
#endif

#ifndef FALSE
#define FALSE    UINT8_C(0)
#endif
/********************************************************/
/**\name Macro definitions */
/**\name API success code */
#define DA270_OK       INT8_C(0)
/**\name API error codes */
#define DA270_E_NULL_PTR          INT8_C(-1)
#define DA270_E_COM_FAIL          INT8_C(-2)
#define DA270_E_DEV_NOT_FOUND     INT8_C(-3)
#define DA270_E_INVALID_CONFIG    INT8_C(-4)
/**\name API warning codes */
#define DA270_W_SELF_TEST_FAIL    INT8_C(1)
/**\name CHIP ID VALUE */
#define DA270_CHIP_ID     UINT8_C(0x13)
/********************************************************/
/**\name	USER CONFIGURATION MACROS               */
/**\name DA270 I2C address macros */
#define DA270_I2C_ADDRESS_SDO_LOW     UINT8_C(0x14)
#define DA270_I2C_ADDRESS_SDO_HIGH    UINT8_C(0x15)
/**\name Power mode configurations */
#define DA270_NORMAL_MODE       UINT8_C(0)
#define DA270_SLEEP_MODE        UINT8_C(1)
#define DA270_LOW_POWER_MODE    UINT8_C(0)
/**\name Enable / Disable macros */
#define DA270_DISABLE        UINT8_C(0)
#define DA270_ENABLE         UINT8_C(1)
/**\name Data/sensortime selection macros */
#define DA270_DATA_ONLY          UINT8_C(0x00)
#define DA270_DATA_SENSOR_TIME   UINT8_C(0x01)
/**\name ODR configurations  */
#define DA270_ODR_1_HZ        UINT8_C(0x00)
#define DA270_ODR_1_95HZ     UINT8_C(0x01)
#define DA270_ODR_3_9HZ      UINT8_C(0x02)
#define DA270_ODR_7_81HZ     UINT8_C(0x03)
#define DA270_ODR_15_63HZ    UINT8_C(0x04)
#define DA270_ODR_31_25HZ    UINT8_C(0x05)
#define DA270_ODR_62_5HZ     UINT8_C(0x06)
#define DA270_ODR_125HZ      UINT8_C(0x07)
#define DA270_ODR_250HZ      UINT8_C(0x08)
#define DA270_ODR_500HZ      UINT8_C(0x09)
#define DA270_ODR_1000HZ      UINT8_C(0x0A)
/**\name Accel Range configuration */
#define DA270_2G_RANGE      UINT8_C(0x00)
#define DA270_4G_RANGE      UINT8_C(0x01)
#define DA270_8G_RANGE      UINT8_C(0x02)
#define DA270_16G_RANGE     UINT8_C(0x03)
/**\name Accel Axes selection settings for
 * DATA SAMPLING, WAKEUP, ORIENTATION CHANGE,
 * GEN1, GEN2 , ACTIVITY CHANGE
 */
#define DA270_X_AXIS_EN       UINT8_C(0x01)
#define DA270_Y_AXIS_EN       UINT8_C(0x02)
#define DA270_Z_AXIS_EN       UINT8_C(0x04)
#define DA270_XYZ_AXIS_EN     UINT8_C(0x07)

/**\name Accel bw settings */
/* Accel bw = 0.48 * ODR */
#define DA270_ACCEL_FILT1_BW_0    UINT8_C(0x00)
/* Accel bw = 0.24 * ODR */
#define DA270_ACCEL_FILT1_BW_1    UINT8_C(0x01)
/**\name Auto wake-up timeout value of 10.24s */
#define DA270_AUTO_WAKEUP_TIMEOUT_MAX  UINT16_C(0x0FFF)
/**\name Auto low power timeout value of 10.24s */
#define DA270_AUTO_LP_TIMEOUT_MAX      UINT16_C(0x0FFF)
/**\name Reference Update macros */
#define DA270_MANUAL_UPDATE         UINT8_C(0x00)
#define DA270_ONE_TIME_UPDATE       UINT8_C(0x01)
#define DA270_EVERY_TIME_UPDATE     UINT8_C(0x02)
#define DA270_LP_EVERY_TIME_UPDATE  UINT8_C(0x03)
/**\name Reference Update macros for orient interrupts */
#define DA270_ORIENT_REFU_ACC_FILT_2       UINT8_C(0x01)
#define DA270_ORIENT_REFU_ACC_FILT_LP      UINT8_C(0x02)
/**\name Stability evaluation macros for orient interrupts */
#define DA270_STABILITY_DISABLED        UINT8_C(0x00)
#define DA270_STABILITY_ACC_FILT_2      UINT8_C(0x01)
#define DA270_STABILITY_ACC_FILT_LP     UINT8_C(0x02)
/**\name Number of samples needed for Auto-wakeup interrupt evaluation  */
#define DA270_SAMPLE_COUNT_1   UINT8_C(0x00)
#define DA270_SAMPLE_COUNT_2   UINT8_C(0x01)
#define DA270_SAMPLE_COUNT_3   UINT8_C(0x02)
#define DA270_SAMPLE_COUNT_4   UINT8_C(0x03)
#define DA270_SAMPLE_COUNT_5   UINT8_C(0x04)
#define DA270_SAMPLE_COUNT_6   UINT8_C(0x05)
#define DA270_SAMPLE_COUNT_7   UINT8_C(0x06)
#define DA270_SAMPLE_COUNT_8   UINT8_C(0x07)
/**\name Auto low power configurations */
/* Auto low power timeout disabled  */
#define DA270_AUTO_LP_TIMEOUT_DISABLE    UINT8_C(0x00)
/* Auto low power entered on drdy interrupt */
#define DA270_AUTO_LP_DRDY_TRIGGER       UINT8_C(0x01)
/* Auto low power entered on GEN1 interrupt */
#define DA270_AUTO_LP_GEN1_TRIGGER       UINT8_C(0x02)
/* Auto low power entered on timeout of threshold value */
#define DA270_AUTO_LP_TIMEOUT_EN         UINT8_C(0x04)
/* Auto low power entered on timeout of threshold value
 * but reset on activity detection
 */
#define DA270_AUTO_LP_TIME_RESET_EN      UINT8_C(0x08)
/**\name    TAP INTERRUPT CONFIG MACROS   */
/* Axes select for TAP interrupt */
#define DA270_X_AXIS_EN_TAP   UINT8_C(0x02)
#define DA270_Y_AXIS_EN_TAP   UINT8_C(0x01)
#define DA270_Z_AXIS_EN_TAP   UINT8_C(0x00)
/*! TAP tics_th setting */
/* Maximum time between upper and lower peak of a tap, in data samples
 * this time depends on the mechanics of the device tapped onto
 * default = 12 samples
 */
/* Configures 6 data samples for high-low tap signal change time */
#define DA270_TICS_TH_6_DATA_SAMPLES  UINT8_C(0x00)
/* Configures 9 data samples for high-low tap signal change time */
#define DA270_TICS_TH_9_DATA_SAMPLES  UINT8_C(0x01)
/* Configures 12 data samples for high-low tap signal change time */
#define DA270_TICS_TH_12_DATA_SAMPLES UINT8_C(0x02)
/* Configures 18 data samples for high-low tap signal change time */
#define DA270_TICS_TH_18_DATA_SAMPLES UINT8_C(0x03)
/*! TAP Sensitivity setting */
/* It modifies the threshold for minimum TAP amplitude */
/* DA270_TAP_SENSITIVITY_0 correspond to highest sensitivity */
#define DA270_TAP_SENSITIVITY_0  UINT8_C(0x00)
#define DA270_TAP_SENSITIVITY_1  UINT8_C(0x01)
#define DA270_TAP_SENSITIVITY_2  UINT8_C(0x02)
#define DA270_TAP_SENSITIVITY_3  UINT8_C(0x03)
#define DA270_TAP_SENSITIVITY_4  UINT8_C(0x04)
#define DA270_TAP_SENSITIVITY_5  UINT8_C(0x05)
#define DA270_TAP_SENSITIVITY_6  UINT8_C(0x06)
/* DA270_TAP_SENSITIVITY_7 correspond to lowest sensitivity */
#define DA270_TAP_SENSITIVITY_7  UINT8_C(0x07)
/*!  DA270 TAP - quiet  settings */
/* Quiet refers to minimum quiet time before and after double tap,
 * in the data samples This time also defines the longest time interval
 * between two taps so that they are considered as double tap
 */
/* Configures 60 data samples quiet time between single or double taps */
#define DA270_QUIET_60_DATA_SAMPLES   UINT8_C(0x00)
/* Configures 80 data samples quiet time between single or double taps */
#define DA270_QUIET_80_DATA_SAMPLES   UINT8_C(0x01)
/* Configures 100 data samples quiet time between single or double taps */
#define DA270_QUIET_100_DATA_SAMPLES  UINT8_C(0x02)
/* Configures 120 data samples quiet time between single or double taps */
#define DA270_QUIET_120_DATA_SAMPLES  UINT8_C(0x03)
/*!  DA270 TAP - quiet_dt  settings */
/* quiet_dt refers to Minimum time between the two taps of a
 * double tap, in data samples
 */
/* Configures 4 data samples minimum time between double taps */
#define DA270_QUIET_DT_4_DATA_SAMPLES   UINT8_C(0x00)
/* Configures 8 data samples minimum time between double taps */
#define DA270_QUIET_DT_8_DATA_SAMPLES   UINT8_C(0x01)
/* Configures 12 data samples minimum time between double taps */
#define DA270_QUIET_DT_12_DATA_SAMPLES  UINT8_C(0x02)
/* Configures 16 data samples minimum time between double taps */
#define DA270_QUIET_DT_16_DATA_SAMPLES  UINT8_C(0x03)
/**\name	ACTIVITY CHANGE CONFIG MACROS	*/

/* Number of samples to evaluate for activity change detection */
#define DA270_ACT_CH_SAMPLE_CNT_32    UINT8_C(0x00)
#define DA270_ACT_CH_SAMPLE_CNT_64    UINT8_C(0x01)
#define DA270_ACT_CH_SAMPLE_CNT_128   UINT8_C(0x02)
#define DA270_ACT_CH_SAMPLE_CNT_256   UINT8_C(0x03)
#define DA270_ACT_CH_SAMPLE_CNT_512   UINT8_C(0x04)
/**\name Interrupt pin configuration macros */
#define DA270_INT_PUSH_PULL_ACTIVE_0    UINT8_C(0x00)
#define DA270_INT_PUSH_PULL_ACTIVE_1    UINT8_C(0x01)
#define DA270_INT_OPEN_DRIVE_ACTIVE_0   UINT8_C(0x02)
#define DA270_INT_OPEN_DRIVE_ACTIVE_1   UINT8_C(0x03)
/**\name Interrupt Assertion status macros */
#define DA270_WAKEUP_INT_ASSERTED      UINT16_C(0x0001)
#define DA270_ORIENT_CH_INT_ASSERTED   UINT16_C(0x0002)
#define DA270_GEN1_INT_ASSERTED        UINT16_C(0x0004)
#define DA270_GEN2_INT_ASSERTED        UINT16_C(0x0008)
#define DA270_INT_OVERRUN_ASSERTED     UINT16_C(0x0010)
#define DA270_DRDY_INT_ASSERTED        UINT16_C(0x0080)
#define DA270_STEP_INT_ASSERTED        UINT16_C(0x0300)
#define DA270_S_TAP_INT_ASSERTED       UINT16_C(0x0400)
#define DA270_D_TAP_INT_ASSERTED       UINT16_C(0x0800)
#define DA270_ACT_CH_X_ASSERTED        UINT16_C(0x2000)
#define DA270_ACT_CH_Y_ASSERTED        UINT16_C(0x4000)
#define DA270_ACT_CH_Z_ASSERTED        UINT16_C(0x8000)
/**\name Generic interrupt criterion_sel configuration macros */
#define DA270_ACTIVITY_INT             UINT8_C(0x01)
#define DA270_INACTIVITY_INT           UINT8_C(0x00)
/**\name Generic interrupt axes evaluation logic configuration macros */
#define DA270_ALL_AXES_INT             UINT8_C(0x01)
#define DA270_ANY_AXES_INT             UINT8_C(0x00)
/**\name Generic interrupt hysteresis configuration macros */
#define DA270_HYST_0_MG     UINT8_C(0x00)
#define DA270_HYST_24_MG    UINT8_C(0x01)
#define DA270_HYST_48_MG    UINT8_C(0x02)
#define DA270_HYST_96_MG    UINT8_C(0x03)
/**********************************************************************/
/**\name DA270 Register Address */
#define DA270_CHIP_ID_ADDR              UINT8_C(0x01)
#define DA270_STATUS_ADDR               UINT8_C(0x03)
#define DA270_ACCEL_DATA_ADDR           UINT8_C(0x02)
#define DA270_INT_STAT0_ADDR            UINT8_C(0x0E)
#define DA270_TEMP_DATA_ADDR            UINT8_C(0x11)
#define DA270_STEP_CNT_0_ADDR           UINT8_C(0x15)
#define DA270_ACCEL_MODE_BW             UINT8_C(0x11)
#define DA270_ACCEL_RESOLUTION_RANGE    UINT8_C(0x0F)
#define DA270_ACCEL_CONFIG_1_ADDR       UINT8_C(0x10)
#define DA270_ACCEL_CONFIG_2_ADDR       UINT8_C(0x11)
#define DA270_INT_CONF_0_ADDR           UINT8_C(0x1F)
#define DA270_INT_12_IO_CTRL_ADDR       UINT8_C(0x24)
#define DA270_INT_MAP1_ADDR             UINT8_C(0x19)
#define DA270_INT_MAP2_ADDR             UINT8_C(0x1A)
#define DA270_INT_MAP3_ADDR             UINT8_C(0x1B)
#define DA270_AUTO_LOW_POW_0_ADDR       UINT8_C(0x2A)
#define DA270_AUTO_LOW_POW_1_ADDR       UINT8_C(0x2B)
#define DA270_AUTOWAKEUP_0_ADDR         UINT8_C(0x2C)
#define DA270_AUTOWAKEUP_1_ADDR         UINT8_C(0x2D)
#define DA270_WAKEUP_INT_CONF_0_ADDR    UINT8_C(0x2F)
#define DA270_ORIENTCH_INT_CONFIG_ADDR  UINT8_C(0x35)
#define DA270_GEN1_INT_CONFIG_ADDR      UINT8_C(0x3F)
#define DA270_GEN2_INT_CONFIG_ADDR      UINT8_C(0x4A)
#define DA270_ACT_CH_CONFIG_0_ADDR      UINT8_C(0x55)
#define DA270_TAP_CONFIG_ADDR           UINT8_C(0x57)
#define DA270_SELF_TEST_ADDR            UINT8_C(0x7D)
#define DA270_SPI_CONFIG                UINT8_C(0x00)
/**\name DA270 Command register */
#define DA270_SOFT_RESET_CMD            UINT8_C(0x24)

#define DA270_SOFT_RESET_CMD_MSK       DA270_SOFT_RESET_CMD
/**\name DA270 Delay definitions */
#define DA270_SOFT_RESET_DELAY_MS       UINT8_C(10)
#define DA270_SELF_TEST_DELAY_MS        UINT8_C(7)
#define DA270_SELF_TEST_DATA_READ_MS    UINT8_C(50)
/**\name Interface selection macro */
#define DA270_SPI_WR_MASK    UINT8_C(0x7F)
#define DA270_SPI_RD_MASK    UINT8_C(0x80)
/**\name UTILITY MACROS	*/
#define DA270_SET_LOW_BYTE     UINT16_C(0x00FF)
#define DA270_SET_HIGH_BYTE    UINT16_C(0xFF00)
/**\name Interrupt mapping selection */
#define DA270_DATA_READY_INT_MAP     UINT8_C(0x01)
#define DA270_FIFO_WM_INT_MAP        UINT8_C(0x02)
#define DA270_FIFO_FULL_INT_MAP      UINT8_C(0x03)
#define DA270_GEN2_INT_MAP           UINT8_C(0x04)
#define DA270_GEN1_INT_MAP           UINT8_C(0x05)
#define DA270_ORIENT_CH_INT_MAP      UINT8_C(0x06)
#define DA270_WAKEUP_INT_MAP         UINT8_C(0x07)
#define DA270_ACT_CH_INT_MAP         UINT8_C(0x08)
#define DA270_TAP_INT_MAP            UINT8_C(0x09)
#define DA270_STEP_INT_MAP           UINT8_C(0x0A)
#define DA270_INT_OVERRUN_MAP        UINT8_C(0x0B)
/**\name DA270 FIFO configurations */
#define DA270_FIFO_AUTO_FLUSH          UINT8_C(0x01)
#define DA270_FIFO_STOP_ON_FULL        UINT8_C(0x02)
#define DA270_FIFO_TIME_EN             UINT8_C(0x04)
#define DA270_FIFO_8_BIT_EN            UINT8_C(0x10)
#define DA270_FIFO_X_EN                UINT8_C(0x20)
#define DA270_FIFO_Y_EN                UINT8_C(0x40)
#define DA270_FIFO_Z_EN                UINT8_C(0x80)
/**\name DA270 FIFO data configurations */
#define DA270_FIFO_EN_X         UINT8_C(0x01)
#define DA270_FIFO_EN_Y         UINT8_C(0x02)
#define DA270_FIFO_EN_Z         UINT8_C(0x04)
#define DA270_FIFO_EN_XY        UINT8_C(0x03)
#define DA270_FIFO_EN_YZ        UINT8_C(0x06)
#define DA270_FIFO_EN_XZ        UINT8_C(0x05)
#define DA270_FIFO_EN_XYZ       UINT8_C(0x07)
/**\name DA270 Self test configurations */
#define DA270_DISABLE_SELF_TEST          UINT8_C(0x00)
#define DA270_ENABLE_POSITIVE_SELF_TEST  UINT8_C(0x07)
#define DA270_ENABLE_NEGATIVE_SELF_TEST  UINT8_C(0x0F)
/**\name DA270 FIFO data masks */
#define DA270_FIFO_HEADER_MASK        UINT8_C(0x3E)
#define DA270_FIFO_BYTES_OVERREAD     UINT8_C(25)
#define DA270_AWIDTH_MASK             UINT8_C(0xEF)
#define DA270_FIFO_DATA_EN_MASK       UINT8_C(0x0E)
/**\name DA270 Step status field - Activity status */
#define DA270_STILL_ACT     UINT8_C(0x00)
#define DA270_WALK_ACT      UINT8_C(0x01)
#define DA270_RUN_ACT       UINT8_C(0x02)

/*! It is inserted when ACC_CONFIG0.bw
 * is changed during the FIFO read
  */
#define DA270_ACCEL_CONF0_CHANGE  UINT8_C(0x02)
/*! It is inserted when ACC_CONFIG1.acc_range
 * acc_odr is changed during the FIFO read
 */
#define DA270_ACCEL_CONF1_CHANGE  UINT8_C(0x04)
/*! Accel width setting either 12/8 bit mode */
#define DA270_12_BIT_FIFO_DATA       UINT8_C(0x01)
#define DA270_8_BIT_FIFO_DATA        UINT8_C(0x00)
/**\name DA270 FIFO header configurations */
#define DA270_FIFO_SENSOR_TIME     UINT8_C(0xA0)
#define DA270_FIFO_EMPTY_FRAME     UINT8_C(0x80)
#define DA270_FIFO_CONTROL_FRAME   UINT8_C(0x48)
#define DA270_FIFO_XYZ_ENABLE      UINT8_C(0x8E)
#define DA270_FIFO_X_ENABLE        UINT8_C(0x82)
#define DA270_FIFO_Y_ENABLE        UINT8_C(0x84)
#define DA270_FIFO_Z_ENABLE        UINT8_C(0x88)
#define DA270_FIFO_XY_ENABLE       UINT8_C(0x86)
#define DA270_FIFO_YZ_ENABLE       UINT8_C(0x8C)
#define DA270_FIFO_XZ_ENABLE       UINT8_C(0x8A)
/**\name DA270 bit mask definitions */
#define DA270_POWER_MODE_STATUS_MSK         UINT8_C(0x06)
#define DA270_POWER_MODE_STATUS_POS         UINT8_C(1)

#define DA270_POWER_MODE_MSK      UINT8_C(0x80)
#define DA270_POWER_MODE_POS      UINT8_C(7)

#define DA270_ACCEL_ODR_MSK      UINT8_C(0x0F)

#define DA270_ACCEL_RANGE_MSK    UINT8_C(0x03)
#define DA270_ACCEL_RANGE_POS    UINT8_C(0)

#define DA270_DATA_FILTER_MSK   UINT8_C(0x0C)
#define DA270_DATA_FILTER_POS   UINT8_C(2)

#define DA270_BW_MSK            UINT8_C(0x06)
#define DA270_BW_POS            UINT8_C(1)

#define DA270_WAKEUP_TIMEOUT_MSK       UINT8_C(0x04)
#define DA270_WAKEUP_TIMEOUT_POS       UINT8_C(2)

#define DA270_WAKEUP_THRES_LSB_MSK     UINT16_C(0x000F)

#define DA270_WAKEUP_THRES_MSB_MSK     UINT16_C(0x0FF0)
#define DA270_WAKEUP_THRES_MSB_POS     UINT8_C(4)

#define DA270_WAKEUP_TIMEOUT_THRES_MSK UINT8_C(0xF0)
#define DA270_WAKEUP_TIMEOUT_THRES_POS UINT8_C(4)

#define DA270_WAKEUP_INTERRUPT_MSK     UINT8_C(0x02)
#define DA270_WAKEUP_INTERRUPT_POS     UINT8_C(1)

#define DA270_AUTO_LOW_POW_MSK       UINT8_C(0x0F)

#define DA270_AUTO_LP_THRES_MSK      UINT16_C(0x0FF0)
#define DA270_AUTO_LP_THRES_POS      UINT8_C(4)

#define DA270_AUTO_LP_THRES_LSB_MSK  UINT16_C(0x000F)

#define DA270_WKUP_REF_UPDATE_MSK    UINT8_C(0x03)

#define DA270_AUTO_LP_TIMEOUT_LSB_MSK UINT8_C(0xF0)
#define DA270_AUTO_LP_TIMEOUT_LSB_POS UINT8_C(4)

#define DA270_SAMPLE_COUNT_MSK        UINT8_C(0x1C)
#define DA270_SAMPLE_COUNT_POS        UINT8_C(2)

#define DA270_WAKEUP_EN_AXES_MSK      UINT8_C(0xE0)
#define DA270_WAKEUP_EN_AXES_POS      UINT8_C(5)

#define DA270_TAP_AXES_EN_MSK         UINT8_C(0x18)
#define DA270_TAP_AXES_EN_POS         UINT8_C(3)

#define DA270_TAP_QUIET_DT_MSK         UINT8_C(0x30)
#define DA270_TAP_QUIET_DT_POS         UINT8_C(4)

#define DA270_TAP_QUIET_MSK            UINT8_C(0x0C)
#define DA270_TAP_QUIET_POS            UINT8_C(2)

#define DA270_TAP_TICS_TH_MSK         UINT8_C(0x03)

#define DA270_TAP_SENSITIVITY_MSK     UINT8_C(0X07)

#define DA270_ACT_CH_AXES_EN_MSK      UINT8_C(0xE0)
#define DA270_ACT_CH_AXES_EN_POS      UINT8_C(5)

#define DA270_ACT_CH_NPTS_MSK         UINT8_C(0x0F)

#define DA270_INT_AXES_EN_MSK    UINT8_C(0xE0)
#define DA270_INT_AXES_EN_POS    UINT8_C(5)

#define DA270_INT_REFU_MSK       UINT8_C(0x0C)
#define DA270_INT_REFU_POS       UINT8_C(2)

#define DA270_INT_HYST_MSK         UINT8_C(0x03)
#define DA270_STABILITY_MODE_MSK   UINT8_C(0x03)

#define DA270_GEN_INT_COMB_MSK      UINT8_C(0x01)

#define DA270_GEN_INT_CRITERION_MSK  UINT8_C(0x02)
#define DA270_GEN_INT_CRITERION_POS  UINT8_C(0x01)

#define DA270_INT_PIN1_CONF_MSK   UINT8_C(0x06)
#define DA270_INT_PIN1_CONF_POS   UINT8_C(1)

#define DA270_INT_PIN2_CONF_MSK   UINT8_C(0x60)
#define DA270_INT_PIN2_CONF_POS   UINT8_C(5)

#define DA270_INT_STATUS_MSK     UINT8_C(0xE0)
#define DA270_INT_STATUS_POS     UINT8_C(5)

#define DA270_EN_DRDY_MSK  UINT8_C(0x80)
#define DA270_EN_DRDY_POS  UINT8_C(7)

#define DA270_EN_FIFO_WM_MSK  UINT8_C(0x40)
#define DA270_EN_FIFO_WM_POS  UINT8_C(6)

#define DA270_EN_FIFO_FULL_MSK  UINT8_C(0x20)
#define DA270_EN_FIFO_FULL_POS  UINT8_C(5)

#define DA270_EN_INT_OVERRUN_MSK  UINT8_C(0x10)
#define DA270_EN_INT_OVERRUN_POS  UINT8_C(4)

#define DA270_EN_GEN2_MSK  UINT8_C(0x08)
#define DA270_EN_GEN2_POS  UINT8_C(3)

#define DA270_EN_GEN1_MSK  UINT8_C(0x04)
#define DA270_EN_GEN1_POS  UINT8_C(2)

#define DA270_EN_ORIENT_CH_MSK  UINT8_C(0x02)
#define DA270_EN_ORIENT_CH_POS  UINT8_C(1)

#define DA270_EN_LATCH_MSK  UINT8_C(0x80)
#define DA270_EN_LATCH_POS  UINT8_C(7)

#define DA270_EN_ACTCH_MSK  UINT8_C(0x10)
#define DA270_EN_ACTCH_POS  UINT8_C(4)

#define DA270_EN_D_TAP_MSK  UINT8_C(0x08)
#define DA270_EN_D_TAP_POS  UINT8_C(3)

#define DA270_EN_S_TAP_MSK  UINT8_C(0x04)
#define DA270_EN_S_TAP_POS  UINT8_C(2)

#define DA270_EN_STEP_INT_MSK  UINT8_C(0x01)

#define DA270_STEP_MAP_INT2_MSK  UINT8_C(0x10)
#define DA270_STEP_MAP_INT2_POS  UINT8_C(4)

#define DA270_EN_WAKEUP_INT_MSK  UINT8_C(0x01)

#define DA270_TAP_MAP_INT1_MSK  UINT8_C(0x04)
#define DA270_TAP_MAP_INT1_POS  UINT8_C(2)

#define DA270_TAP_MAP_INT2_MSK  UINT8_C(0x40)
#define DA270_TAP_MAP_INT2_POS  UINT8_C(6)

#define DA270_ACTCH_MAP_INT1_MSK  UINT8_C(0x08)
#define DA270_ACTCH_MAP_INT1_POS  UINT8_C(3)

#define DA270_ACTCH_MAP_INT2_MSK  UINT8_C(0x80)
#define DA270_ACTCH_MAP_INT2_POS  UINT8_C(7)

#define DA270_FIFO_BYTES_CNT_MSK  UINT8_C(0x07)

#define DA270_FIFO_TIME_EN_MSK  UINT8_C(0x04)
#define DA270_FIFO_TIME_EN_POS  UINT8_C(2)

#define DA270_FIFO_AXES_EN_MSK  UINT8_C(0xE0)
#define DA270_FIFO_AXES_EN_POS  UINT8_C(5)

#define DA270_FIFO_8_BIT_EN_MSK  UINT8_C(0x10)
#define DA270_FIFO_8_BIT_EN_POS  UINT8_C(4)
/**\name Macro to SET and GET BITS of a register */
#define DA270_SET_BITS(reg_data, bitname, data) \
	((reg_data & ~(bitname ## _MSK)) |	 \
	 ((data << bitname ## _POS) & bitname ## _MSK))

#define DA270_GET_BITS(reg_data, bitname)  ((reg_data & (bitname ## _MSK)) >> \
					     (bitname ## _POS))

#define DA270_SET_BITS_POS_0(reg_data, bitname, data) \
	((reg_data & ~(bitname ## _MSK)) |	       \
	 (data & bitname ## _MSK))

#define DA270_GET_BITS_POS_0(reg_data, bitname)  (reg_data & (bitname ## _MSK))

#define DA270_SET_BIT_VAL_0(reg_data, bitname) (reg_data & ~(bitname ## _MSK))

#define DA270_GET_LSB(var)     (uint8_t)(var & DA270_SET_LOW_BYTE)
#define DA270_GET_MSB(var)     (uint8_t)((var & DA270_SET_HIGH_BYTE) >> 8)
/********************************************************/
/*!
 * @brief Interface selection enums
 */
enum da270_intf {
	/*! SPI interface */
	DA270_SPI_INTF,
	/*! I2C interface */
	DA270_I2C_INTF
};
/********************************************************/
/**\name	TYPE DEFINITIONS */
/*!
 * @brief Bus communication function pointer which should be mapped to
 * the platform specific read and write functions of the user
 */
typedef int8_t (*da270_com_fptr_t)(void *intf_ptr, uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data,
				    uint16_t length);
/*!	Delay function pointer */
typedef void (*da270_delay_fptr_t)(uint32_t period);
/********************************************************/
/**\name	STRUCTURE DEFINITIONS*/
/*!
 * @brief Sensor selection enums
 */
enum da270_sensor {
	DA270_ACCEL,
	DA270_TAP_INT,
	DA270_ACTIVITY_CHANGE_INT,
	DA270_GEN1_INT,
	DA270_GEN2_INT,
	DA270_ORIENT_CHANGE_INT,
	DA270_STEP_COUNTER_INT
};

/*!
 * @brief Interrupt channel selection enums
 */
enum da270_int_chan {
	DA270_UNMAP_INT_PIN,
	DA270_INT_CHANNEL_1,
	DA270_INT_CHANNEL_2,
	DA270_MAP_BOTH_INT_PINS
};
/*!
 * @brief Interrupt pin hardware configurations
 */
struct da270_int_pin_conf {
	/*! Interrupt channel selection enums */
	enum da270_int_chan int_chan;
	/*! Interrupt pin configuration
	 * Assignable Macros :
	 *  - DA270_INT_PUSH_PULL_ACTIVE_0
	 *  - DA270_INT_PUSH_PULL_ACTIVE_1
	 *  - DA270_INT_OPEN_DRIVE_ACTIVE_0
	 *  - DA270_INT_OPEN_DRIVE_ACTIVE_1
	 */
	uint8_t pin_conf;
};
/*!
 * @brief Accel basic configuration
 */
struct da270_acc_conf {
	/*! Output data rate
	 * Assignable macros :
	 *                      - DA270_ODR_1_HZ   - DA270_ODR_1_95HZ
	 *  - DA270_ODR_3_9HZ   - DA270_ODR_7_81HZ  - DA270_ODR_15_63HZ
	 *  - DA270_ODR_31_25HZ - DA270_ODR_62_5HZ  - DA270_ODR_125HZ   
	 *  - DA270_ODR_250HZ   - DA270_ODR_500HZ   - DA270_ODR_1000HZ
	 */
	uint8_t odr;
	/*! Range of sensor
	 * Assignable macros :
	 *  - DA270_2G_RANGE   - DA270_8G_RANGE
	 *  - DA270_4G_RANGE   - DA270_16G_RANGE
	 */
	uint8_t range;

	/*! Filter 1 Bandwidth
	 * Assignable macros :
	 *  - DA270_ACCEL_FILT1_BW_0
	 *  - DA270_ACCEL_FILT1_BW_1
	 */
	uint8_t bw;
};
/*!
 * @brief Union of sensor Configurations
 */
union da270_set_param {
	/* Accel configurations */
	struct da270_acc_conf accel;
};
/*!
 * @brief Sensor selection and their configurations
 */
struct da270_sensor_conf {
	/* Sensor selection */
	enum da270_sensor type;
	/* Sensor configuration */
	union da270_set_param param;
};
/*!
 * @brief enum to select device settings
 */
enum da270_device {
	DA270_AUTOWAKEUP_TIMEOUT,
	DA270_AUTOWAKEUP_INT,
	DA270_AUTO_LOW_POWER,
	DA270_INT_PIN_CONF,
	DA270_INT_OVERRUN_CONF,
	DA270_FIFO_CONF
};

/*!
 * @brief DA270 auto-wakeup configurations
 */
struct da270_auto_wakeup_conf {
	/*! Enable auto wake-up by using timeout threshold
	 * Assignable Macros :
	 *   - DA270_ENABLE    - DA270_DISABLE
	 */
	uint8_t wakeup_timeout;
	/*! Timeout threshold after which auto wake-up occurs
	 * It is 12bit value configurable at 2.5ms/LSB
	 * Maximum timeout is 10.24s (4096 * 2.5) for
	 * which the assignable macro is :
	 *      - DA270_AUTO_WAKEUP_TIMEOUT_MAX
	 */
	uint16_t timeout_thres;
};
/*!
 * @brief DA270 wakeup configurations
 */
struct da270_wakeup_conf {
	/*! Wakeup reference update
	 *  Assignable macros:
	 *   - DA270_MANUAL_UPDATE
	 *   - DA270_ONE_TIME_UPDATE
	 *   - DA270_EVERY_TIME_UPDATE
	 */
	uint8_t wakeup_ref_update;
	/*! Number of samples for interrupt condition evaluation
	 * Assignable Macros :
	 *  - DA270_SAMPLE_COUNT_1  - DA270_SAMPLE_COUNT_5
	 *  - DA270_SAMPLE_COUNT_2  - DA270_SAMPLE_COUNT_6
	 *  - DA270_SAMPLE_COUNT_3  - DA270_SAMPLE_COUNT_7
	 *  - DA270_SAMPLE_COUNT_4  - DA270_SAMPLE_COUNT_8
	 */
	uint8_t sample_count;
	/*! Enable low power wake-up interrupt for X(BIT 0), Y(BIT 1), Z(BIT 2)
	 * axes  0 - not active; 1 - active
	 * Assignable macros :
	 *  - DA270_X_AXIS_EN
	 *  - DA270_Y_AXIS_EN
	 *  - DA270_Z_AXIS_EN
	 *  - DA270_XYZ_AXIS_EN
	 */
	uint8_t wakeup_axes_en;
	/*! Interrupt threshold configuration  */
	uint8_t int_wkup_threshold;
	/*! Reference acceleration x-axis for the wake-up interrupt */
	uint8_t int_wkup_ref_x;
	/*! Reference acceleration y-axis for the wake-up interrupt */
	uint8_t int_wkup_ref_y;
	/*! Reference acceleration z-axis for the wake-up interrupt */
	uint8_t int_wkup_ref_z;
	/*! Interrupt channel to be mapped */
	enum da270_int_chan int_chan;
};
/*!
 * @brief DA270 auto-low power configurations
 */
struct da270_auto_lp_conf {
	/*! Enable auto low power mode using  data ready interrupt /
	 * Genric interrupt1 / timeout counter value
	 * Assignable macros :
	 * - DA270_AUTO_LP_DRDY_TRIGGER
	 * - DA270_AUTO_LP_GEN1_TRIGGER
	 * - DA270_AUTO_LP_TIMEOUT_EN
	 * - DA270_AUTO_LP_TIME_RESET_EN
	 * - DA270_AUTO_LP_TIMEOUT_DISABLE
	 */
	uint8_t auto_low_power_trigger;
	/*! Timeout threshold after which auto wake-up occurs
	 * It is 12bit value configurable at 2.5ms/LSB
	 * Maximum timeout is 10.24s (4096 * 2.5) for
	 *  which the assignable macro is :
	 *  - DA270_AUTO_LP_TIMEOUT_MAX
	 */
	uint16_t auto_lp_timeout_threshold;
};

/*!
 * @brief Interrupt overrun configurations
 */
struct da270_int_overrun {
	/*! Interrupt pin mapping for interrupt overrun */
	enum da270_int_chan int_chan;
};
/*!
 * @brief Union of device configuration parameters
 */
union da270_device_params {
	/* Auto wakeup configurations */
	struct da270_auto_wakeup_conf auto_wakeup;
	/* Wakeup interrupt configurations */
	struct da270_wakeup_conf wakeup;
	/* Auto Low power configurations */
	struct da270_auto_lp_conf auto_lp;
	/* Interrupt pin configurations */
	struct da270_int_pin_conf int_conf;
	/* Interrupt overrun configuration */
	struct da270_int_overrun overrun_int;
};
/*!
 * @brief DA270 device configuration
 */
struct da270_device_conf {
	/* Device feature selection */
	enum da270_device type;
	/* Device feature configuration */
	union da270_device_params param;
};
/*!
 * @brief DA270 sensor data
 */
struct da270_sensor_data {
	/*! X-axis sensor data */
	int16_t x;
	/*! Y-axis sensor data */
	int16_t y;
	/*! Z-axis sensor data */
	int16_t z;
	/*! sensor time */
	uint32_t sensortime;
};
/*!
 * @brief DA270 interrupt selection
 */
enum da270_int_type {
	/* DRDY interrupt */
	DA270_DRDY_INT_EN,
	/* Generic interrupt 2 */
	DA270_GEN2_INT_EN,
	/* Generic interrupt 1 */
	DA270_GEN1_INT_EN,
	/* Orient change interrupt */
	DA270_ORIENT_CHANGE_INT_EN,
	/* Latch interrupt */
	DA270_LATCH_INT_EN,
	/* Activity change interrupt */
	DA270_ACTIVITY_CHANGE_INT_EN,
	/* Double tap interrupt */
	DA270_DOUBLE_TAP_INT_EN,
	/* Single tap interrupt */
	DA270_SINGLE_TAP_INT_EN,
	/* Step interrupt */
	DA270_STEP_COUNTER_INT_EN,
	/* Auto wakeup interrupt */
	DA270_AUTO_WAKEUP_EN
};
/*!
 * @brief Interrupt enable/disable configurations
 */
struct da270_int_enable {
	/*! Enum to choose the interrupt to be enabled */
	enum da270_int_type type;
	/*! Enable/ disable selected interrupts
	 * Assignable Macros :
	 *   - DA270_ENABLE
	 *   - DA270_DISABLE
	 */
	uint8_t conf;
};

/*!
 * @brief da270 device structure
 */
struct da270_dev {
	/*! Chip Id */
	uint8_t chip_id;
	/*! Device Id */
	uint8_t dev_id;
	/*! SPI/I2C Interface selection */
	enum da270_intf intf;
	/*! Interface handle pointer */
	void *intf_ptr;
	/*! Decide SPI or I2C read mechanism */
	uint8_t dummy_byte;
	/*! Bus read function pointer */
	da270_com_fptr_t read;
	/*! Bus write function pointer */
	da270_com_fptr_t write;
	/*! delay(in ms) function pointer */
	da270_delay_fptr_t delay_ms;
};

#endif /* DA270_DEFS_H_ */
/** @}*/
/** @}*/
