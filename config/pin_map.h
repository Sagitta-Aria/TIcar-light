#ifndef PIN_MAP_H
#define PIN_MAP_H

#include "ti_msp_dl_config.h"

/*
 * Pin source: 地猛星最小系统板 H3/H5 引脚排布 + ccs1.2 接线表。
 * 详细约束见 doc/PIN_ASSIGNMENT_2026-05-23.md。
 */

/*
 * 四个闭环步进驱动器使用 DIR/STEP 控制。
 * 这些宏只描述 MCU 到驱动器控制口的逻辑线，电机相线和驱动电源不接 MCU。
 */
#define PIN_STEPPER_CHASSIS_LEFT_STEP_PORT   STEPPER_CHASSIS_LEFT_STEP_PORT
#define PIN_STEPPER_CHASSIS_LEFT_STEP        STEPPER_CHASSIS_LEFT_STEP_PIN
#define PIN_STEPPER_CHASSIS_LEFT_DIR_PORT    STEPPER_CHASSIS_LEFT_DIR_PORT
#define PIN_STEPPER_CHASSIS_LEFT_DIR         STEPPER_CHASSIS_LEFT_DIR_PIN

#define PIN_STEPPER_CHASSIS_RIGHT_STEP_PORT  STEPPER_CHASSIS_RIGHT_STEP_PORT
#define PIN_STEPPER_CHASSIS_RIGHT_STEP       STEPPER_CHASSIS_RIGHT_STEP_PIN
#define PIN_STEPPER_CHASSIS_RIGHT_DIR_PORT   STEPPER_CHASSIS_RIGHT_DIR_PORT
#define PIN_STEPPER_CHASSIS_RIGHT_DIR        STEPPER_CHASSIS_RIGHT_DIR_PIN

#define PIN_STEPPER_GIMBAL_1_STEP_PORT       STEPPER_GIMBAL_1_STEP_PORT
#define PIN_STEPPER_GIMBAL_1_STEP            STEPPER_GIMBAL_1_STEP_PIN
#define PIN_STEPPER_GIMBAL_1_DIR_PORT        STEPPER_GIMBAL_1_DIR_PORT
#define PIN_STEPPER_GIMBAL_1_DIR             STEPPER_GIMBAL_1_DIR_PIN

#define PIN_STEPPER_GIMBAL_2_STEP_PORT       STEPPER_GIMBAL_2_STEP_PORT
#define PIN_STEPPER_GIMBAL_2_STEP            STEPPER_GIMBAL_2_STEP_PIN
#define PIN_STEPPER_GIMBAL_2_DIR_PORT        STEPPER_GIMBAL_2_DIR_PORT
#define PIN_STEPPER_GIMBAL_2_DIR             STEPPER_GIMBAL_2_DIR_PIN

#define PIN_KEY_PORT                    KEY_PORT
#define PIN_KEY_1                       KEY_1_PIN
#define PIN_KEY_2                       KEY_2_PIN

#define PIN_GRAY_ADC0                   GRAY_ADC0_INST
#define PIN_GRAY_ADC1                   GRAY_ADC1_INST

#define PIN_GRAY_DIGITAL_PORT           GPIOA
#define PIN_GRAY_1                      DL_GPIO_PIN_15
#define PIN_GRAY_2                      DL_GPIO_PIN_16
#define PIN_GRAY_3                      DL_GPIO_PIN_17
#define PIN_GRAY_4                      DL_GPIO_PIN_24
#define PIN_GRAY_5                      DL_GPIO_PIN_25
#define PIN_GRAY_6                      DL_GPIO_PIN_26
#define PIN_GRAY_7                      DL_GPIO_PIN_27
#define PIN_GRAY_1_IOMUX                GRAY_S1_IOMUX
#define PIN_GRAY_2_IOMUX                GRAY_S2_IOMUX
#define PIN_GRAY_3_IOMUX                GRAY_S3_IOMUX
#define PIN_GRAY_4_IOMUX                GRAY_S4_IOMUX
#define PIN_GRAY_5_IOMUX                GRAY_S5_IOMUX
#define PIN_GRAY_6_IOMUX                GRAY_S6_IOMUX
#define PIN_GRAY_7_IOMUX                GRAY_S7_IOMUX

#endif
