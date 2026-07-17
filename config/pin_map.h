#ifndef PIN_MAP_H
#define PIN_MAP_H

#include "ti_msp_dl_config.h"

/*
 * Pin source: 地猛星最小系统板 H3/H5 引脚排布 + ccs1.2 接线表。
 * 详细约束见 doc/PINOUT.md。
 */

/*
 * 底盘改为两路编码直流电机，云台仍为两路 STEP/DIR 步进电机。
 * PA31/PB19 已被底盘方向和编码器占用，云台 EN 必须在硬件上固定有效。
 */

/* 左轮：TB6612 A 通道，TIMA0 CCP1 输出 PWM。 */
#define PIN_CHASSIS_LEFT_PWM_PORT            GPIOA
#define PIN_CHASSIS_LEFT_PWM                 DL_GPIO_PIN_22
#define PIN_CHASSIS_LEFT_PWM_IOMUX           IOMUX_PINCM47
#define PIN_CHASSIS_LEFT_PWM_FUNC            IOMUX_PINCM47_PF_TIMA0_CCP1
#define PIN_CHASSIS_LEFT_PWM_CC_INDEX        DL_TIMER_CC_1_INDEX
#define PIN_CHASSIS_LEFT_IN1_PORT            GPIOA
#define PIN_CHASSIS_LEFT_IN1                 DL_GPIO_PIN_31
#define PIN_CHASSIS_LEFT_IN1_IOMUX           IOMUX_PINCM6
#define PIN_CHASSIS_LEFT_IN2_PORT            GPIOA
#define PIN_CHASSIS_LEFT_IN2                 DL_GPIO_PIN_28
#define PIN_CHASSIS_LEFT_IN2_IOMUX           IOMUX_PINCM3

/* 右轮：TB6612 B 通道，TIMA0 CCP3 输出 PWM。 */
#define PIN_CHASSIS_RIGHT_PWM_PORT           GPIOA
#define PIN_CHASSIS_RIGHT_PWM                DL_GPIO_PIN_12
#define PIN_CHASSIS_RIGHT_PWM_IOMUX          IOMUX_PINCM34
#define PIN_CHASSIS_RIGHT_PWM_FUNC           IOMUX_PINCM34_PF_TIMA0_CCP3
#define PIN_CHASSIS_RIGHT_PWM_CC_INDEX       DL_TIMER_CC_3_INDEX
#define PIN_CHASSIS_RIGHT_IN1_PORT           GPIOA
#define PIN_CHASSIS_RIGHT_IN1                DL_GPIO_PIN_21
#define PIN_CHASSIS_RIGHT_IN1_IOMUX          IOMUX_PINCM46
#define PIN_CHASSIS_RIGHT_IN2_PORT           GPIOA
#define PIN_CHASSIS_RIGHT_IN2                DL_GPIO_PIN_23
#define PIN_CHASSIS_RIGHT_IN2_IOMUX          IOMUX_PINCM53

/* 左右编码器 A/B 相输入，四路均使用 GPIO 双边沿中断。 */
#define PIN_CHASSIS_LEFT_ENCODER_A_PORT      GPIOB
#define PIN_CHASSIS_LEFT_ENCODER_A           DL_GPIO_PIN_19
#define PIN_CHASSIS_LEFT_ENCODER_A_IOMUX     IOMUX_PINCM45
#define PIN_CHASSIS_LEFT_ENCODER_B_PORT      GPIOB
#define PIN_CHASSIS_LEFT_ENCODER_B           DL_GPIO_PIN_20
#define PIN_CHASSIS_LEFT_ENCODER_B_IOMUX     IOMUX_PINCM48
#define PIN_CHASSIS_RIGHT_ENCODER_A_PORT     GPIOA
#define PIN_CHASSIS_RIGHT_ENCODER_A          DL_GPIO_PIN_13
#define PIN_CHASSIS_RIGHT_ENCODER_A_IOMUX    IOMUX_PINCM35
#define PIN_CHASSIS_RIGHT_ENCODER_B_PORT     GPIOB
#define PIN_CHASSIS_RIGHT_ENCODER_B          DL_GPIO_PIN_24
#define PIN_CHASSIS_RIGHT_ENCODER_B_IOMUX    IOMUX_PINCM52

/* 云台轴语义别名；右侧 STEPPER_GIMBAL_1/2 是 SysConfig 生成的硬件名。 */
#define PIN_STEPPER_GIMBAL_YAW_STEP_PORT     STEPPER_GIMBAL_1_STEP_PORT
#define PIN_STEPPER_GIMBAL_YAW_STEP          STEPPER_GIMBAL_1_STEP_PIN
#define PIN_STEPPER_GIMBAL_YAW_DIR_PORT      STEPPER_GIMBAL_1_DIR_PORT
#define PIN_STEPPER_GIMBAL_YAW_DIR           STEPPER_GIMBAL_1_DIR_PIN
#define PIN_STEPPER_GIMBAL_PITCH_STEP_PORT   STEPPER_GIMBAL_2_STEP_PORT
#define PIN_STEPPER_GIMBAL_PITCH_STEP        STEPPER_GIMBAL_2_STEP_PIN
#define PIN_STEPPER_GIMBAL_PITCH_DIR_PORT    STEPPER_GIMBAL_2_DIR_PORT
#define PIN_STEPPER_GIMBAL_PITCH_DIR         STEPPER_GIMBAL_2_DIR_PIN

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
