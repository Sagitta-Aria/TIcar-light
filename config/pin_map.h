#ifndef PIN_MAP_H
#define PIN_MAP_H

#include "ti_msp_dl_config.h"

/*
 * Pin source: D:\激光循迹\Netlist_Schematic1_2026-05-16.tel
 */

#define PIN_MOTOR_PWM_TIMER             PWM_INST
#define PIN_MOTOR_LEFT_PWM_CC           GPIO_PWM_C1_IDX
#define PIN_MOTOR_RIGHT_PWM_CC          GPIO_PWM_C0_IDX
#define PIN_MOTOR_DIR_PORT              MOTOR_DIR_PORT
#define PIN_MOTOR_LEFT_IN1              MOTOR_A_IN1_PIN
#define PIN_MOTOR_LEFT_IN2              MOTOR_A_IN2_PIN
#define PIN_MOTOR_RIGHT_IN1             MOTOR_B_IN1_PIN
#define PIN_MOTOR_RIGHT_IN2             MOTOR_B_IN2_PIN

#define PIN_ENCODER_PORT                ENCODER_PORT
#define PIN_ENCODER_LEFT_A              ENCODER_LEFT_A_PIN
#define PIN_ENCODER_LEFT_B              ENCODER_LEFT_B_PIN
#define PIN_ENCODER_RIGHT_A             ENCODER_RIGHT_A_PIN
#define PIN_ENCODER_RIGHT_B             ENCODER_RIGHT_B_PIN

#define PIN_KEY_PORT                    KEY_PORT
#define PIN_KEY_1                       KEY_1_PIN
#define PIN_KEY_2                       KEY_2_PIN

#define PIN_GRAY_ADC0                   GRAY_ADC0_INST
#define PIN_GRAY_ADC1                   GRAY_ADC1_INST

#endif
