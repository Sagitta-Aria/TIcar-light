#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} MotorId;

typedef enum {
    MOTOR_COAST = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE,
    MOTOR_BRAKE
} MotorDir;

void Motor_Init(void);
void Motor_Set(MotorId motor, MotorDir dir, uint16_t duty);
void Motor_SetSpeed(int16_t left, int16_t right);
void Motor_Stop(void);

#endif
