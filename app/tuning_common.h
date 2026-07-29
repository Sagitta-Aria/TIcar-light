#ifndef TUNING_COMMON_H
#define TUNING_COMMON_H

#include <stdint.h>

#define TUNING_COMMON_PWM_PERCENT_MAX (100L)

/* Pure helpers shared by the Gmr and Full UART tuning consoles. */
uint8_t TuningCommon_TextEquals(const char *left, const char *right);
void TuningCommon_ToLower(char *text);
uint8_t TuningCommon_ParseInt32(const char *text, int32_t *value);
uint8_t TuningCommon_Split(char *line, char *tokens[], uint8_t maxTokens);
int16_t TuningCommon_PercentToPwm(int32_t percent);
int32_t TuningCommon_PwmToPercent(int32_t pwm);

#endif
