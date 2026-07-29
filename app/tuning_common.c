#include "tuning_common.h"

#include "control_config.h"

uint8_t TuningCommon_TextEquals(const char *left, const char *right)
{
    while ((*left != '\0') && (*right != '\0')) {
        if (*left != *right) {
            return 0U;
        }
        ++left;
        ++right;
    }
    return (uint8_t)((*left == '\0') && (*right == '\0'));
}

void TuningCommon_ToLower(char *text)
{
    while (*text != '\0') {
        if ((*text >= 'A') && (*text <= 'Z')) {
            *text = (char)(*text + ('a' - 'A'));
        }
        ++text;
    }
}

uint8_t TuningCommon_ParseInt32(const char *text, int32_t *value)
{
    uint32_t magnitude = 0U;
    uint32_t limit = 0x7FFFFFFFUL;
    uint8_t negative = 0U;
    uint8_t hasDigit = 0U;

    if ((text == 0) || (value == 0)) {
        return 0U;
    }
    if (*text == '-') {
        negative = 1U;
        limit = 0x80000000UL;
        ++text;
    } else if (*text == '+') {
        ++text;
    }
    while ((*text >= '0') && (*text <= '9')) {
        uint32_t digit = (uint32_t)(*text - '0');

        hasDigit = 1U;
        if (magnitude > ((limit - digit) / 10U)) {
            return 0U;
        }
        magnitude = magnitude * 10U + digit;
        ++text;
    }
    if ((hasDigit == 0U) || (*text != '\0')) {
        return 0U;
    }
    *value = (negative != 0U) ?
        ((magnitude == 0x80000000UL) ?
            (int32_t)0x80000000UL : -(int32_t)magnitude) :
        (int32_t)magnitude;
    return 1U;
}

uint8_t TuningCommon_Split(char *line, char *tokens[], uint8_t maxTokens)
{
    uint8_t count = 0U;

    while (*line != '\0') {
        while ((*line == ' ') || (*line == '\t')) {
            ++line;
        }
        if (*line == '\0') {
            break;
        }
        if (count >= maxTokens) {
            return (uint8_t)(maxTokens + 1U);
        }
        tokens[count++] = line;
        while ((*line != '\0') && (*line != ' ') && (*line != '\t')) {
            ++line;
        }
        if (*line != '\0') {
            *line++ = '\0';
        }
    }
    return count;
}

int16_t TuningCommon_PercentToPwm(int32_t percent)
{
    return (int16_t)((percent * (int32_t)CHASSIS_PWM_LIMIT_COUNTS) /
        TUNING_COMMON_PWM_PERCENT_MAX);
}

int32_t TuningCommon_PwmToPercent(int32_t pwm)
{
    int32_t scaled = pwm * TUNING_COMMON_PWM_PERCENT_MAX;
    int32_t half = (int32_t)CHASSIS_PWM_LIMIT_COUNTS / 2;

    if (scaled > 0) {
        scaled += half;
    } else if (scaled < 0) {
        scaled -= half;
    }
    return scaled / (int32_t)CHASSIS_PWM_LIMIT_COUNTS;
}
