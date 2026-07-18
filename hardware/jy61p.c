#include "jy61p.h"

#include "FreeRTOS.h"
#include "task.h"

#include "board_config.h"
#include "log_uart.h"
#include "ti_msp_dl_config.h"

#define JY61P_FRAME_SIZE              (11U)
#define JY61P_FRAME_HEAD              (0x55U)
#define JY61P_FRAME_GYRO              (0x52U)
#define JY61P_FRAME_ANGLE             (0x53U)
#define JY61P_UART_SERVICE_LIMIT      (16U)
#define JY61P_UART_RX_DRAIN_LIMIT     (64U)
#define JY61P_PRINT_PERIOD_MS         (100U)
#define JY61P_PRINT_WAIT_MS           (1000U)
#define JY61P_PRINT_PERIOD_TICKS \
    ((JY61P_PRINT_PERIOD_MS + CAR_APP_LOOP_DELAY_MS - 1U) / \
        CAR_APP_LOOP_DELAY_MS)
#define JY61P_PRINT_WAIT_TICKS \
    ((JY61P_PRINT_WAIT_MS + CAR_APP_LOOP_DELAY_MS - 1U) / \
        CAR_APP_LOOP_DELAY_MS)

typedef struct {
    volatile int16_t rollX100;
    volatile int16_t pitchX100;
    volatile int16_t yawX100;
    volatile int32_t rollRateX100PerSec;
    volatile int32_t pitchRateX100PerSec;
    volatile int32_t yawRateX100PerSec;
    volatile uint32_t angleFrameCount;
    volatile uint32_t gyroFrameCount;
    volatile uint32_t angleFrameTick;
    volatile uint32_t gyroFrameTick;
    volatile uint32_t badFrameCount;
    uint8_t frame[JY61P_FRAME_SIZE];
    uint8_t frameIndex;
    uint16_t printTicks;
    uint16_t waitTicks;
    uint32_t lastPrintedFrameCount;
} JY61P_State;

static JY61P_State g_jy61p;

static uint32_t JY61P_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void JY61P_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static int16_t JY61P_ReadInt16(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

static int16_t JY61P_AngleRawToX100(int16_t raw)
{
    return (int16_t)(((int32_t)raw * 18000L) / 32768L);
}

static int32_t JY61P_GyroRawToX100PerSec(int16_t raw)
{
    /* JY61 0x52 满量程为正负 2000 deg/s；6250/1024 等于 200000/32768。 */
    return ((int32_t)raw * 6250L) / 1024L;
}

static uint8_t JY61P_FrameChecksumOk(const uint8_t frame[JY61P_FRAME_SIZE])
{
    uint8_t sum = 0U;
    uint8_t i;

    for (i = 0U; i < (uint8_t)(JY61P_FRAME_SIZE - 1U); ++i) {
        sum = (uint8_t)(sum + frame[i]);
    }
    return (uint8_t)(sum == frame[JY61P_FRAME_SIZE - 1U]);
}

static void JY61P_ApplyFrame(const uint8_t frame[JY61P_FRAME_SIZE])
{
    if (JY61P_FrameChecksumOk(frame) == 0U) {
        ++g_jy61p.badFrameCount;
        return;
    }

    if (frame[1] == JY61P_FRAME_GYRO) {
        g_jy61p.rollRateX100PerSec =
            JY61P_GyroRawToX100PerSec(JY61P_ReadInt16(&frame[2]));
        g_jy61p.pitchRateX100PerSec =
            JY61P_GyroRawToX100PerSec(JY61P_ReadInt16(&frame[4]));
        g_jy61p.yawRateX100PerSec =
            JY61P_GyroRawToX100PerSec(JY61P_ReadInt16(&frame[6]));
        g_jy61p.gyroFrameTick = (uint32_t)xTaskGetTickCountFromISR();
        ++g_jy61p.gyroFrameCount;
    } else if (frame[1] == JY61P_FRAME_ANGLE) {
        g_jy61p.rollX100 = JY61P_AngleRawToX100(JY61P_ReadInt16(&frame[2]));
        g_jy61p.pitchX100 = JY61P_AngleRawToX100(JY61P_ReadInt16(&frame[4]));
        g_jy61p.yawX100 = JY61P_AngleRawToX100(JY61P_ReadInt16(&frame[6]));
        g_jy61p.angleFrameTick = (uint32_t)xTaskGetTickCountFromISR();
        ++g_jy61p.angleFrameCount;
    }
}

static void JY61P_ParseByte(uint8_t data)
{
    if (g_jy61p.frameIndex == 0U) {
        if (data != JY61P_FRAME_HEAD) {
            return;
        }
        g_jy61p.frame[g_jy61p.frameIndex] = data;
        ++g_jy61p.frameIndex;
        return;
    }

    if ((g_jy61p.frameIndex == 1U) &&
        (data != JY61P_FRAME_GYRO) && (data != JY61P_FRAME_ANGLE)) {
        g_jy61p.frameIndex = 0U;
        if (data == JY61P_FRAME_HEAD) {
            g_jy61p.frame[0] = data;
            g_jy61p.frameIndex = 1U;
        }
        return;
    }

    g_jy61p.frame[g_jy61p.frameIndex] = data;
    ++g_jy61p.frameIndex;
    if (g_jy61p.frameIndex >= JY61P_FRAME_SIZE) {
        JY61P_ApplyFrame(g_jy61p.frame);
        g_jy61p.frameIndex = 0U;
    }
}

static void JY61P_PrintAngle(const char *label, int16_t valueX100)
{
    int32_t value = valueX100;
    uint32_t magnitude;
    uint32_t fraction;

    LogUart_SendString(label);
    if (value < 0) {
        LogUart_SendByte((uint8_t)'-');
        magnitude = (uint32_t)(-value);
    } else {
        magnitude = (uint32_t)value;
    }
    LogUart_SendUnsigned(magnitude / 100U);
    LogUart_SendByte((uint8_t)'.');
    fraction = magnitude % 100U;
    if (fraction < 10U) {
        LogUart_SendByte((uint8_t)'0');
    }
    LogUart_SendUnsigned(fraction);
}

void JY61P_Init(void)
{
    uint8_t i;

    g_jy61p.rollX100 = 0;
    g_jy61p.pitchX100 = 0;
    g_jy61p.yawX100 = 0;
    g_jy61p.rollRateX100PerSec = 0;
    g_jy61p.pitchRateX100PerSec = 0;
    g_jy61p.yawRateX100PerSec = 0;
    g_jy61p.angleFrameCount = 0U;
    g_jy61p.gyroFrameCount = 0U;
    g_jy61p.angleFrameTick = 0U;
    g_jy61p.gyroFrameTick = 0U;
    g_jy61p.badFrameCount = 0U;
    for (i = 0U; i < JY61P_FRAME_SIZE; ++i) {
        g_jy61p.frame[i] = 0U;
    }
    g_jy61p.frameIndex = 0U;
    g_jy61p.printTicks = 0U;
    g_jy61p.waitTicks = 0U;
    g_jy61p.lastPrintedFrameCount = 0U;

    DL_UART_Main_setRXFIFOThreshold(
        JY61P_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(JY61P_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
        DL_UART_MAIN_INTERRUPT_NOISE_ERROR);
    NVIC_ClearPendingIRQ(JY61P_INST_INT_IRQN);
    NVIC_EnableIRQ(JY61P_INST_INT_IRQN);
}

void JY61P_HandleUARTInterrupt(void)
{
    uint8_t data;
    DL_UART_IIDX pending;
    uint8_t serviceCount = 0U;
    uint8_t rxCount;

    do {
        pending = DL_UART_Main_getPendingInterrupt(JY61P_INST);
        if (pending == DL_UART_MAIN_IIDX_RX) {
            rxCount = 0U;
            while ((rxCount < JY61P_UART_RX_DRAIN_LIMIT) &&
                DL_UART_Main_receiveDataCheck(JY61P_INST, &data)) {
                JY61P_ParseByte(data);
                ++rxCount;
            }
        } else if ((pending == DL_UART_MAIN_IIDX_FRAMING_ERROR) ||
            (pending == DL_UART_MAIN_IIDX_NOISE_ERROR) ||
            (pending == DL_UART_MAIN_IIDX_OVERRUN_ERROR)) {
            ++g_jy61p.badFrameCount;
        }
        ++serviceCount;
    } while ((pending != DL_UART_MAIN_IIDX_NO_INTERRUPT) &&
        (serviceCount < JY61P_UART_SERVICE_LIMIT));
}

uint8_t JY61P_GetAttitude(JY61P_Attitude *attitude)
{
    uint32_t primask;

    if (attitude == 0) {
        return 0U;
    }

    primask = JY61P_EnterCritical();
    attitude->rollX100 = g_jy61p.rollX100;
    attitude->pitchX100 = g_jy61p.pitchX100;
    attitude->yawX100 = g_jy61p.yawX100;
    attitude->rollRateX100PerSec = g_jy61p.rollRateX100PerSec;
    attitude->pitchRateX100PerSec = g_jy61p.pitchRateX100PerSec;
    attitude->yawRateX100PerSec = g_jy61p.yawRateX100PerSec;
    attitude->angleFrameCount = g_jy61p.angleFrameCount;
    attitude->gyroFrameCount = g_jy61p.gyroFrameCount;
    attitude->angleFrameTick = g_jy61p.angleFrameTick;
    attitude->gyroFrameTick = g_jy61p.gyroFrameTick;
    attitude->badFrameCount = g_jy61p.badFrameCount;
    JY61P_ExitCritical(primask);

    return ((attitude->angleFrameCount != 0U) ||
        (attitude->gyroFrameCount != 0U)) ? 1U : 0U;
}

void JY61P_PrintTask(void)
{
    JY61P_Attitude attitude;

    if (g_jy61p.printTicks < (uint16_t)JY61P_PRINT_PERIOD_TICKS) {
        ++g_jy61p.printTicks;
        return;
    }
    g_jy61p.printTicks = 0U;

    if (JY61P_GetAttitude(&attitude) == 0U) {
        if (g_jy61p.waitTicks < (uint16_t)JY61P_PRINT_WAIT_TICKS) {
            ++g_jy61p.waitTicks;
            return;
        }
        g_jy61p.waitTicks = 0U;
        LOG_LINE("jy61p wait");
        return;
    }

    g_jy61p.waitTicks = 0U;
    if (attitude.angleFrameCount == g_jy61p.lastPrintedFrameCount) {
        return;
    }
    g_jy61p.lastPrintedFrameCount = attitude.angleFrameCount;

    LogUart_SendString("jy61p ");
    JY61P_PrintAngle("roll=", attitude.rollX100);
    JY61P_PrintAngle(" pitch=", attitude.pitchX100);
    JY61P_PrintAngle(" yaw=", attitude.yawX100);
    LogUart_SendString(" yaw_rate_x100_s=");
    LogUart_SendSigned(attitude.yawRateX100PerSec);
    LogUart_SendString(" frame=");
    LogUart_SendUnsigned(attitude.angleFrameCount);
    LogUart_SendString(" gyro=");
    LogUart_SendUnsigned(attitude.gyroFrameCount);
    LogUart_SendString(" bad=");
    LogUart_SendUnsigned(attitude.badFrameCount);
    LogUart_SendString("\r\n");
}
