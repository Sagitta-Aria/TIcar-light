#include "jy61p.h"

#include "ti_msp_dl_config.h"

/* JY61P 一帧固定 11 字节，角度帧类型为 0x53。 */
#define JY61P_FRAME_HEAD              (0x55U)
#define JY61P_FRAME_ANGLE             (0x53U)
#define JY61P_FRAME_LENGTH            (11U)
#define JY61P_CHECKSUM_LENGTH         (10U)

/* g_jy61pYawDeg：当前缓存的航向角。 */
static volatile int16_t g_jy61pYawDeg;

/* g_jy61pYawValid：是否已经写入过有效航向角。 */
static volatile uint8_t g_jy61pYawValid;

/* g_jy61pFrame：串口中断里拼出的当前 JY61P 数据帧。 */
static uint8_t g_jy61pFrame[JY61P_FRAME_LENGTH];

/* g_jy61pFrameIndex：当前已经接收到的数据帧位置。 */
static uint8_t g_jy61pFrameIndex;

/*
 * 作用：检查 JY61P 数据帧校验和。
 * 使用场景：串口收到 11 字节完整帧后，先确认数据可信再更新 yaw。
 */
static uint8_t JY61P_CheckFrame(const uint8_t *frame)
{
    uint8_t i;
    uint8_t checksum = 0U;

    for (i = 0U; i < JY61P_CHECKSUM_LENGTH; ++i) {
        checksum = (uint8_t)(checksum + frame[i]);
    }

    return (checksum == frame[JY61P_CHECKSUM_LENGTH]) ? 1U : 0U;
}

/*
 * 作用：把小端格式的 int16 原始值取出来。
 * 使用场景：解析 JY61P 角度帧里的 yaw 原始值。
 */
static int16_t JY61P_ReadInt16LE(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

/*
 * 作用：把 JY61P 角度原始值换算成整数角度。
 * 使用场景：路线外环判断是否已经转过 90 度。
 */
static int16_t JY61P_RawAngleToDeg(int16_t rawAngle)
{
    return (int16_t)(((int32_t)rawAngle * 180L) / 32768L);
}

/*
 * 作用：处理完整角度帧，提取 yaw 并写入缓存。
 * 使用场景：接收到 0x55 0x53 姿态角帧后调用。
 */
static void JY61P_HandleAngleFrame(const uint8_t *frame)
{
    int16_t rawYaw = JY61P_ReadInt16LE(frame[6], frame[7]);
    JY61P_SetYawDeg(JY61P_RawAngleToDeg(rawYaw));
}

/*
 * 作用：逐字节同步并解析 JY61P 数据帧。
 * 使用场景：UART0 接收中断每拿到 1 个字节就喂给它。
 */
static void JY61P_ParseByte(uint8_t data)
{
    if (g_jy61pFrameIndex == 0U) {
        if (data == JY61P_FRAME_HEAD) {
            g_jy61pFrame[0] = data;
            g_jy61pFrameIndex = 1U;
        }
        return;
    }

    if ((g_jy61pFrameIndex == 1U) && (data == JY61P_FRAME_HEAD)) {
        g_jy61pFrame[0] = data;
        return;
    }

    g_jy61pFrame[g_jy61pFrameIndex] = data;
    ++g_jy61pFrameIndex;

    if (g_jy61pFrameIndex >= JY61P_FRAME_LENGTH) {
        if ((g_jy61pFrame[1] == JY61P_FRAME_ANGLE) &&
            JY61P_CheckFrame(g_jy61pFrame)) {
            JY61P_HandleAngleFrame(g_jy61pFrame);
        }
        g_jy61pFrameIndex = 0U;
    }
}

void JY61P_Init(void)
{
    DL_UART_Main_setRXFIFOThreshold(
        JY61P_INST, DL_UART_MAIN_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_enableInterrupt(JY61P_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(JY61P_INST_INT_IRQN);
    NVIC_EnableIRQ(JY61P_INST_INT_IRQN);
    g_jy61pYawDeg = 0;
    g_jy61pYawValid = 0U;
    g_jy61pFrameIndex = 0U;
}

void JY61P_Task(void)
{
}

void JY61P_HandleUARTInterrupt(void)
{
    uint8_t data;
    DL_UART_IIDX pending;

    do {
        pending = DL_UART_Main_getPendingInterrupt(JY61P_INST);
        if (pending == DL_UART_MAIN_IIDX_RX) {
            while (DL_UART_Main_receiveDataCheck(JY61P_INST, &data)) {
                JY61P_ParseByte(data);
            }
        }
    } while (pending != DL_UART_MAIN_IIDX_NO_INTERRUPT);
}

void JY61P_SendByte(uint8_t data)
{
    DL_UART_transmitDataBlocking(JY61P_INST, data);
}

void JY61P_SendBytes(const uint8_t *data, uint16_t length)
{
    uint16_t i;

    if (data == 0) {
        return;
    }
    for (i = 0U; i < length; ++i) {
        JY61P_SendByte(data[i]);
    }
}

void JY61P_SetYawDeg(int16_t yawDeg)
{
    g_jy61pYawDeg = yawDeg;
    g_jy61pYawValid = 1U;
}

int16_t JY61P_GetYawDeg(void)
{
    return g_jy61pYawDeg;
}

uint8_t JY61P_HasYaw(void)
{
    return g_jy61pYawValid;
}
