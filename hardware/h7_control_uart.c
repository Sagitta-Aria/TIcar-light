#include "h7_control_uart.h"

#include "library_config.h"
#include "resource_config.h"

#if CAR_PROFILE_IS_GMR && CAR_H7_UART_REQUIRED

#include "pin_map.h"
#include "ti_msp_dl_config.h"

#define H7_CONTROL_TX_TIMEOUT_COUNT   (100000U)
#define H7_FORWARD_ACCEL_PERIOD_MS    (20U)
#define H7_FORWARD_ACCEL_COMMAND_SIZE (16U)
#define H7_STEPPER_COMMAND_SIZE       (32U)

typedef enum {
    H7_BALL_BALANCE_REQUEST_NONE = 0,
    H7_BALL_BALANCE_REQUEST_START_DEBUG,
    H7_BALL_BALANCE_REQUEST_START_TASK3,
    H7_BALL_BALANCE_REQUEST_START_TASK4,
    H7_BALL_BALANCE_REQUEST_START_TASK5,
    H7_BALL_BALANCE_REQUEST_START_TASK6,
    H7_BALL_BALANCE_REQUEST_START_IMU_Y_FF,
    H7_BALL_BALANCE_REQUEST_STOP
} H7BallBalanceRequest;

static volatile uint8_t g_h7ControlTxBusy;
static volatile H7BallBalanceRequest g_h7BallBalancePendingRequest;
static uint16_t g_h7ForwardAccelElapsedMs;
static volatile int32_t g_h7StepperPositionSteps;
static volatile uint16_t g_h7StepperSpeedRpm;
static volatile uint8_t g_h7StepperAcceleration;
static volatile uint8_t g_h7StepperCommandPending;

static const DL_UART_Main_ClockConfig g_h7ControlClockConfig = {
    .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config g_h7ControlConfig = {
    .mode = DL_UART_MAIN_MODE_NORMAL,
    .direction = DL_UART_MAIN_DIRECTION_TX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity = DL_UART_MAIN_PARITY_NONE,
    .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits = DL_UART_MAIN_STOP_BITS_ONE
};

static uint32_t H7ControlUart_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static void H7ControlUart_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static uint16_t H7ControlUart_FormatForwardAcceleration(
    int32_t accelerationX100,
    uint8_t command[H7_FORWARD_ACCEL_COMMAND_SIZE])
{
    uint8_t digits[10];
    uint32_t magnitude;
    uint16_t length = 0U;
    uint8_t digitCount = 0U;

    command[length++] = (uint8_t)'@';
    command[length++] = (uint8_t)'A';
    command[length++] = (uint8_t)'=';
    if (accelerationX100 < 0) {
        command[length++] = (uint8_t)'-';
        magnitude = (uint32_t)(-(accelerationX100 + 1)) + 1U;
    } else {
        magnitude = (uint32_t)accelerationX100;
    }
    do {
        digits[digitCount++] = (uint8_t)('0' + (magnitude % 10U));
        magnitude /= 10U;
    } while (magnitude != 0U);
    while (digitCount > 0U) {
        command[length++] = digits[--digitCount];
    }
    command[length++] = (uint8_t)'\n';
    return length;
}

static uint16_t H7ControlUart_AppendUnsigned(uint32_t value,
    uint8_t *command, uint16_t length)
{
    uint8_t digits[10];
    uint8_t digitCount = 0U;

    do {
        digits[digitCount++] = (uint8_t)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);
    while (digitCount > 0U) {
        command[length++] = digits[--digitCount];
    }
    return length;
}

static uint16_t H7ControlUart_FormatStepperMove(int32_t positionSteps,
    uint16_t speedRpm, uint8_t acceleration,
    uint8_t command[H7_STEPPER_COMMAND_SIZE])
{
    uint32_t magnitude;
    uint16_t length = 0U;

    command[length++] = (uint8_t)'@';
    command[length++] = (uint8_t)'S';
    command[length++] = (uint8_t)'T';
    command[length++] = (uint8_t)'E';
    command[length++] = (uint8_t)'P';
    command[length++] = (uint8_t)'=';
    if (positionSteps < 0) {
        command[length++] = (uint8_t)'-';
        magnitude = (uint32_t)(-(positionSteps + 1)) + 1U;
    } else {
        magnitude = (uint32_t)positionSteps;
    }
    length = H7ControlUart_AppendUnsigned(magnitude, command, length);
    command[length++] = (uint8_t)',';
    length = H7ControlUart_AppendUnsigned(speedRpm, command, length);
    command[length++] = (uint8_t)',';
    length = H7ControlUart_AppendUnsigned(acceleration, command, length);
    command[length++] = (uint8_t)'\n';
    return length;
}

void H7ControlUart_Init(void)
{
    g_h7ControlTxBusy = 0U;
    g_h7BallBalancePendingRequest = H7_BALL_BALANCE_REQUEST_NONE;
    g_h7ForwardAccelElapsedMs = 0U;
    g_h7StepperPositionSteps = 0L;
    g_h7StepperSpeedRpm = 1U;
    g_h7StepperAcceleration = 0U;
    g_h7StepperCommandPending = 0U;

    DL_UART_Main_reset(CAR_H7_UART_INST);
    DL_UART_Main_enablePower(CAR_H7_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);

    DL_GPIO_initPeripheralOutputFunction(PIN_H7_CONTROL_UART_TX_IOMUX,
        PIN_H7_CONTROL_UART_TX_FUNC);

    DL_UART_Main_setClockConfig(CAR_H7_UART_INST,
        (DL_UART_Main_ClockConfig *)&g_h7ControlClockConfig);
    DL_UART_Main_init(CAR_H7_UART_INST,
        (DL_UART_Main_Config *)&g_h7ControlConfig);
    DL_UART_Main_configBaudRate(CAR_H7_UART_INST,
        CAR_H7_UART_FREQUENCY, CAR_H7_UART_BAUD_RATE);
    DL_UART_Main_enable(CAR_H7_UART_INST);
}

uint8_t H7ControlUart_TrySendBytes(const uint8_t *data, uint16_t length)
{
    uint32_t primask;
    uint16_t index;
    uint8_t success = 1U;

    if (data == 0) {
        return 0U;
    }
    primask = H7ControlUart_EnterCritical();
    if (g_h7ControlTxBusy != 0U) {
        H7ControlUart_ExitCritical(primask);
        return 0U;
    }
    g_h7ControlTxBusy = 1U;
    H7ControlUart_ExitCritical(primask);

    for (index = 0U; index < length; ++index) {
        uint32_t timeout = H7_CONTROL_TX_TIMEOUT_COUNT;

        while ((timeout > 0U) &&
            !DL_UART_Main_transmitDataCheck(CAR_H7_UART_INST,
                data[index])) {
            --timeout;
        }
        if (timeout == 0U) {
            success = 0U;
            break;
        }
    }

    primask = H7ControlUart_EnterCritical();
    g_h7ControlTxBusy = 0U;
    H7ControlUart_ExitCritical(primask);
    return success;
}

static void H7ControlUart_RequestBallBalance(
    H7BallBalanceRequest request)
{
    uint32_t primask = H7ControlUart_EnterCritical();

    g_h7BallBalancePendingRequest = request;
    if (request == H7_BALL_BALANCE_REQUEST_STOP) {
        g_h7StepperCommandPending = 0U;
    }
    H7ControlUart_ExitCritical(primask);
}

void H7ControlUart_RequestBallBalanceStart(void)
{
    H7ControlUart_RequestBallBalance(
        H7_BALL_BALANCE_REQUEST_START_DEBUG);
}

void H7ControlUart_RequestBallBalanceMode(H7BallBalanceMode mode)
{
    H7BallBalanceRequest request;

    if (mode == H7_BALL_BALANCE_MODE_TASK3) {
        request = H7_BALL_BALANCE_REQUEST_START_TASK3;
    } else if (mode == H7_BALL_BALANCE_MODE_TASK4) {
        request = H7_BALL_BALANCE_REQUEST_START_TASK4;
    } else if (mode == H7_BALL_BALANCE_MODE_TASK5) {
        request = H7_BALL_BALANCE_REQUEST_START_TASK5;
    } else if (mode == H7_BALL_BALANCE_MODE_TASK6) {
        request = H7_BALL_BALANCE_REQUEST_START_TASK6;
    } else if (mode == H7_BALL_BALANCE_MODE_IMU_Y_FEEDFORWARD) {
        request = H7_BALL_BALANCE_REQUEST_START_IMU_Y_FF;
    } else {
        request = H7_BALL_BALANCE_REQUEST_START_DEBUG;
    }
    H7ControlUart_RequestBallBalance(request);
}

void H7ControlUart_RequestBallBalanceStop(void)
{
    H7ControlUart_RequestBallBalance(H7_BALL_BALANCE_REQUEST_STOP);
}

void H7ControlUart_RequestStepperMove(int32_t positionSteps,
    uint16_t speedRpm, uint8_t acceleration)
{
    uint32_t primask = H7ControlUart_EnterCritical();

    g_h7StepperPositionSteps = positionSteps;
    g_h7StepperSpeedRpm = speedRpm;
    g_h7StepperAcceleration = acceleration;
    g_h7StepperCommandPending = 1U;
    H7ControlUart_ExitCritical(primask);
}

void H7ControlUart_ServiceTx(
    int32_t forwardAccelerationX100, uint16_t elapsedMs)
{
    static const uint8_t startCommand[] = "@BALL=BMI\n";
    static const uint8_t task3Command[] = "@BALL=TASK3\n";
    static const uint8_t task4Command[] = "@BALL=TASK4\n";
    static const uint8_t task5Command[] = "@BALL=TASK5\n";
    static const uint8_t task6Command[] = "@BALL=TASK6\n";
    static const uint8_t imuYCommand[] = "@BALL=IMUY\n";
    static const uint8_t stopCommand[] = "@BALL=STOP\n";
    const uint8_t *command;
    uint8_t accelCommand[H7_FORWARD_ACCEL_COMMAND_SIZE];
    uint16_t length;
    uint32_t elapsed;
    uint32_t primask;
    H7BallBalanceRequest ballRequest;
    uint8_t accelDue;
    int32_t stepperPosition;
    uint16_t stepperSpeed;
    uint8_t stepperAcceleration;
    uint8_t stepperDue;
    uint8_t stepperCommand[H7_STEPPER_COMMAND_SIZE];

    primask = H7ControlUart_EnterCritical();
    ballRequest = g_h7BallBalancePendingRequest;
    elapsed = (uint32_t)g_h7ForwardAccelElapsedMs + elapsedMs;
    g_h7ForwardAccelElapsedMs = (uint16_t)(
        (elapsed > H7_FORWARD_ACCEL_PERIOD_MS) ?
            H7_FORWARD_ACCEL_PERIOD_MS : elapsed);
    accelDue = (uint8_t)(g_h7ForwardAccelElapsedMs >=
        H7_FORWARD_ACCEL_PERIOD_MS);
    stepperPosition = g_h7StepperPositionSteps;
    stepperSpeed = g_h7StepperSpeedRpm;
    stepperAcceleration = g_h7StepperAcceleration;
    stepperDue = g_h7StepperCommandPending;
    H7ControlUart_ExitCritical(primask);

    if (ballRequest != H7_BALL_BALANCE_REQUEST_NONE) {
        if (ballRequest == H7_BALL_BALANCE_REQUEST_START_DEBUG) {
            command = startCommand;
            length = (uint16_t)(sizeof(startCommand) - 1U);
        } else if (ballRequest == H7_BALL_BALANCE_REQUEST_START_TASK3) {
            command = task3Command;
            length = (uint16_t)(sizeof(task3Command) - 1U);
        } else if (ballRequest == H7_BALL_BALANCE_REQUEST_START_TASK4) {
            command = task4Command;
            length = (uint16_t)(sizeof(task4Command) - 1U);
        } else if (ballRequest == H7_BALL_BALANCE_REQUEST_START_TASK5) {
            command = task5Command;
            length = (uint16_t)(sizeof(task5Command) - 1U);
        } else if (ballRequest == H7_BALL_BALANCE_REQUEST_START_TASK6) {
            command = task6Command;
            length = (uint16_t)(sizeof(task6Command) - 1U);
        } else if (ballRequest ==
            H7_BALL_BALANCE_REQUEST_START_IMU_Y_FF) {
            command = imuYCommand;
            length = (uint16_t)(sizeof(imuYCommand) - 1U);
        } else {
            command = stopCommand;
            length = (uint16_t)(sizeof(stopCommand) - 1U);
        }
        if (H7ControlUart_TrySendBytes(command, length) != 0U) {
            primask = H7ControlUart_EnterCritical();
            if (g_h7BallBalancePendingRequest == ballRequest) {
                g_h7BallBalancePendingRequest =
                    H7_BALL_BALANCE_REQUEST_NONE;
            }
            H7ControlUart_ExitCritical(primask);
        }
    }

    if (stepperDue != 0U) {
        length = H7ControlUart_FormatStepperMove(stepperPosition,
            stepperSpeed, stepperAcceleration, stepperCommand);
        if (H7ControlUart_TrySendBytes(stepperCommand, length) != 0U) {
            primask = H7ControlUart_EnterCritical();
            if ((g_h7StepperPositionSteps == stepperPosition) &&
                (g_h7StepperSpeedRpm == stepperSpeed) &&
                (g_h7StepperAcceleration == stepperAcceleration)) {
                g_h7StepperCommandPending = 0U;
            }
            H7ControlUart_ExitCritical(primask);
        }
    }

    if (accelDue == 0U) {
        return;
    }
    length = H7ControlUart_FormatForwardAcceleration(
        forwardAccelerationX100, accelCommand);
    if (H7ControlUart_TrySendBytes(accelCommand, length) != 0U) {
        primask = H7ControlUart_EnterCritical();
        g_h7ForwardAccelElapsedMs = 0U;
        H7ControlUart_ExitCritical(primask);
    }
}

#else

void H7ControlUart_Init(void)
{
}

uint8_t H7ControlUart_TrySendBytes(const uint8_t *data, uint16_t length)
{
    (void)data;
    (void)length;
    return 0U;
}

void H7ControlUart_RequestBallBalanceStart(void)
{
}

void H7ControlUart_RequestBallBalanceMode(H7BallBalanceMode mode)
{
    (void)mode;
}

void H7ControlUart_RequestBallBalanceStop(void)
{
}

void H7ControlUart_RequestStepperMove(int32_t positionSteps,
    uint16_t speedRpm, uint8_t acceleration)
{
    (void)positionSteps;
    (void)speedRpm;
    (void)acceleration;
}

void H7ControlUart_ServiceTx(
    int32_t forwardAccelerationX100, uint16_t elapsedMs)
{
    (void)forwardAccelerationX100;
    (void)elapsedMs;
}

#endif
