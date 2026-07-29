#ifndef GMR_BLUETOOTH_MISSION_H
#define GMR_BLUETOOTH_MISSION_H

#include <stdint.h>

typedef enum {
    GMR_BLUETOOTH_MISSION_RESULT_RUNNING = 0,
    GMR_BLUETOOTH_MISSION_RESULT_FINISHED,
    GMR_BLUETOOTH_MISSION_RESULT_ERROR
} GmrBluetoothMissionResult;

typedef enum {
    GMR_BLUETOOTH_MISSION_PHASE_IDLE = 0,
    GMR_BLUETOOTH_MISSION_PHASE_WAIT_LINK,
    GMR_BLUETOOTH_MISSION_PHASE_WAIT_READY,
    GMR_BLUETOOTH_MISSION_PHASE_WAIT_START_ACK,
    GMR_BLUETOOTH_MISSION_PHASE_RECORDING,
    GMR_BLUETOOTH_MISSION_PHASE_WAIT_OVER_ACK,
    GMR_BLUETOOTH_MISSION_PHASE_WAIT_START,
    GMR_BLUETOOTH_MISSION_PHASE_RECEIVING,
    GMR_BLUETOOTH_MISSION_PHASE_REPLAY,
    GMR_BLUETOOTH_MISSION_PHASE_DONE,
    GMR_BLUETOOTH_MISSION_PHASE_ERROR
} GmrBluetoothMissionPhase;

typedef enum {
    GMR_BLUETOOTH_MISSION_ERROR_NONE = 0,
    GMR_BLUETOOTH_MISSION_ERROR_DISABLED,
    GMR_BLUETOOTH_MISSION_ERROR_LINK_LOST,
    GMR_BLUETOOTH_MISSION_ERROR_TX_QUEUE,
    GMR_BLUETOOTH_MISSION_ERROR_PROTOCOL,
    GMR_BLUETOOTH_MISSION_ERROR_SAMPLE_ORDER,
    GMR_BLUETOOTH_MISSION_ERROR_BUFFER_FULL,
    GMR_BLUETOOTH_MISSION_ERROR_DISTANCE_RANGE,
    GMR_BLUETOOTH_MISSION_ERROR_REPLAY_TIMEOUT,
    GMR_BLUETOOTH_MISSION_ERROR_LINE_FOLLOW
} GmrBluetoothMissionError;

typedef struct {
    uint8_t active;
    uint8_t connected;
    uint8_t role;
    GmrBluetoothMissionPhase phase;
    GmrBluetoothMissionError error;
    uint16_t sampleCount;
    uint16_t replayIndex;
    uint32_t completedTurns;
    int32_t replayLeftErrorCounts;
    int32_t replayRightErrorCounts;
} GmrBluetoothMissionStatus;

void GmrBluetoothMission_Init(void);
void GmrBluetoothMission_Start(void);
void GmrBluetoothMission_Stop(void);

/* Called after BluetoothService_Task() from the fixed 5 ms Comm task. */
void GmrBluetoothMission_CommTask(uint32_t elapsedMs);

/* Called before the encoder PI loop from the fixed 10 ms chassis task. */
GmrBluetoothMissionResult GmrBluetoothMission_ControlPeriod(void);
void GmrBluetoothMission_HandleFastEvent(void);

void GmrBluetoothMission_GetStatus(GmrBluetoothMissionStatus *status);
const char *GmrBluetoothMission_GetPhaseName(GmrBluetoothMissionPhase phase);
const char *GmrBluetoothMission_GetErrorName(GmrBluetoothMissionError error);

#endif
