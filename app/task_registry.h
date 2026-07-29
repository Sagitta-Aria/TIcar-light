#ifndef TASK_REGISTRY_H
#define TASK_REGISTRY_H

#include <stdint.h>

#include "state_machine.h"

/* Returns the number of menu tasks exposed by the active product profile. */
uint8_t TaskRegistry_GetCount(void);

/* Returns the display name for a zero-based menu index, or an empty string. */
const char *TaskRegistry_GetName(uint8_t taskIndex);

/* Returns the mission ID behind a zero-based menu index, or 0 when invalid. */
uint8_t TaskRegistry_GetMissionId(uint8_t taskIndex);

/* Returns the start event behind a zero-based menu index. */
CarEvent TaskRegistry_GetStartEvent(uint8_t taskIndex);

#endif
