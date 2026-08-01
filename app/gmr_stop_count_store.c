#include "library_config.h"

#if CAR_PROFILE_IS_GMR

#include "gmr_stop_count_store.h"

#include "control_config.h"
#include "ti_msp_dl_config.h"

/* MSPM0G3507主Flash最后一个1KB扇区；链接文件已从程序区永久排除。 */
#define GMR_STOP_COUNT_FLASH_ADDRESS          (0x0001FC00UL)
#define GMR_STOP_COUNT_FLASH_MAGIC            (0x53434D47UL)
#define GMR_STOP_COUNT_FLASH_VERSION          (1UL)
#define GMR_STOP_COUNT_FLASH_WORD_COUNT       (10U)
#define GMR_STOP_COUNT_FLASH_CRC_BYTE_COUNT   (32U)

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t values[GMR_STOP_COUNT_ITEM_COUNT];
    uint32_t crc32;
    uint32_t reserved;
} GmrStopCountFlashRecord;

static GmrStopCountValues g_runtimeValues;
static GmrStopCountFlashRecord g_writeRecord;
static uint8_t g_flashLoaded;

static uint32_t GmrStopCountStore_CalculateCrc32(
    const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    uint32_t byteIndex;
    uint8_t bitIndex;

    for (byteIndex = 0U; byteIndex < length; ++byteIndex) {
        crc ^= data[byteIndex];
        for (bitIndex = 0U; bitIndex < 8U; ++bitIndex) {
            crc = ((crc & 1U) != 0U) ?
                ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
        }
    }
    return ~crc;
}

static uint8_t GmrStopCountStore_ValuesAreValid(
    const uint32_t values[GMR_STOP_COUNT_ITEM_COUNT])
{
    uint8_t item;

    for (item = 0U; item < GMR_STOP_COUNT_ITEM_COUNT; ++item) {
        if ((values[item] < GMR_STOP_COUNT_MIN_VALUE) ||
            (values[item] > GMR_STOP_COUNT_MAX_VALUE)) {
            return 0U;
        }
    }
    return 1U;
}

static void GmrStopCountStore_LoadDefaults(void)
{
    g_runtimeValues.values[GMR_STOP_COUNT_TASK1_LEFT] =
        (uint32_t)GMR_MISSION2_LEFT_STOP_COUNT;
    g_runtimeValues.values[GMR_STOP_COUNT_TASK1_RIGHT] =
        (uint32_t)GMR_MISSION2_RIGHT_STOP_COUNT;
    g_runtimeValues.values[GMR_STOP_COUNT_TASK4_LEFT] =
        (uint32_t)GMR_TASK4_LINE_FOLLOW_LEFT_STOP_COUNT;
    g_runtimeValues.values[GMR_STOP_COUNT_TASK4_RIGHT] =
        (uint32_t)GMR_TASK4_LINE_FOLLOW_RIGHT_STOP_COUNT;
    g_runtimeValues.values[GMR_STOP_COUNT_TASK5_LEFT] =
        (uint32_t)GMR_TASK5_LINE_FOLLOW_LEFT_STOP_COUNT;
    g_runtimeValues.values[GMR_STOP_COUNT_TASK5_RIGHT] =
        (uint32_t)GMR_TASK5_LINE_FOLLOW_RIGHT_STOP_COUNT;
}

static uint8_t GmrStopCountStore_RecordIsValid(
    const GmrStopCountFlashRecord *record)
{
    uint32_t expectedCrc;

    if ((record->magic != GMR_STOP_COUNT_FLASH_MAGIC) ||
        (record->version != GMR_STOP_COUNT_FLASH_VERSION) ||
        (GmrStopCountStore_ValuesAreValid(record->values) == 0U)) {
        return 0U;
    }
    expectedCrc = GmrStopCountStore_CalculateCrc32(
        (const uint8_t *)record, GMR_STOP_COUNT_FLASH_CRC_BYTE_COUNT);
    return (uint8_t)((record->crc32 == expectedCrc) ? 1U : 0U);
}

void GmrStopCountStore_Init(void)
{
    const GmrStopCountFlashRecord *record =
        (const GmrStopCountFlashRecord *)(uintptr_t)
            GMR_STOP_COUNT_FLASH_ADDRESS;
    uint8_t item;

    GmrStopCountStore_LoadDefaults();
    g_flashLoaded = 0U;
    if (GmrStopCountStore_RecordIsValid(record) == 0U) {
        return;
    }
    for (item = 0U; item < GMR_STOP_COUNT_ITEM_COUNT; ++item) {
        g_runtimeValues.values[item] = record->values[item];
    }
    g_flashLoaded = 1U;
}

uint32_t GmrStopCountStore_GetValue(GmrStopCountItem item)
{
    if ((uint32_t)item >= (uint32_t)GMR_STOP_COUNT_ITEM_COUNT) {
        return 0U;
    }
    return g_runtimeValues.values[item];
}

void GmrStopCountStore_GetValues(GmrStopCountValues *values)
{
    if (values != 0) {
        *values = g_runtimeValues;
    }
}

uint8_t GmrStopCountStore_Save(const GmrStopCountValues *values)
{
    const GmrStopCountFlashRecord *storedRecord =
        (const GmrStopCountFlashRecord *)(uintptr_t)
            GMR_STOP_COUNT_FLASH_ADDRESS;
    DL_FLASHCTL_COMMAND_STATUS status;
    uint32_t primask;
    uint8_t item;

    if ((values == 0) ||
        (GmrStopCountStore_ValuesAreValid(values->values) == 0U)) {
        return 0U;
    }

    g_writeRecord.magic = GMR_STOP_COUNT_FLASH_MAGIC;
    g_writeRecord.version = GMR_STOP_COUNT_FLASH_VERSION;
    for (item = 0U; item < GMR_STOP_COUNT_ITEM_COUNT; ++item) {
        g_writeRecord.values[item] = values->values[item];
    }
    g_writeRecord.crc32 = GmrStopCountStore_CalculateCrc32(
        (const uint8_t *)&g_writeRecord,
        GMR_STOP_COUNT_FLASH_CRC_BYTE_COUNT);
    g_writeRecord.reserved = 0xFFFFFFFFUL;

    /* Flash忙时禁止中断进入主Flash；菜单保存时底盘已经停车。 */
    primask = __get_PRIMASK();
    __disable_irq();
    DL_FlashCTL_unprotectSector(FLASHCTL,
        GMR_STOP_COUNT_FLASH_ADDRESS, DL_FLASHCTL_REGION_SELECT_MAIN);
    status = DL_FlashCTL_eraseMemoryFromRAM(FLASHCTL,
        GMR_STOP_COUNT_FLASH_ADDRESS, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    if (status != DL_FLASHCTL_COMMAND_STATUS_FAILED) {
        status = DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
            FLASHCTL, GMR_STOP_COUNT_FLASH_ADDRESS,
            (uint32_t *)&g_writeRecord, GMR_STOP_COUNT_FLASH_WORD_COUNT,
            DL_FLASHCTL_REGION_SELECT_MAIN);
    }
    __set_PRIMASK(primask);

    if ((status == DL_FLASHCTL_COMMAND_STATUS_FAILED) ||
        (GmrStopCountStore_RecordIsValid(storedRecord) == 0U)) {
        return 0U;
    }
    for (item = 0U; item < GMR_STOP_COUNT_ITEM_COUNT; ++item) {
        if (storedRecord->values[item] != values->values[item]) {
            return 0U;
        }
    }
    g_runtimeValues = *values;
    g_flashLoaded = 1U;
    return 1U;
}

uint8_t GmrStopCountStore_IsFlashLoaded(void)
{
    return g_flashLoaded;
}

#endif
