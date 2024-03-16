#include "common.h"
#include <libtwl/ipc/ipcFifo.h>
#include <libtwl/ipc/ipcFifoSystem.h>
#include <string.h>
#include "Fat/ff.h"
#include "Core/Environment.h"
#include "MemFastSearch.h"
#include "SaveSwi.h"
#include "SaveTypeInfo.h"
#include "VirtualMachine/VMNestedIrq.h"
#include "MemoryEmulator/RomDefs.h"
#include "IpcChannels.h"
#include "GbaSaveIpcCommand.h"
#include "Save.h"
#include "MemCopy.h"

#define DEFAULT_SAVE_SIZE   (32 * 1024)

[[gnu::section(".ewram.bss")]]
u8 gSaveData[SAVE_DATA_SIZE] alignas(32);

[[gnu::section(".ewram.bss")]]
FIL gSaveFile alignas(32);

[[gnu::section(".ewram.bss"), gnu::aligned(32)]]
gba_save_shared_t gGbaSaveShared;

static DWORD sClusterTable[64];
static u32 sSkipSaveCheckInstruction;

void fillDebugBuf(void* buf, u32 size, const char* filePath)
{
    FIL gDebugBuf;
    memset(&gDebugBuf, 0, sizeof(gDebugBuf));
    if (f_open(&gDebugBuf, filePath, FA_OPEN_EXISTING | FA_READ | FA_WRITE) == FR_OK)
    {
        f_lseek(&gDebugBuf, 0);
        for (u32 i = 0; i < size; ++i)
        {
            const u32 data = *(u32*)(void*)(buf + i);
            UINT written = 0;
            f_write(&gDebugBuf, &data, 1, &written);
        }
        f_sync(&gDebugBuf);
    } else if (f_open(&gDebugBuf, filePath, FA_CREATE_NEW | FA_OPEN_EXISTING | FA_READ | FA_WRITE) == FR_OK){
        f_lseek(&gDebugBuf, 0);
        for (u32 i = 0; i < size; ++i)
        {
            const u32 data = *(u32*)(void*)(buf + i);
            UINT written = 0;
            f_write(&gDebugBuf, &data, 1, &written);
        }
        f_sync(&gDebugBuf);
    }
}

bool sav_tryPatchFunction(const u32* signature, u32 saveSwiNumber, void* patchFunction)
{
    u32* function = (u32*)mem_fastSearch16((const u32*)ROM_LINEAR_DS_ADDRESS, ROM_LINEAR_SIZE, signature);
    if (!function) {
        return false;
    }
    sav_swiTable[saveSwiNumber] = patchFunction;
    *(u16*)function = SAVE_THUMB_SWI(saveSwiNumber);
    return true;
}

static void loadSaveClusterMap(void)
{
    sClusterTable[0] = sizeof(sClusterTable) / sizeof(DWORD);
    gSaveFile.cltbl = sClusterTable;
    f_lseek(&gSaveFile, CREATE_LINKMAP);
}

static void fillSaveFile(u32 start, u32 end)
{
    const u8 saveFill = SAVE_DATA_FILL;
    f_lseek(&gSaveFile, start);
    for (u32 i = start; i < end; ++i)
    {
        UINT written = 0;
        f_write(&gSaveFile, &saveFill, 1, &written);
    }
    f_sync(&gSaveFile);
}

void sav_initializeSave(const SaveTypeInfo* saveTypeInfo, const char* savePath)
{
    u32 saveSize = saveTypeInfo ? saveTypeInfo->size : DEFAULT_SAVE_SIZE;
    memset(gSaveData, SAVE_DATA_FILL, SAVE_DATA_SIZE);
    if (Environment::IsIsNitroEmulator())
    {
        memset((void*)ISNITRO_SAVE_BUFFER, SAVE_DATA_FILL, ISNITRO_SAVE_BUFFER_SIZE);
    }
    memset(&gSaveFile, 0, sizeof(gSaveFile));
    if (f_open(&gSaveFile, savePath, FA_OPEN_EXISTING | FA_READ | FA_WRITE) == FR_OK)
    {
        bool clusterMapLoaded = false;
        u32 initialSize = f_size(&gSaveFile);
        if (initialSize < saveSize)
        {
            if (f_lseek(&gSaveFile, saveSize) == FR_OK)
            {
                f_rewind(&gSaveFile);
                loadSaveClusterMap();
                clusterMapLoaded = true;
                fillSaveFile(initialSize, saveSize);
            }
        }

        if (!clusterMapLoaded)
        {
            loadSaveClusterMap();
            clusterMapLoaded = true;
        }

        if (saveSize <= SAVE_DATA_SIZE)
        {
            f_rewind(&gSaveFile);
            UINT read = 0;
            f_read(&gSaveFile, gSaveData, saveSize, &read);
        }

        if (Environment::IsIsNitroEmulator())
        {
            f_rewind(&gSaveFile);
            UINT read = 0;
            f_read(&gSaveFile, (void*)ISNITRO_SAVE_BUFFER, saveSize, &read);
        }
    }
    else if (!Environment::IsIsNitroEmulator())
    {
        if (f_open(&gSaveFile, savePath, FA_CREATE_NEW | FA_READ | FA_WRITE) == FR_OK)
        {
            if (f_lseek(&gSaveFile, saveSize) == FR_OK)
            {
                f_rewind(&gSaveFile);
                loadSaveClusterMap();
                fillSaveFile(0, saveSize);
            }
        }
    }

    gGbaSaveShared.saveState = GBA_SAVE_STATE_CLEAN;
    sSkipSaveCheckInstruction = emu_vblankIrqSkipSaveCheckInstruction;
    if (!saveTypeInfo || (saveTypeInfo->type & SAVE_TYPE_SRAM))
    {
        gGbaSaveShared.saveData = gSaveData;
        gGbaSaveShared.saveDataSize = saveSize;
    }
    else
    {
        gGbaSaveShared.saveData = nullptr;
        gGbaSaveShared.saveDataSize = 0;
    }

    ipc_sendWordDirect(
        ((((u32)&gGbaSaveShared) >> 5) << (IPC_FIFO_MSG_CHANNEL_BITS + 3)) |
        (GBA_SAVE_IPC_CMD_SETUP << IPC_FIFO_MSG_CHANNEL_BITS) |
        IPC_CHANNEL_GBA_SAVE);
    while (ipc_isRecvFifoEmpty());
    ipc_recvWordDirect();
}

extern "C" u8 sav_readSaveByteFromFile(u32 saveAddress)
{
    vm_enableNestedIrqs();
    u8 saveByte;
    if (Environment::IsIsNitroEmulator())
    {
        // save buffer in extended memory
        saveByte = ISNITRO_SAVE_BUFFER[saveAddress];
    }
    else
    {
        // write to file
        f_lseek(&gSaveFile, saveAddress);
        UINT bytesRead = 0;
        f_read(&gSaveFile, &saveByte, 1, &bytesRead);
    }
    vm_disableNestedIrqs();
    return saveByte;
}

extern "C" void sav_writeSaveByteToFile(u32 saveAddress, u8 data)
{
    vm_enableNestedIrqs();
    if (Environment::IsIsNitroEmulator())
    {
        // save buffer in extended memory
        ISNITRO_SAVE_BUFFER[saveAddress] = data;
    }
    else
    {
        // write to file
        f_lseek(&gSaveFile, saveAddress);
        UINT bytesWritten = 0;
        f_write(&gSaveFile, &data, 1, &bytesWritten);
    }
    vm_disableNestedIrqs();
}

extern "C" void sav_flushSaveFile(void)
{
    vm_enableNestedIrqs();
    if (!Environment::IsIsNitroEmulator())
    {
        f_sync(&gSaveFile);
    }
    vm_disableNestedIrqs();
}

extern "C" void sav_writeSaveToFile(void)
{
    if (gGbaSaveShared.saveDataSize != 0 && !Environment::IsIsNitroEmulator())
    {
        f_lseek(&gSaveFile, 0);
        UINT bytesWritten = 0;
        f_write(&gSaveFile, gSaveData, gGbaSaveShared.saveDataSize, &bytesWritten);
        f_sync(&gSaveFile);
    }

    gGbaSaveShared.saveState = GBA_SAVE_STATE_CLEAN;
    emu_vblankIrqSkipSaveCheckInstruction = sSkipSaveCheckInstruction;
}
