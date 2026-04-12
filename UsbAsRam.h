/*++
Module Name: UsbAsRam.h
Abstract: Shared device context; Modules 2–5 hooks (DMA, LZ4, write-back cache, IOCTL).
Environment: Kernel mode, KMDF
--*/

#pragma once

#include <ntddk.h>
#include <wdf.h>

#define USR_POOL_TAG 'RsAU'

#ifndef USR_DEFAULT_VID
#define USR_DEFAULT_VID 0x048D
#endif
#ifndef USR_DEFAULT_PID
#define USR_DEFAULT_PID 0x1234
#endif

#define USR_BYTES_PER_SECTOR 512
#define USR_PAGE_SIZE 4096
#define USR_WB_CACHE_BYTES (128 * 1024 * 1024)
#define USR_WB_SLOT_COUNT (USR_WB_CACHE_BYTES / USR_PAGE_SIZE)
#define USR_WB_SMALL_WRITE_MAX (USR_PAGE_SIZE)

#define USR_MAX_DMA_TRANSFER_LENGTH (1024 * 1024)

//
// Private IOCTL — user-mode must open the device interface and DeviceIoControl.
//
#define IOCTL_USBASRAM_REVERT_PMEM CTL_CODE(FILE_DEVICE_UNKNOWN, 0x901, METHOD_BUFFERED, FILE_ANY_ACCESS)

typedef struct _USBASRAM_WB_SLOT {
    LONGLONG PageBase;
    ULONG Dirty;
} USBASRAM_WB_SLOT, *PUSBASRAM_WB_SLOT;

typedef struct _USBASRAM_DEVICE_CONTEXT {
    WDFDEVICE WdfDevice;
    ULONG VendorId;
    ULONG ProductId;
    ULONG ExpectedVendorId;
    ULONG ExpectedProductId;
    BOOLEAN IsTargetUsbDevice;
    BOOLEAN MemoryArrayInterfaceRegistered;
    BOOLEAN InterceptVolumeMountRequested;

    WDFDMAENABLER DmaEnabler;
    WDFIOQUEUE IoQueue;
    ULONG LogicalSectorSize;
    ULONG MaxXferPerTransaction;
    BOOLEAN DmaInitialized;
    BOOLEAN IoQueueCreated;
    BOOLEAN PreferUaspPath;

    //
    // Module 3 — LZ4 (scratch in NonPagedPoolNx; compress before “USB”).
    //
    BOOLEAN Lz4Initialized;
    BOOLEAN Lz4Enabled;
    PUCHAR Lz4Scratch;
    SIZE_T Lz4ScratchBytes;

    //
    // Module 4 — 64 MiB write-back staging (4 KiB slots, direct-mapped).
    //
    BOOLEAN WbInitialized;
    PUCHAR WbBuffer;
    PUSBASRAM_WB_SLOT WbMeta;
    WDFSPINLOCK WbLock;
    WDFWORKITEM WbFlushWorkItem;
    LONG WbFlushScheduled;

    //
    // Module 5 — revert path (full detach needs installer/stack cooperation).
    //
    BOOLEAN RevertRequested;

    // Module 6 — USB / hardware watchdog and MMIO mapping
    BOOLEAN UsbInitialized;
    WDFUSBDEVICE UsbDevice;
    WDFUSBPIPE BulkInPipe;
    WDFUSBPIPE BulkOutPipe;
    WDFTIMER HwWatchdogTimer;
    PVOID MmioBase;
    SIZE_T MmioSize;

    // Pending async USB requests tracking
    LIST_ENTRY PendingList;
    WDFSPINLOCK PendingLock;
    ULONG PendingCount;
} USBASRAM_DEVICE_CONTEXT, *PUSBASRAM_DEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(USBASRAM_DEVICE_CONTEXT, UsbAsRamDeviceGetContext)

NTSTATUS
UsbAsRamModule2Initialize(
    _In_ WDFDEVICE Device
    );

VOID
UsbAsRamModule2Cleanup(
    _In_ WDFDEVICE Device
    );

NTSTATUS
UsbAsRamLz4Initialize(
    _In_ WDFDEVICE Device
    );

VOID
UsbAsRamLz4Cleanup(
    _In_ WDFDEVICE Device
    );

NTSTATUS
UsbAsRamWbInitialize(
    _In_ WDFDEVICE Device
    );

VOID
UsbAsRamWbCleanup(
    _In_ WDFDEVICE Device
    );

BOOLEAN
UsbAsRamWbTryStageWrite(
    _In_ WDFDEVICE Device,
    _In_ WDFREQUEST Request
    );

VOID
UsbAsRamWbFlushAll(
    _In_ WDFDEVICE Device
    );

NTSTATUS
UsbAsRamProcessWriteWithLz4(
    _In_ WDFDEVICE Device,
    _In_ WDFREQUEST Request,
    _Out_ PBOOLEAN RequestCompleted
    );

VOID
UsbAsRamLz4DecompressAfterDmaIfNeeded(
    _In_ WDFDEVICE Device,
    _In_ WDFREQUEST Request
    );

// Per-child async USB request context
typedef struct _USBASRAM_CHILD_REQUEST_CONTEXT {
    PVOID Buffer;
    SIZE_T BufferSize;
    WDFMEMORY Memory;
    PVOID Owner; // pointer to owner struct
    WDFREQUEST Request;
    LARGE_INTEGER StartTime;
    LIST_ENTRY ListEntry;
} USBASRAM_CHILD_REQUEST_CONTEXT, *PUSBASRAM_CHILD_REQUEST_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(USBASRAM_CHILD_REQUEST_CONTEXT, UsbAsRamChildRequestGetContext)

// Owner structure that tracks original request and outstanding child count
typedef struct _USBASRAM_OWNER {
    WDFREQUEST OriginalRequest;
    LONG Outstanding;
    ULONG TotalBytes;
} USBASRAM_OWNER, *PUSBASRAM_OWNER;

NTSTATUS
UsbAsRamUsbInitialize(
    _In_ WDFDEVICE Device
    );

VOID
UsbAsRamUsbCleanup(
    _In_ WDFDEVICE Device
    );

NTSTATUS
UsbAsRamSendBulkTransfersFromSgList(
    _In_ WDFDEVICE Device,
    _In_ PSCATTER_GATHER_LIST SgList,
    _In_ BOOLEAN WriteToDevice,
    _In_ WDFREQUEST OriginalRequest
    );

EVT_WDF_TIMER UsbAsRamHwWatchdogTimerFunc;

// USB / URB helpers (minimal placeholders).
NTSTATUS
UsbAsRamUsbInitialize(
    _In_ WDFDEVICE Device
    );

VOID
UsbAsRamUsbCleanup(
    _In_ WDFDEVICE Device
    );

NTSTATUS
UsbAsRamSendBulkTransfersFromSgList(
    _In_ WDFDEVICE Device,
    _In_ PSCATTER_GATHER_LIST SgList,
    _In_ BOOLEAN WriteToDevice
    );
