/*++
Module Name: DmaIo.cpp
Abstract: Modules 2–5 — DMA S/G, LBA mapping, LZ4 write path, write-back cache hooks,
          IOCTL revert forward-path.
Environment: Kernel mode, Windows 11, KMDF
--*/

#include <ntddk.h>
#include <wdf.h>

#include "UsbAsRam.h"
#include <wdfusb.h>

extern "C" {

EVT_WDF_IO_QUEUE_IO_READ UsbAsRamEvtIoRead;
EVT_WDF_IO_QUEUE_IO_WRITE UsbAsRamEvtIoWrite;
EVT_WDF_IO_QUEUE_IO_DEVICE_CONTROL UsbAsRamEvtIoDeviceControl;
EVT_WDF_IO_QUEUE_IO_DEFAULT UsbAsRamEvtIoDefault;
EVT_WDF_PROGRAM_DMA UsbAsRamEvtProgramDma;
EVT_WDF_DMA_TRANSACTION_CLIENT_DmaCompleted UsbAsRamEvtDmaTransactionComplete;

}

#define USR_LOG_COMPONENT "UsbAsRam"

#define UsbAsRamTraceError(_fmt, ...) \
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_ERROR_LEVEL, \
        "[" USR_LOG_COMPONENT ":ERR] " _fmt "\n", __VA_ARGS__)

#define UsbAsRamTraceInfo(_fmt, ...) \
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_INFO_LEVEL, \
        "[" USR_LOG_COMPONENT ":INF] " _fmt "\n", __VA_ARGS__)

#define UsbAsRamTraceVerbose(_fmt, ...) \
    DbgPrintEx(DPFLTR_IHVDRIVER_ID, DPFLTR_TRACE_LEVEL, \
        "[" USR_LOG_COMPONENT ":VRB] " _fmt "\n", __VA_ARGS__)

static VOID
UsbAsRamByteRangeToLba(
    _In_ LONGLONG StartingOffset,
    _In_ ULONG Length,
    _In_ ULONG BytesPerSector,
    _Out_ PLONGLONG FirstLba,
    _Out_ PLONGLONG LastLbaInclusive
    );

static NTSTATUS
UsbAsRamReadUaspPreference(
    _In_ WDFDRIVER Driver,
    _Out_ PBOOLEAN PreferUasp
    );

static VOID
UsbAsRamLogScatterGatherList(
    _In_ PSCATTER_GATHER_LIST SgList,
    _In_ LONGLONG FirstLba,
    _In_ LONGLONG LastLbaInclusive,
    _In_ BOOLEAN WriteToDevice
    );

static VOID
UsbAsRamForwardRequestToLower(
    _In_ WDFDEVICE Device,
    _In_ WDFREQUEST Request
    );

VOID
UsbAsRamModule2Cleanup(
    _In_ WDFDEVICE Device
    )
{
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);

    if (ctx->IoQueueCreated) {
        WdfIoQueuePurgeSynchronously(ctx->IoQueue);
    }

    UsbAsRamWbFlushAll(Device);
    UsbAsRamWbCleanup(Device);
    UsbAsRamLz4Cleanup(Device);

    if (ctx->IoQueueCreated) {
        WdfObjectDelete(ctx->IoQueue);
        ctx->IoQueue = NULL;
        ctx->IoQueueCreated = FALSE;
    }

    // Cleanup watchdog timer and USB placeholders
    if (ctx->HwWatchdogTimer != NULL) {
        WdfObjectDelete(ctx->HwWatchdogTimer);
        ctx->HwWatchdogTimer = NULL;
    }
    ctx->UsbInitialized = FALSE;
    ctx->BulkInPipe = NULL;
    ctx->BulkOutPipe = NULL;

    if (ctx->DmaInitialized) {
        WdfObjectDelete(ctx->DmaEnabler);
        ctx->DmaEnabler = NULL;
        ctx->DmaInitialized = FALSE;
    }
}

NTSTATUS
UsbAsRamModule2Initialize(
    _In_ WDFDEVICE Device
    )
{
    NTSTATUS status;
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);
    WDF_DMA_ENABLER_CONFIG dmaConfig;
    WDF_OBJECT_ATTRIBUTES dmaAttributes;
    WDF_IO_QUEUE_CONFIG queueConfig;
    BOOLEAN preferUasp = FALSE;

    if (!ctx->IsTargetUsbDevice) {
        return STATUS_SUCCESS;
    }

    (VOID)UsbAsRamReadUaspPreference(WdfDeviceGetDriver(Device), &preferUasp);
    ctx->PreferUaspPath = preferUasp;

    ctx->LogicalSectorSize = USR_BYTES_PER_SECTOR;
    ctx->MaxXferPerTransaction = USR_MAX_DMA_TRANSFER_LENGTH;

    WDF_DMA_ENABLER_CONFIG_INIT(
        &dmaConfig,
        WdfDmaProfileScatterGather64,
        ctx->MaxXferPerTransaction);

    dmaConfig.EvtDmaTransactionProgram = UsbAsRamEvtProgramDma;

    WDF_OBJECT_ATTRIBUTES_INIT(&dmaAttributes);
    dmaAttributes.ExecutionLevel = WdfExecutionLevelPassive;

    status = WdfDmaEnablerCreate(Device, &dmaConfig, &dmaAttributes, &ctx->DmaEnabler);
    if (!NT_SUCCESS(status)) {
        UsbAsRamTraceError("%s: WdfDmaEnablerCreate(ScatterGather64) failed: 0x%08X, trying Packet64",
            __FUNCTION__, status);

        WDF_DMA_ENABLER_CONFIG_INIT(
            &dmaConfig,
            WdfDmaProfilePacket64,
            ctx->MaxXferPerTransaction);
        dmaConfig.EvtDmaTransactionProgram = UsbAsRamEvtProgramDma;

        status = WdfDmaEnablerCreate(Device, &dmaConfig, &dmaAttributes, &ctx->DmaEnabler);
        if (!NT_SUCCESS(status)) {
            UsbAsRamTraceError("%s: WdfDmaEnablerCreate(Packet64) failed: 0x%08X",
                __FUNCTION__, status);
            return status;
        }
    }

    ctx->DmaInitialized = TRUE;
    UsbAsRamTraceInfo("%s: DMA enabler ready (max xfer=%lu bytes, UASP preferred=%u)",
        __FUNCTION__, ctx->MaxXferPerTransaction, ctx->PreferUaspPath);

    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&queueConfig, WdfIoQueueDispatchParallel);
    queueConfig.PowerManaged = WdfTrue;
    queueConfig.EvtIoRead = UsbAsRamEvtIoRead;
    queueConfig.EvtIoWrite = UsbAsRamEvtIoWrite;
    queueConfig.EvtIoDeviceControl = UsbAsRamEvtIoDeviceControl;
    queueConfig.EvtIoDefault = UsbAsRamEvtIoDefault;

    status = WdfIoQueueCreate(
        Device,
        &queueConfig,
        WDF_NO_OBJECT_ATTRIBUTES,
        &ctx->IoQueue);

    if (!NT_SUCCESS(status)) {
        UsbAsRamTraceError("%s: WdfIoQueueCreate failed: 0x%08X", __FUNCTION__, status);
        if (ctx->DmaInitialized) {
            WdfObjectDelete(ctx->DmaEnabler);
            ctx->DmaEnabler = NULL;
            ctx->DmaInitialized = FALSE;
        }
        return status;
    }

    ctx->IoQueueCreated = TRUE;

    status = UsbAsRamLz4Initialize(Device);
    if (!NT_SUCCESS(status)) {
        UsbAsRamTraceError("%s: UsbAsRamLz4Initialize failed 0x%08X", __FUNCTION__, status);
        WdfObjectDelete(ctx->IoQueue);
        ctx->IoQueue = NULL;
        ctx->IoQueueCreated = FALSE;
        WdfObjectDelete(ctx->DmaEnabler);
        ctx->DmaEnabler = NULL;
        ctx->DmaInitialized = FALSE;
        return status;
    }

    status = UsbAsRamWbInitialize(Device);
    if (!NT_SUCCESS(status)) {
        UsbAsRamTraceError("%s: UsbAsRamWbInitialize failed 0x%08X", __FUNCTION__, status);
        UsbAsRamLz4Cleanup(Device);
        WdfObjectDelete(ctx->IoQueue);
        ctx->IoQueue = NULL;
        ctx->IoQueueCreated = FALSE;
        WdfObjectDelete(ctx->DmaEnabler);
        ctx->DmaEnabler = NULL;
        ctx->DmaInitialized = FALSE;
        return status;
    }

    UsbAsRamTraceInfo(
        "%s: Queue+DMA+LZ4+WB cache ready. UASP is negotiated by usbstor/uaspstor.",
        __FUNCTION__);

    // USB/watchdog setup is performed after USB pipes are enumerated in UsbAsRamUsbInitialize.
    ctx->UsbInitialized = FALSE;

    return STATUS_SUCCESS;
}

static NTSTATUS
UsbAsRamReadUaspPreference(
    _In_ WDFDRIVER Driver,
    _Out_ PBOOLEAN PreferUasp
    )
{
    NTSTATUS status;
    WDFKEY hKey = NULL;
    ULONG pref = 1;
    DECLARE_CONST_UNICODE_STRING(name, L"UaspPreferred");

    *PreferUasp = TRUE;

    status = WdfDriverOpenParametersRegistryKey(
        Driver,
        KEY_READ,
        WDF_NO_OBJECT_ATTRIBUTES,
        &hKey);

    if (!NT_SUCCESS(status)) {
        return status;
    }

    if (NT_SUCCESS(WdfRegistryQueryULong(hKey, &name, &pref))) {
        *PreferUasp = (pref != 0);
    }

    WdfRegistryClose(hKey);
    return STATUS_SUCCESS;
}

static VOID
UsbAsRamByteRangeToLba(
    _In_ LONGLONG StartingOffset,
    _In_ ULONG Length,
    _In_ ULONG BytesPerSector,
    _Out_ PLONGLONG FirstLba,
    _Out_ PLONGLONG LastLbaInclusive
    )
{
    LONGLONG endInclusive;

    if (BytesPerSector == 0) {
        *FirstLba = 0;
        *LastLbaInclusive = 0;
        return;
    }

    if (Length == 0) {
        *FirstLba = StartingOffset / (LONGLONG)BytesPerSector;
        *LastLbaInclusive = *FirstLba;
        return;
    }

    endInclusive = StartingOffset + (LONGLONG)Length - 1;

    *FirstLba = StartingOffset / (LONGLONG)BytesPerSector;
    *LastLbaInclusive = endInclusive / (LONGLONG)BytesPerSector;
}

static VOID
UsbAsRamLogScatterGatherList(
    _In_ PSCATTER_GATHER_LIST SgList,
    _In_ LONGLONG FirstLba,
    _In_ LONGLONG LastLbaInclusive,
    _In_ BOOLEAN WriteToDevice
    )
{
    ULONG i;

    if (SgList == NULL) {
        return;
    }

    UsbAsRamTraceVerbose(
        "%s: %s DMA S/G elements=%u LBA [%lld..%lld]",
        __FUNCTION__,
        WriteToDevice ? "WRITE" : "READ",
        SgList->NumberOfElements,
        FirstLba,
        LastLbaInclusive);

    for (i = 0; i < SgList->NumberOfElements; ++i) {
        UsbAsRamTraceVerbose(
            "%s:   S/G[%u] PA=0x%I64x Len=%u",
            __FUNCTION__,
            i,
            SgList->Elements[i].Address.QuadPart,
            SgList->Elements[i].Length);
    }
}

BOOLEAN
UsbAsRamEvtProgramDma(
    _In_ WDFDMATRANSACTION DmaTransaction,
    _In_ WDFDEVICE Device,
    _In_ PVOID Context,
    _In_ WDF_DMA_DIRECTION Direction,
    _In_ PSCATTER_GATHER_LIST SgList
    )
{
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);
    ULONG totalLength = 0;
    ULONG i;
    BOOLEAN writeToDevice;

    UNREFERENCED_PARAMETER(Context);
    UNREFERENCED_PARAMETER(Direction);

    writeToDevice = (WdfDmaDirectionWriteToDevice == Direction);

    //
    // IRQL: EvtProgramDma runs at DISPATCH_LEVEL. No paged code, no faults.
    // Module 2 stub: log S/G elements, then complete DMA synchronously. Later modules
    // will issue USB/UASP or staged bulk transfers per element.
    //
    if (SgList != NULL) {
        for (i = 0; i < SgList->NumberOfElements; ++i) {
            totalLength += SgList->Elements[i].Length;
        }
    }

    UsbAsRamTraceVerbose(
        "%s: ProgramDMA %s totalLength=%u ctxSector=%lu",
        __FUNCTION__,
        writeToDevice ? "WRITE" : "READ",
        totalLength,
        ctx->LogicalSectorSize);

    UsbAsRamLogScatterGatherList(SgList, 0, 0, writeToDevice);

    (VOID)WdfDmaTransactionDmaCompletedWithLength(DmaTransaction, totalLength, STATUS_SUCCESS);
    return TRUE;
}

VOID
UsbAsRamEvtDmaTransactionComplete(
    _In_ WDFDMATRANSACTION DmaTransaction,
    _In_ WDFDEVICE Device,
    _In_ PVOID Context,
    _In_ NTSTATUS Status,
    _In_ BOOLEAN DeviceReleasedDma
    )
{
    WDFREQUEST request = static_cast<WDFREQUEST>(Context);
    ULONG_PTR information = 0;
    WDF_REQUEST_PARAMETERS params;
    WDF_REQUEST_PARAMETERS_INIT(&params);

    UNREFERENCED_PARAMETER(DeviceReleasedDma);

    if (request != NULL) {
        WdfRequestGetParameters(request, &params);
    }

    if (request != NULL && NT_SUCCESS(Status)) {
        if (params.Type == WdfRequestTypeRead) {
            information = params.Parameters.Read.Length;
        } else if (params.Type == WdfRequestTypeWrite) {
            information = params.Parameters.Write.Length;
        }
    }

    UsbAsRamTraceVerbose(
        "%s: DmaTransaction complete Status=0x%08X bytes=%Iu",
        __FUNCTION__,
        Status,
        information);

    if (request != NULL && NT_SUCCESS(Status) && params.Type == WdfRequestTypeRead) {
        UsbAsRamLz4DecompressAfterDmaIfNeeded(Device, request);
    }

    // For writes: perform USB bulk-out transfers asynchronously using WDF USB pipe requests.
    if (request != NULL && NT_SUCCESS(Status) && params.Type == WdfRequestTypeWrite) {
        PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);
        PIRP irp = WdfRequestWdmGetIrp(request);
        PMDL mdl = NULL;
        PVOID mapped = NULL;
        NTSTATUS writeStatus = STATUS_SUCCESS;

        if (ctx->UsbInitialized && ctx->BulkOutPipe != NULL && irp != NULL) {
            mdl = irp->MdlAddress;
            if (mdl == NULL) {
                writeStatus = STATUS_INVALID_PARAMETER;
            } else {
                // Attempt to map with UserMode flag as requested
                mapped = MmMapLockedPagesSpecifyCache(mdl, UserMode, MmCached, NULL, FALSE, NormalPagePriority);
                if (mapped == NULL) {
                    // Fall back to system VA mapping
                    mapped = MmGetSystemAddressForMdlSafe(mdl, NormalPagePriority | MdlMappingNoExecute);
                }
            }

            if (mapped != NULL) {
                // Create a small synthetic scatter/gather structure on the stack.
                // Note: this is a simplified representation; the SendBulkTransfers
                // routine expects Elements[].Address.QuadPart to contain a virtual
                // pointer in our simplified flow.
                struct SIMPLE_SG_LIST {
                    ULONG NumberOfElements;
                    struct {
                        LARGE_INTEGER Address;
                        ULONG Length;
                    } Elements[1];
                } sg;

                sg.NumberOfElements = 1;
                sg.Elements[0].Address.QuadPart = reinterpret_cast<ULONG_PTR>(mapped);
                sg.Elements[0].Length = (ULONG)information;

                writeStatus = UsbAsRamSendBulkTransfersFromSgList(Device, (PSCATTER_GATHER_LIST)&sg, TRUE, request);

                if (mapped != NULL && mdl != NULL) {
                    // If we mapped with MmMapLockedPages..., unmap
                    MmUnmapLockedPages(mapped, mdl);
                }
            }
        }

        if (!NT_SUCCESS(writeStatus)) {
            // Convert failure to offline status per requirements
            Status = STATUS_DEVICE_OFF_LINE;
        }
    }

    if (request != NULL) {
        WdfRequestCompleteWithInformation(request, Status, information);
    }

    WdfObjectDelete(DmaTransaction);
}

static NTSTATUS
UsbAsRamStartDmaForRequest(
    _In_ WDFDEVICE Device,
    _In_ WDFREQUEST Request,
    _In_ BOOLEAN WriteToDevice,
    _In_ LONGLONG FirstLba,
    _In_ LONGLONG LastLbaInclusive
    )
{
    NTSTATUS status;
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);
    WDFDMATRANSACTION dmaTransaction = NULL;
    WDF_DMA_TRANSACTION_CONFIG txnConfig;
    WDF_OBJECT_ATTRIBUTES txnAttributes;

    WDF_DMA_TRANSACTION_CONFIG_INIT(&txnConfig, UsbAsRamEvtDmaTransactionComplete);
    WDF_OBJECT_ATTRIBUTES_INIT(&txnAttributes);
    txnAttributes.ParentObject = Device;

    status = WdfDmaTransactionCreate(
        ctx->DmaEnabler,
        &txnConfig,
        &txnAttributes,
        &dmaTransaction);

    if (!NT_SUCCESS(status)) {
        UsbAsRamTraceError("%s: WdfDmaTransactionCreate failed: 0x%08X", __FUNCTION__, status);
        return status;
    }

    UsbAsRamTraceVerbose(
        "%s: DMA LBA [%lld..%lld] %s",
        __FUNCTION__,
        FirstLba,
        LastLbaInclusive,
        WriteToDevice ? "WRITE" : "READ");

    status = WdfDmaTransactionInitializeUsingRequest(
        dmaTransaction,
        Request,
        WriteToDevice);

    if (!NT_SUCCESS(status)) {
        UsbAsRamTraceError("%s: WdfDmaTransactionInitializeUsingRequest failed: 0x%08X",
            __FUNCTION__, status);
        WdfObjectDelete(dmaTransaction);
        return status;
    }

    status = WdfDmaTransactionExecute(
        dmaTransaction,
        reinterpret_cast<WDFCONTEXT>(Request));

    if (status == STATUS_PENDING) {
        return STATUS_PENDING;
    }

    if (!NT_SUCCESS(status)) {
        UsbAsRamTraceError("%s: WdfDmaTransactionExecute failed: 0x%08X", __FUNCTION__, status);
        WdfObjectDelete(dmaTransaction);
        return status;
    }

    //
    // STATUS_SUCCESS: EvtDmaTransactionComplete has already completed the WDFREQUEST.
    //
    return STATUS_SUCCESS;
}

static VOID
UsbAsRamForwardRequestToLower(
    _In_ WDFDEVICE Device,
    _In_ WDFREQUEST Request
    )
{
    WDFIOTARGET target = WdfDeviceGetIoTarget(Device);

    if (!NT_SUCCESS(WdfRequestFormatRequestUsingCurrentType(Request))) {
        WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
        return;
    }

    WdfRequestSend(Request, target, WDF_NO_SEND_OPTIONS);
}

VOID
UsbAsRamEvtIoDeviceControl(
    _In_ WDFQUEUE Queue,
    _In_ WDFREQUEST Request,
    _In_ size_t OutputBufferLength,
    _In_ size_t InputBufferLength,
    _In_ ULONG IoControlCode
    )
{
    WDFDEVICE device = WdfIoQueueGetDevice(Queue);
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(device);

    UNREFERENCED_PARAMETER(OutputBufferLength);
    UNREFERENCED_PARAMETER(InputBufferLength);

    if (IoControlCode == IOCTL_USBASRAM_REVERT_PMEM) {
        UsbAsRamWbFlushAll(device);
        ctx->RevertRequested = TRUE;
        UsbAsRamTraceInfo(
            "%s: IOCTL_USBASRAM_REVERT_PMEM — cache flushed; revert flag set "
            "(full stack fallback requires PnP/INF cooperation)",
            __FUNCTION__);
        WdfRequestCompleteWithInformation(Request, STATUS_SUCCESS, 0);
        return;
    }

    UsbAsRamForwardRequestToLower(device, Request);
}

VOID
UsbAsRamEvtIoRead(
    _In_ WDFQUEUE Queue,
    _In_ WDFREQUEST Request
    )
{
    WDFDEVICE device = WdfIoQueueGetDevice(Queue);
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(device);
    NTSTATUS status;
    ULONG length = 0;
    LONGLONG byteOffset = 0;
    LONGLONG firstLba = 0;
    LONGLONG lastLba = 0;
    PIRP irp = WdfRequestWdmGetIrp(Request);
    PIO_STACK_LOCATION sp = NULL;

    if (irp == NULL) {
        WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
        return;
    }

    sp = IoGetCurrentIrpStackLocation(irp);
    length = sp->Parameters.Read.Length;
    byteOffset = sp->Parameters.Read.ByteOffset.QuadPart;

    if (length == 0) {
        WdfRequestCompleteWithInformation(Request, STATUS_SUCCESS, 0);
        return;
    }

    if (ctx->RevertRequested) {
        UsbAsRamForwardRequestToLower(device, Request);
        return;
    }

    UsbAsRamByteRangeToLba(byteOffset, length, ctx->LogicalSectorSize, &firstLba, &lastLba);

    UsbAsRamTraceVerbose(
        "%s: READ offset=0x%I64x len=%lu -> LBA %lld..%lld",
        __FUNCTION__,
        byteOffset,
        length,
        firstLba,
        lastLba);

    status = UsbAsRamStartDmaForRequest(device, Request, FALSE, firstLba, lastLba);
    if (status == STATUS_PENDING) {
        return;
    }

    if (NT_SUCCESS(status)) {
        return;
    }

    WdfRequestCompleteWithInformation(Request, status, 0);
}

VOID
UsbAsRamEvtIoWrite(
    _In_ WDFQUEUE Queue,
    _In_ WDFREQUEST Request
    )
{
    WDFDEVICE device = WdfIoQueueGetDevice(Queue);
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(device);
    NTSTATUS status;
    ULONG length = 0;
    LONGLONG byteOffset = 0;
    LONGLONG firstLba = 0;
    LONGLONG lastLba = 0;
    PIRP irp = WdfRequestWdmGetIrp(Request);
    PIO_STACK_LOCATION sp = NULL;

    if (irp == NULL) {
        WdfRequestComplete(Request, STATUS_INVALID_PARAMETER);
        return;
    }

    sp = IoGetCurrentIrpStackLocation(irp);
    length = sp->Parameters.Write.Length;
    byteOffset = sp->Parameters.Write.ByteOffset.QuadPart;

    if (length == 0) {
        WdfRequestCompleteWithInformation(Request, STATUS_SUCCESS, 0);
        return;
    }

    if (ctx->RevertRequested) {
        UsbAsRamForwardRequestToLower(device, Request);
        return;
    }

    if (UsbAsRamWbTryStageWrite(device, Request)) {
        return;
    }

    {
        BOOLEAN lzDone = FALSE;

        (VOID)UsbAsRamProcessWriteWithLz4(device, Request, &lzDone);
        if (lzDone) {
            return;
        }
    }

    UsbAsRamByteRangeToLba(byteOffset, length, ctx->LogicalSectorSize, &firstLba, &lastLba);

    UsbAsRamTraceVerbose(
        "%s: WRITE offset=0x%I64x len=%lu -> LBA %lld..%lld",
        __FUNCTION__,
        byteOffset,
        length,
        firstLba,
        lastLba);

    status = UsbAsRamStartDmaForRequest(device, Request, TRUE, firstLba, lastLba);
    if (status == STATUS_PENDING) {
        return;
    }

    if (NT_SUCCESS(status)) {
        return;
    }

    WdfRequestCompleteWithInformation(Request, status, 0);
}

VOID
UsbAsRamEvtIoDefault(
    _In_ WDFQUEUE Queue,
    _In_ WDFREQUEST Request
    )
{
    WDFDEVICE device = WdfIoQueueGetDevice(Queue);

    UsbAsRamTraceVerbose("%s: forward request %p", __FUNCTION__, Request);

    UsbAsRamForwardRequestToLower(device, Request);
}
