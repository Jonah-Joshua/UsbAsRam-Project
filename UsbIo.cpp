/*++
Module Name: UsbIo.cpp
Abstract: Minimal USB wiring for bulk endpoints. Provides safe placeholders so the
          driver builds and later can be extended to send URBs to WdfUsbTargetPipe.
Environment: Kernel mode, KMDF
--*/

#include <ntddk.h>
#include <wdf.h>
#include <wdfusb.h>

#include "UsbAsRam.h"

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

NTSTATUS
UsbAsRamUsbInitialize(
    _In_ WDFDEVICE Device
    )
{
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);

    UNREFERENCED_PARAMETER(Device);

    // Placeholder: in a full implementation we would call WdfUsbTargetDeviceCreate,
    // enumerate interfaces, find bulk endpoints and save WDFUSBPIPE handles.
    ctx->UsbInitialized = FALSE;
    ctx->UsbDevice = NULL;
    ctx->BulkInPipe = NULL;
    ctx->BulkOutPipe = NULL;

    UsbAsRamTraceInfo("%s: UsbInitialize placeholder (no pipes opened)", __FUNCTION__);
    return STATUS_SUCCESS;
}

// Watchdog timer callback: scan pending requests and reset pipe if any older than 150ms
VOID
UsbAsRamHwWatchdogTimerFunc(
    _In_ WDFTIMER Timer
    )
{
    WDFDEVICE device = WdfTimerGetParentObject(Timer);
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(device);

    if (ctx == NULL || !ctx->UsbInitialized || ctx->BulkOutPipe == NULL) {
        return;
    }

    LARGE_INTEGER now;
    KeQuerySystemTime(&now);

    WdfSpinLockAcquire(ctx->PendingLock);
    PLIST_ENTRY entry = ctx->PendingList.Flink;
    while (entry != &ctx->PendingList) {
        PUSBASRAM_CHILD_REQUEST_CONTEXT c = CONTAINING_RECORD(entry, USBASRAM_CHILD_REQUEST_CONTEXT, ListEntry);
        LONGLONG ageMs = (now.QuadPart - c->StartTime.QuadPart) / 10000; // 100ns to ms
        if (ageMs > 150) {
            UsbAsRamTraceError("%s: pending USB child request older than %lld ms, resetting pipe", __FUNCTION__, ageMs);
            WdfSpinLockRelease(ctx->PendingLock);
            (VOID)WdfUsbTargetPipeResetSynchronously(ctx->BulkOutPipe, NULL, NULL);
            WdfSpinLockAcquire(ctx->PendingLock);
            // After reset, continue scanning but do not attempt to cancel individual requests here.
        }
        entry = entry->Flink;
    }
    WdfSpinLockRelease(ctx->PendingLock);
}

VOID
UsbAsRamUsbCleanup(
    _In_ WDFDEVICE Device
    )
{
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);

    if (ctx->UsbDevice != NULL) {
        WdfObjectDelete(ctx->UsbDevice);
        ctx->UsbDevice = NULL;
    }
    ctx->BulkInPipe = NULL;
    ctx->BulkOutPipe = NULL;
    ctx->UsbInitialized = FALSE;

    if (ctx->HwWatchdogTimer != NULL) {
        WdfTimerStop(ctx->HwWatchdogTimer, TRUE);
        WdfObjectDelete(ctx->HwWatchdogTimer);
        ctx->HwWatchdogTimer = NULL;
    }

    if (ctx->PendingLock != NULL) {
        WdfObjectDelete(ctx->PendingLock);
        ctx->PendingLock = NULL;
    }

    UsbAsRamTraceInfo("%s: UsbCleanup placeholder", __FUNCTION__);
}

EVT_WDF_REQUEST_COMPLETION_ROUTINE UsbAsRamEvtUsbRequestComplete;

NTSTATUS
UsbAsRamSendBulkTransfersFromSgList(
    _In_ WDFDEVICE Device,
    _In_ PSCATTER_GATHER_LIST SgList,
    _In_ BOOLEAN WriteToDevice,
    _In_ WDFREQUEST OriginalRequest
    )
{
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);
    ULONG i;

    if (SgList == NULL || !ctx->UsbInitialized || ctx->BulkOutPipe == NULL) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    // For each SG element, split into 64KB child requests, allocate a child buffer
    // with ExAllocatePoolWithTag('RamU'), copy compressed data into it, and
    // format a WDFREQUEST for the pipe with completion callback.

    for (i = 0; i < SgList->NumberOfElements; ++i) {
        PSCATTER_GATHER_ELEMENT el = &SgList->Elements[i];
        ULONG remaining = el->Length;
        PUCHAR ptr = reinterpret_cast<PUCHAR>(el->Address.QuadPart);

        while (remaining > 0) {
            ULONG chunk = (remaining > 64 * 1024) ? (64 * 1024) : remaining;

            // Allocate child buffer
            PVOID childBuf = ExAllocatePoolWithTag(NonPagedPoolNx, chunk, 'RamU');
            if (childBuf == NULL) {
                return STATUS_INSUFFICIENT_RESOURCES;
            }

            RtlCopyMemory(childBuf, ptr, chunk);

            // Create WDF memory object for the child buffer
            WDF_OBJECT_ATTRIBUTES memAttr;
            WDF_OBJECT_ATTRIBUTES_INIT(&memAttr);

            WDFMEMORY childMem;
            NTSTATUS status = WdfMemoryCreatePreallocated(&memAttr, childBuf, chunk, &childMem);
            if (!NT_SUCCESS(status)) {
                ExFreePoolWithTag(childBuf, 'RamU');
                return status;
            }

            // Create a request for the pipe
            WDFREQUEST pipeReq;
            status = WdfRequestCreate(WDF_NO_OBJECT_ATTRIBUTES, WdfUsbTargetPipeGetIoTarget(ctx->BulkOutPipe), &pipeReq);
            if (!NT_SUCCESS(status)) {
                WdfObjectDelete(childMem);
                ExFreePoolWithTag(childBuf, 'RamU');
                return status;
            }

            // Attach child context
            PUSBASRAM_CHILD_REQUEST_CONTEXT childCtx = (PUSBASRAM_CHILD_REQUEST_CONTEXT)ExAllocatePoolWithTag(NonPagedPoolNx, sizeof(USBASRAM_CHILD_REQUEST_CONTEXT), 'RamU');
            if (childCtx == NULL) {
                WdfObjectDelete(pipeReq);
                WdfObjectDelete(childMem);
                ExFreePoolWithTag(childBuf, 'RamU');
                return STATUS_INSUFFICIENT_RESOURCES;
            }
            RtlZeroMemory(childCtx, sizeof(*childCtx));
            childCtx->Buffer = childBuf;
            childCtx->BufferSize = chunk;
            childCtx->Memory = childMem;
            childCtx->Owner = Device;
            KeQuerySystemTime(&childCtx->StartTime);

            childCtx->Request = pipeReq;

            // Insert into pending list
            WdfSpinLockAcquire(ctx->PendingLock);
            InsertTailList(&ctx->PendingList, &childCtx->ListEntry);
            ctx->PendingCount++;
            WdfSpinLockRelease(ctx->PendingLock);

            // Format request for write
            WdfUsbTargetPipeFormatRequestForWrite(ctx->BulkOutPipe, pipeReq, childMem, NULL);

            // Set completion callback and context
            WDF_REQUEST_SEND_OPTIONS options;
            WDF_REQUEST_SEND_OPTIONS_INIT(&options, WDF_REQUEST_SEND_OPTION_ASYNC);

            WdfRequestSetCompletionRoutine(pipeReq, UsbAsRamEvtUsbRequestComplete, childCtx);

            if (!WdfRequestSend(pipeReq, WdfUsbTargetPipeGetIoTarget(ctx->BulkOutPipe), &options)) {
                // Sending failed; cleanup
                WdfSpinLockAcquire(ctx->PendingLock);
                RemoveEntryList(&childCtx->ListEntry);
                ctx->PendingCount--;
                WdfSpinLockRelease(ctx->PendingLock);

                WdfObjectDelete(pipeReq);
                WdfObjectDelete(childMem);
                ExFreePoolWithTag(childBuf, 'RamU');
                ExFreePoolWithTag(childCtx, 'RamU');
                return STATUS_UNSUCCESSFUL;
            }

            ptr += chunk;
            remaining -= chunk;
        }
    }

    return STATUS_SUCCESS;
}

VOID
UsbAsRamEvtUsbRequestComplete(
    _In_ WDFREQUEST Request,
    _In_ WDFIOTARGET Target,
    _In_ PWDF_REQUEST_COMPLETION_PARAMS Params,
    _In_ WDFCONTEXT Context
    )
{
    UNREFERENCED_PARAMETER(Target);
    PUSBASRAM_CHILD_REQUEST_CONTEXT childCtx = (PUSBASRAM_CHILD_REQUEST_CONTEXT)Context;
    PUSBASRAM_DEVICE_CONTEXT ctx;

    if (childCtx == NULL) {
        return;
    }

    // Remove from pending list
    ctx = UsbAsRamDeviceGetContext((WDFDEVICE)childCtx->Owner);
    if (ctx != NULL) {
        WdfSpinLockAcquire(ctx->PendingLock);
        RemoveEntryList(&childCtx->ListEntry);
        ctx->PendingCount--;
        WdfSpinLockRelease(ctx->PendingLock);
    }

    // Free resources
    if (childCtx->Memory != NULL) {
        WdfObjectDelete(childCtx->Memory);
    }
    if (childCtx->Buffer != NULL) {
        ExFreePoolWithTag(childCtx->Buffer, 'RamU');
    }

    ExFreePoolWithTag(childCtx, 'RamU');

    // Delete the request object created for the pipe
    WdfObjectDelete(Request);
    return;
}
