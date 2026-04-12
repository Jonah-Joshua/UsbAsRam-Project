/*++
Module Name: WbCache.cpp
Abstract: Module 4 — 64 MiB NonPagedPoolNx write-back staging, direct-mapped 4 KiB slots,
          WdfWorkItem flush (stub: trace + clear dirty).
Environment: Kernel mode, KMDF
--*/

#include <ntddk.h>
#include <wdf.h>

#include "UsbAsRam.h"

extern "C" {

EVT_WDF_WORKITEM UsbAsRamWbFlushWorkItem;

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
UsbAsRamWbFlushSlotLocked(
    _In_ PUSBASRAM_DEVICE_CONTEXT Ctx,
    _In_ ULONG Slot
    );

VOID
UsbAsRamWbFlushWorkItem(
    _In_ WDFWORKITEM WorkItem
    )
{
    WDFDEVICE device = static_cast<WDFDEVICE>(WdfWorkItemGetParentObject(WorkItem));
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(device);

    UsbAsRamWbFlushAll(device);

    InterlockedExchange(&ctx->WbFlushScheduled, 0);
}

VOID
UsbAsRamWbFlushAll(
    _In_ WDFDEVICE Device
    )
{
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);
    ULONG i;

    if (!ctx->WbInitialized || ctx->WbBuffer == NULL || ctx->WbMeta == NULL) {
        return;
    }

    WdfSpinLockAcquire(ctx->WbLock);
    for (i = 0; i < USR_WB_SLOT_COUNT; ++i) {
        if (ctx->WbMeta[i].Dirty) {
            UsbAsRamWbFlushSlotLocked(ctx, i);
        }
    }
    WdfSpinLockRelease(ctx->WbLock);
}

static VOID
UsbAsRamWbFlushSlotLocked(
    _In_ PUSBASRAM_DEVICE_CONTEXT Ctx,
    _In_ ULONG Slot
    )
{
    UNREFERENCED_PARAMETER(Ctx);

    UsbAsRamTraceInfo(
        "%s: [stub] flush slot=%lu pageBase=0x%I64x to USB (contiguous 4 KiB)",
        __FUNCTION__,
        Slot,
        Ctx->WbMeta[Slot].PageBase);

    Ctx->WbMeta[Slot].Dirty = 0;
}

NTSTATUS
UsbAsRamWbInitialize(
    _In_ WDFDEVICE Device
    )
{
    NTSTATUS status;
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);
    WDF_OBJECT_ATTRIBUTES wiAttr;
    WDF_OBJECT_ATTRIBUTES slAttr;
    WDF_WORKITEM_CONFIG wiConfig;
    SIZE_T metaBytes = USR_WB_SLOT_COUNT * sizeof(USBASRAM_WB_SLOT);

    if (ctx->WbInitialized) {
        return STATUS_SUCCESS;
    }

    WDF_OBJECT_ATTRIBUTES_INIT(&slAttr);
    status = WdfSpinLockCreate(&slAttr, &ctx->WbLock);
    if (!NT_SUCCESS(status)) {
        UsbAsRamTraceError("%s: WdfSpinLockCreate failed 0x%08X", __FUNCTION__, status);
        return status;
    }

    ctx->WbBuffer = static_cast<PUCHAR>(
        ExAllocatePool2(POOL_FLAG_NON_PAGED_NX, USR_WB_CACHE_BYTES, USR_POOL_TAG));
    ctx->WbMeta = static_cast<PUSBASRAM_WB_SLOT>(
        ExAllocatePool2(POOL_FLAG_NON_PAGED_NX, metaBytes, USR_POOL_TAG));

    if (ctx->WbBuffer == NULL || ctx->WbMeta == NULL) {
        UsbAsRamTraceError("%s: cache/meta allocation failed", __FUNCTION__);
        if (ctx->WbBuffer) {
            ExFreePoolWithTag(ctx->WbBuffer, USR_POOL_TAG);
            ctx->WbBuffer = NULL;
        }
        if (ctx->WbMeta) {
            ExFreePoolWithTag(ctx->WbMeta, USR_POOL_TAG);
            ctx->WbMeta = NULL;
        }
        WdfObjectDelete(ctx->WbLock);
        ctx->WbLock = NULL;
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    RtlZeroMemory(ctx->WbMeta, metaBytes);
    {
        ULONG i;
        for (i = 0; i < USR_WB_SLOT_COUNT; ++i) {
            ctx->WbMeta[i].PageBase = -1LL;
        }
    }

    WDF_WORKITEM_CONFIG_INIT(&wiConfig, UsbAsRamWbFlushWorkItem);
    WDF_OBJECT_ATTRIBUTES_INIT(&wiAttr);
    wiAttr.ParentObject = Device;

    status = WdfWorkItemCreate(&wiConfig, &wiAttr, &ctx->WbFlushWorkItem);
    if (!NT_SUCCESS(status)) {
        UsbAsRamTraceError("%s: WdfWorkItemCreate failed 0x%08X", __FUNCTION__, status);
        ExFreePoolWithTag(ctx->WbBuffer, USR_POOL_TAG);
        ctx->WbBuffer = NULL;
        ExFreePoolWithTag(ctx->WbMeta, USR_POOL_TAG);
        ctx->WbMeta = NULL;
        WdfObjectDelete(ctx->WbLock);
        ctx->WbLock = NULL;
        return status;
    }

    ctx->WbFlushScheduled = 0;
    ctx->WbInitialized = TRUE;

    UsbAsRamTraceInfo("%s: 128 MiB write-back cache ready (%lu slots)", __FUNCTION__, USR_WB_SLOT_COUNT);
    return STATUS_SUCCESS;
}

VOID
UsbAsRamWbCleanup(
    _In_ WDFDEVICE Device
    )
{
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);

    if (ctx->WbFlushWorkItem != NULL) {
        WdfObjectDelete(ctx->WbFlushWorkItem);
        ctx->WbFlushWorkItem = NULL;
    }

    if (ctx->WbLock != NULL) {
        WdfObjectDelete(ctx->WbLock);
        ctx->WbLock = NULL;
    }

    if (ctx->WbBuffer != NULL) {
        ExFreePoolWithTag(ctx->WbBuffer, USR_POOL_TAG);
        ctx->WbBuffer = NULL;
    }

    if (ctx->WbMeta != NULL) {
        ExFreePoolWithTag(ctx->WbMeta, USR_POOL_TAG);
        ctx->WbMeta = NULL;
    }

    ctx->WbInitialized = FALSE;
}

BOOLEAN
UsbAsRamWbTryStageWrite(
    _In_ WDFDEVICE Device,
    _In_ WDFREQUEST Request
    )
{
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);
    PIRP irp = WdfRequestWdmGetIrp(Request);
    PIO_STACK_LOCATION sp = NULL;
    ULONG length = 0;
    LONGLONG byteOffset = 0;
    PMDL mdl = NULL;
    PUCHAR sysVa = NULL;
    PUCHAR dst = NULL;
    ULONG slot = 0;
    LONGLONG pageBase = 0;

    if (!ctx->WbInitialized || ctx->RevertRequested) {
        return FALSE;
    }

    if (irp == NULL) {
        return FALSE;
    }

    sp = IoGetCurrentIrpStackLocation(irp);
    length = sp->Parameters.Write.Length;
    byteOffset = sp->Parameters.Write.ByteOffset.QuadPart;

    if (length == 0 || length > USR_WB_SMALL_WRITE_MAX) {
        return FALSE;
    }

    mdl = irp->MdlAddress;
    if (mdl == NULL) {
        return FALSE;
    }

    sysVa = static_cast<PUCHAR>(MmGetSystemAddressForMdlSafe(mdl, NormalPagePriority | MdlMappingNoExecute));
    if (sysVa == NULL) {
        return FALSE;
    }

    pageBase = byteOffset & ~((LONGLONG)USR_PAGE_SIZE - 1);
    slot = (ULONG)((byteOffset / USR_PAGE_SIZE) % USR_WB_SLOT_COUNT);

    dst = ctx->WbBuffer + (SIZE_T)slot * USR_PAGE_SIZE;

    WdfSpinLockAcquire(ctx->WbLock);

    if (ctx->WbMeta[slot].PageBase != pageBase) {
        if (ctx->WbMeta[slot].Dirty) {
            UsbAsRamWbFlushSlotLocked(ctx, slot);
        }
        ctx->WbMeta[slot].PageBase = pageBase;
    }

    if (length == USR_PAGE_SIZE) {
        RtlCopyMemory(dst, sysVa, USR_PAGE_SIZE);
    } else {
        ULONG off = (ULONG)(byteOffset - pageBase);
        RtlCopyMemory(dst + off, sysVa, length);
    }

    ctx->WbMeta[slot].Dirty = 1;

    WdfSpinLockRelease(ctx->WbLock);

    WdfRequestCompleteWithInformation(Request, STATUS_SUCCESS, length);

    if (InterlockedCompareExchange(&ctx->WbFlushScheduled, 1, 0) == 0) {
        WdfWorkItemEnqueue(ctx->WbFlushWorkItem);
    }

    UsbAsRamTraceVerbose(
        "%s: staged small write off=0x%I64x len=%lu slot=%lu",
        __FUNCTION__,
        byteOffset,
        length,
        slot);

    return TRUE;
}
