/*++
Module Name: Compression.cpp
Abstract: Module 3 — LZ4 compress on write (4 KiB pages, padded to 4 KiB for USB alignment).
Environment: Kernel mode, KMDF
--*/

#include <ntddk.h>
#include <wdf.h>

#include "lz4.h"
#include "lz4hc.h"

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

static SIZE_T
UsbAsRamRoundUp4096(
    _In_ SIZE_T Value
    )
{
    return (Value + USR_PAGE_SIZE - 1) & ~(SIZE_T)(USR_PAGE_SIZE - 1);
}

NTSTATUS
UsbAsRamLz4Initialize(
    _In_ WDFDEVICE Device
    )
{
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);
    SIZE_T scratch;

    if (ctx->Lz4Initialized) {
        return STATUS_SUCCESS;
    }

    // For HC mode we need a larger scratch to hold streaming state and output buffers.
    scratch = (SIZE_T)LZ4_COMPRESSBOUND(USR_PAGE_SIZE) + USR_PAGE_SIZE +  sizeof(LZ4_streamHC_t);
    ctx->Lz4Scratch = static_cast<PUCHAR>(
        ExAllocatePool2(POOL_FLAG_NON_PAGED_NX, scratch, USR_POOL_TAG));

    if (ctx->Lz4Scratch == NULL) {
        UsbAsRamTraceError("%s: ExAllocatePool2(Lz4Scratch) failed", __FUNCTION__);
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    ctx->Lz4ScratchBytes = scratch;
    ctx->Lz4Enabled = TRUE;
    ctx->Lz4Initialized = TRUE;

    // Initialize HC stream state at front of scratch
    if (ctx->Lz4Scratch) {
        LZ4_initStreamHC(ctx->Lz4Scratch, sizeof(LZ4_streamHC_t));
        // default to high compression level
        LZ4_resetStreamHC_fast(reinterpret_cast<LZ4_streamHC_t*>(ctx->Lz4Scratch), LZ4HC_CLEVEL_MAX);
    }

    UsbAsRamTraceInfo("%s: LZ4 scratch=%Iu bytes", __FUNCTION__, scratch);
    return STATUS_SUCCESS;
}

VOID
UsbAsRamLz4Cleanup(
    _In_ WDFDEVICE Device
    )
{
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);

    if (ctx->Lz4Scratch != NULL) {
        ExFreePoolWithTag(ctx->Lz4Scratch, USR_POOL_TAG);
        ctx->Lz4Scratch = NULL;
    }

    ctx->Lz4ScratchBytes = 0;
    ctx->Lz4Initialized = FALSE;
    ctx->Lz4Enabled = FALSE;
}

NTSTATUS
UsbAsRamProcessWriteWithLz4(
    _In_ WDFDEVICE Device,
    _In_ WDFREQUEST Request,
    _Out_ PBOOLEAN RequestCompleted
    )
{
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);

    *RequestCompleted = FALSE;
    PIRP irp = WdfRequestWdmGetIrp(Request);
    PIO_STACK_LOCATION sp = NULL;
    PVOID userBuf = NULL;
    PMDL mdl = NULL;
    LONGLONG byteOffset = 0;
    ULONG length = 0;
    PUCHAR sysVa = NULL;
    ULONG page;
    ULONG numPages;
    PUCHAR dst;
    int comp;
    SIZE_T padded;

    *RequestCompleted = FALSE;

    if (!ctx->Lz4Initialized || !ctx->Lz4Enabled) {
        return STATUS_SUCCESS;
    }

    if (irp == NULL) {
        return STATUS_SUCCESS;
    }

    sp = IoGetCurrentIrpStackLocation(irp);
    length = sp->Parameters.Write.Length;
    byteOffset = sp->Parameters.Write.ByteOffset.QuadPart;

    if (((ULONG_PTR)byteOffset & (USR_PAGE_SIZE - 1)) != 0 ||
        (length & (USR_PAGE_SIZE - 1)) != 0 ||
        length == 0) {
        UsbAsRamTraceVerbose(
            "%s: skip LZ4 (need 4 KiB-aligned offset/length) off=%I64x len=%lu",
            __FUNCTION__,
            byteOffset,
            length);
        return STATUS_SUCCESS;
    }

    mdl = irp->MdlAddress;
    if (mdl == NULL) {
        return STATUS_SUCCESS;
    }

    sysVa = static_cast<PUCHAR>(MmGetSystemAddressForMdlSafe(mdl, NormalPagePriority | MdlMappingNoExecute));
    if (sysVa == NULL) {
        WdfRequestComplete(Request, STATUS_INSUFFICIENT_RESOURCES);
        *RequestCompleted = TRUE;
        return STATUS_SUCCESS;
    }

    numPages = length / USR_PAGE_SIZE;
    // Use LZ4 HC streaming API for higher compression at cost of CPU.
    dst = ctx->Lz4Scratch + sizeof(LZ4_streamHC_t);
    LZ4_streamHC_t* hc = reinterpret_cast<LZ4_streamHC_t*>(ctx->Lz4Scratch);

    for (page = 0; page < numPages; ++page) {
        PUCHAR srcPage = sysVa + (page * USR_PAGE_SIZE);

        comp = LZ4_compress_HC_continue(
            hc,
            reinterpret_cast<const char*>(srcPage),
            reinterpret_cast<char*>(dst),
            (int)USR_PAGE_SIZE,
            (int)LZ4_COMPRESSBOUND(USR_PAGE_SIZE));

        if (comp <= 0) {
            UsbAsRamTraceError("%s: LZ4_compress_HC_continue failed page=%lu", __FUNCTION__, page);
            WdfRequestComplete(Request, STATUS_INTERNAL_ERROR);
            *RequestCompleted = TRUE;
            return STATUS_SUCCESS;
        }

        padded = UsbAsRamRoundUp4096(static_cast<SIZE_T>(comp));
        if (padded > ctx->Lz4ScratchBytes - sizeof(LZ4_streamHC_t)) {
            WdfRequestComplete(Request, STATUS_INTERNAL_ERROR);
            *RequestCompleted = TRUE;
            return STATUS_SUCCESS;
        }

        if (padded > static_cast<SIZE_T>(comp)) {
            RtlZeroMemory(dst + comp, padded - static_cast<SIZE_T>(comp));
        }

        UsbAsRamTraceInfo(
            "%s: HC page %lu/%lu compressed=%d paddedUsb=%Iu (4 KiB aligned)",
            __FUNCTION__,
            page + 1,
            numPages,
            comp,
            padded);
    }

    WdfRequestCompleteWithInformation(Request, STATUS_SUCCESS, length);
    *RequestCompleted = TRUE;
    return STATUS_SUCCESS;
}

VOID
UsbAsRamLz4DecompressAfterDmaIfNeeded(
    _In_ WDFDEVICE Device,
    _In_ WDFREQUEST Request
    )
{
    UNREFERENCED_PARAMETER(Request);

    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);

    if (!ctx->Lz4Initialized || !ctx->Lz4Enabled) {
        return;
    }

    //
    // DMA stub leaves plaintext in the buffer; real USB path would supply LZ4 payloads.
    //
    UsbAsRamTraceVerbose("%s: decompress skipped (no compressed payload in DMA stub)", __FUNCTION__);
}
