/*++
Module Name: DeviceSetup.cpp
Abstract: Module 1 — FDO filter setup, target USB VID/PID match, and registration of
          GUID_DEVINTERFACE_MEMORY_ARRAY. Volume / usbstor bypass is staged here with
          explicit TODOs for later IOCTL and PnP policy work.
Environment: Kernel mode, Windows 11, KMDF
--*/

#include <ntddk.h>
#include <wdf.h>

#include <initguid.h>

#include "UsbAsRam.h"

//
// Memory array device interface (same semantic as user-mode GUID_DEVINTERFACE_MEMORY_ARRAY).
// https://learn.microsoft.com/en-us/windows-hardware/drivers/install/guid-devinterface-memory-array
//
DEFINE_GUID(GUID_DEVINTERFACE_MEMORY_ARRAY,
    0x3f8e0f2e, 0x9492, 0x4bfc, 0xb4, 0x73, 0xf0, 0x59, 0x6d, 0x4d, 0x22, 0x40);

extern "C" {

EVT_WDF_DRIVER_DEVICE_ADD UsbAsRamEvtDeviceAdd;
EVT_WDF_DEVICE_PREPARE_HARDWARE UsbAsRamEvtDevicePrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE UsbAsRamEvtDeviceReleaseHardware;

}

// Watchdog timer callback declared in UsbAsRam.h / implemented in UsbIo.cpp
EVT_WDF_TIMER UsbAsRamHwWatchdogTimerFunc;

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

static NTSTATUS
UsbAsRamReadTargetIdsFromParameters(
    _In_ WDFDRIVER Driver,
    _Out_ PULONG VendorId,
    _Out_ PULONG ProductId
    );

static NTSTATUS
UsbAsRamParseVidPidFromHardwareId(
    _In_ PCUNICODE_STRING HardwareId,
    _Out_ PULONG VendorId,
    _Out_ PULONG ProductId
    );

static NTSTATUS
UsbAsRamRegisterMemoryArrayInterface(
    _In_ WDFDEVICE Device
    );

NTSTATUS
UsbAsRamEvtDeviceAdd(
    _In_ WDFDRIVER Driver,
    _Inout_ PWDFDEVICE_INIT DeviceInit
    )
{
    NTSTATUS status;
    WDF_OBJECT_ATTRIBUTES attributes;
    WDFDEVICE device;
    PUSBASRAM_DEVICE_CONTEXT ctx;
    WDF_PNPPOWER_EVENT_CALLBACKS pnpCallbacks;

    UNREFERENCED_PARAMETER(Driver);

    //
    // Filter driver: sit above the PDO stack for the device class this INF attaches to.
    //
    WdfFdoInitSetFilter(DeviceInit);

    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpCallbacks);
    pnpCallbacks.EvtDevicePrepareHardware = UsbAsRamEvtDevicePrepareHardware;
    pnpCallbacks.EvtDeviceReleaseHardware = UsbAsRamEvtDeviceReleaseHardware;
    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpCallbacks);

    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, USBASRAM_DEVICE_CONTEXT);
    attributes.ExecutionLevel = WdfExecutionLevelPassive;

    status = WdfDeviceCreate(&DeviceInit, &attributes, &device);
    if (!NT_SUCCESS(status)) {
        UsbAsRamTraceError("%s: WdfDeviceCreate failed: 0x%08X", __FUNCTION__, status);
        return status;
    }

    ctx = UsbAsRamDeviceGetContext(device);
    RtlZeroMemory(ctx, sizeof(*ctx));
    ctx->WdfDevice = device;
    ctx->ExpectedVendorId = USR_DEFAULT_VID;
    ctx->ExpectedProductId = USR_DEFAULT_PID;
    ctx->IsTargetUsbDevice = FALSE;
    ctx->MemoryArrayInterfaceRegistered = FALSE;
    ctx->InterceptVolumeMountRequested = FALSE;
    ctx->DmaEnabler = NULL;
    ctx->IoQueue = NULL;
    ctx->LogicalSectorSize = USR_BYTES_PER_SECTOR;
    ctx->MaxXferPerTransaction = USR_MAX_DMA_TRANSFER_LENGTH;
    ctx->DmaInitialized = FALSE;
    ctx->IoQueueCreated = FALSE;
    ctx->PreferUaspPath = FALSE;
    ctx->Lz4Initialized = FALSE;
    ctx->Lz4Enabled = FALSE;
    ctx->Lz4Scratch = NULL;
    ctx->Lz4ScratchBytes = 0;
    ctx->WbInitialized = FALSE;
    ctx->WbBuffer = NULL;
    ctx->WbMeta = NULL;
    ctx->WbLock = NULL;
    ctx->WbFlushWorkItem = NULL;
    ctx->WbFlushScheduled = 0;
    ctx->RevertRequested = FALSE;

    // Enforce strict hardware lock for the specific ITE controller only.
    // Ignore optional registry overrides to avoid accidental interception of other devices.
    (VOID)UsbAsRamReadTargetIdsFromParameters(
        WdfDeviceGetDriver(device),
        &ctx->ExpectedVendorId,
        &ctx->ExpectedProductId);
    ctx->ExpectedVendorId = USR_DEFAULT_VID;
    ctx->ExpectedProductId = USR_DEFAULT_PID;
    UsbAsRamTraceInfo("%s: Enforcing strict hardware lock to USB\\VID_%04X&PID_%04X",
        __FUNCTION__, ctx->ExpectedVendorId, ctx->ExpectedProductId);

    UsbAsRamTraceInfo("%s: Filter device created, expecting VID=0x%04X PID=0x%04X",
        __FUNCTION__, ctx->ExpectedVendorId, ctx->ExpectedProductId);

    return STATUS_SUCCESS;
}

NTSTATUS
UsbAsRamEvtDevicePrepareHardware(
    _In_ WDFDEVICE Device,
    _In_ WDFCMRESOURCELIST ResourcesRaw,
    _In_ WDFCMRESOURCELIST ResourcesTranslated
    )
{
    NTSTATUS status;
    PUSBASRAM_DEVICE_CONTEXT ctx;
    UNICODE_STRING valueName;
    WCHAR buffer[256];
    UNICODE_STRING hardwareId;
    ULONG vid = 0;
    ULONG pid = 0;

    UNREFERENCED_PARAMETER(ResourcesRaw);
    UNREFERENCED_PARAMETER(ResourcesTranslated);

    ctx = UsbAsRamDeviceGetContext(Device);

    RtlInitUnicodeString(&valueName, L"HardwareID");
    hardwareId.Buffer = buffer;
    hardwareId.MaximumLength = sizeof(buffer);

    status = WdfDeviceAllocAndQueryProperty(
        Device,
        DevicePropertyHardwareID,
        NonPagedPoolNx,
        USR_POOL_TAG,
        &hardwareId);

    if (!NT_SUCCESS(status)) {
        UsbAsRamTraceError("%s: WdfDeviceAllocAndQueryProperty(HardwareID) failed: 0x%08X",
            __FUNCTION__, status);
        return status;
    }

    UsbAsRamTraceVerbose("%s: HardwareID=%wZ", __FUNCTION__, &hardwareId);

    status = UsbAsRamParseVidPidFromHardwareId(&hardwareId, &vid, &pid);

    if (hardwareId.Buffer != NULL) {
        ExFreePoolWithTag(hardwareId.Buffer, USR_POOL_TAG);
        hardwareId.Buffer = NULL;
    }

    if (!NT_SUCCESS(status)) {
        UsbAsRamTraceVerbose("%s: VID/PID not parsed from HardwareID (not a USB-style ID): 0x%08X",
            __FUNCTION__, status);
        ctx->VendorId = 0;
        ctx->ProductId = 0;
        ctx->IsTargetUsbDevice = FALSE;
        return STATUS_SUCCESS;
    }

    ctx->VendorId = vid;
    ctx->ProductId = pid;

    if (vid == ctx->ExpectedVendorId && pid == ctx->ExpectedProductId) {
        ctx->IsTargetUsbDevice = TRUE;
        ctx->InterceptVolumeMountRequested = TRUE;
        UsbAsRamTraceInfo("%s: Target USB device matched VID=0x%04X PID=0x%04X",
            __FUNCTION__, vid, pid);

        status = UsbAsRamRegisterMemoryArrayInterface(Device);
        if (!NT_SUCCESS(status)) {
            UsbAsRamTraceError("%s: UsbAsRamRegisterMemoryArrayInterface failed: 0x%08X",
                __FUNCTION__, status);
            return status;
        }

        //
        // TODO (later modules): prevent volume mount / usbstor binding for this PDO.
        // Typical approaches: disk-class upper filter IOCTL handling, driver replacement
        // on USBSTOR, or policy in INF + exclusive access — depends on deployment model.
        //
        UsbAsRamTraceInfo("%s: InterceptVolumeMountRequested=TRUE (mount bypass not yet implemented)",
            __FUNCTION__);

        status = UsbAsRamModule2Initialize(Device);
        if (!NT_SUCCESS(status)) {
            UsbAsRamTraceError("%s: UsbAsRamModule2Initialize failed: 0x%08X",
                __FUNCTION__, status);
            return status;
        }
    } else {
        ctx->IsTargetUsbDevice = FALSE;
        UsbAsRamTraceVerbose("%s: Non-target device VID=0x%04X PID=0x%04X",
            __FUNCTION__, vid, pid);
    }

    return STATUS_SUCCESS;
}

NTSTATUS
UsbAsRamEvtDeviceReleaseHardware(
    _In_ WDFDEVICE Device,
    _In_ WDFCMRESOURCELIST ResourcesTranslated
    )
{
    UNREFERENCED_PARAMETER(ResourcesTranslated);

    UsbAsRamTraceVerbose("%s", __FUNCTION__);

    UsbAsRamModule2Cleanup(Device);

    return STATUS_SUCCESS;
}

static NTSTATUS
UsbAsRamReadTargetIdsFromParameters(
    _In_ WDFDRIVER Driver,
    _Out_ PULONG VendorId,
    _Out_ PULONG ProductId
    )
{
    NTSTATUS status;
    WDFKEY hKey = NULL;
    ULONG vid = USR_DEFAULT_VID;
    ULONG pid = USR_DEFAULT_PID;
    DECLARE_CONST_UNICODE_STRING(vidName, L"VendorId");
    DECLARE_CONST_UNICODE_STRING(pidName, L"ProductId");

    status = WdfDriverOpenParametersRegistryKey(
        Driver,
        KEY_READ,
        WDF_NO_OBJECT_ATTRIBUTES,
        &hKey);

    if (!NT_SUCCESS(status)) {
        UsbAsRamTraceVerbose("%s: WdfDriverOpenParametersRegistryKey failed 0x%08X, using defaults",
            __FUNCTION__, status);
        *VendorId = vid & 0xFFFF;
        *ProductId = pid & 0xFFFF;
        return STATUS_SUCCESS;
    }

    if (NT_SUCCESS(WdfRegistryQueryULong(hKey, &vidName, &vid))) {
        // optional override
    }
    if (NT_SUCCESS(WdfRegistryQueryULong(hKey, &pidName, &pid))) {
        // optional override
    }

    WdfRegistryClose(hKey);

    *VendorId = vid & 0xFFFF;
    *ProductId = pid & 0xFFFF;

    UsbAsRamTraceVerbose("%s: Parameters VendorId=0x%04X ProductId=0x%04X",
        __FUNCTION__, *VendorId, *ProductId);

    return STATUS_SUCCESS;
}

static NTSTATUS
UsbAsRamHexDigit(
    _In_ WCHAR Ch,
    _Out_ PULONG Digit
    )
{
    if (Ch >= L'0' && Ch <= L'9') {
        *Digit = (ULONG)(Ch - L'0');
        return STATUS_SUCCESS;
    }
    if (Ch >= L'A' && Ch <= L'F') {
        *Digit = 10 + (ULONG)(Ch - L'A');
        return STATUS_SUCCESS;
    }
    if (Ch >= L'a' && Ch <= L'f') {
        *Digit = 10 + (ULONG)(Ch - L'a');
        return STATUS_SUCCESS;
    }
    return STATUS_INVALID_PARAMETER;
}

static NTSTATUS
UsbAsRamParseHexUlongBounded(
    _In_reads_(Len) PCWSTR Buf,
    _In_ USHORT Len,
    _Out_ PULONG Value
    )
{
    ULONG v = 0;
    USHORT i;

    for (i = 0; i < Len; ++i) {
        ULONG d = 0;

        if (!NT_SUCCESS(UsbAsRamHexDigit(Buf[i], &d))) {
            break;
        }
        v = (v << 4) | d;
    }

    if (i == 0) {
        return STATUS_INVALID_PARAMETER;
    }

    *Value = v;
    return STATUS_SUCCESS;
}

static BOOLEAN
UsbAsRamMatchPrefixCi(
    _In_reads_(PrefixLen) PCWSTR Buf,
    _In_ USHORT BufChars,
    _In_ PCWSTR Prefix,
    _In_ USHORT PrefixLen
    )
{
    USHORT i;

    if (BufChars < PrefixLen) {
        return FALSE;
    }

    for (i = 0; i < PrefixLen; ++i) {
        WCHAR a = Buf[i];
        WCHAR b = Prefix[i];

        if (a >= L'a' && a <= L'z') {
            a = (WCHAR)(a - (L'a' - L'A'));
        }
        if (b >= L'a' && b <= L'z') {
            b = (WCHAR)(b - (L'a' - L'A'));
        }
        if (a != b) {
            return FALSE;
        }
    }

    return TRUE;
}

static NTSTATUS
UsbAsRamParseVidPidFromHardwareId(
    _In_ PCUNICODE_STRING HardwareId,
    _Out_ PULONG VendorId,
    _Out_ PULONG ProductId
    )
{
    USHORT n;
    USHORT i;
    ULONG vid = 0;
    ULONG pid = 0;
    BOOLEAN haveVid = FALSE;
    BOOLEAN havePid = FALSE;
    DECLARE_CONST_UNICODE_STRING(vidPrefix, L"VID_");
    DECLARE_CONST_UNICODE_STRING(pidPrefix, L"PID_");

    if (HardwareId == NULL || HardwareId->Buffer == NULL || HardwareId->Length == 0) {
        return STATUS_INVALID_PARAMETER;
    }

    n = HardwareId->Length / sizeof(WCHAR);

    for (i = 0; i + vidPrefix.Length / sizeof(WCHAR) < n; ++i) {
        if (UsbAsRamMatchPrefixCi(
                &HardwareId->Buffer[i],
                (USHORT)(n - i),
                vidPrefix.Buffer,
                (USHORT)(vidPrefix.Length / sizeof(WCHAR)))) {

            USHORT j = i + (USHORT)(vidPrefix.Length / sizeof(WCHAR));
            USHORT k = 0;

            while (j + k < n && k < 4) {
                WCHAR c = HardwareId->Buffer[j + k];
                if (c == L'&' || c == L'\0') {
                    break;
                }
                ++k;
            }

            if (NT_SUCCESS(UsbAsRamParseHexUlongBounded(&HardwareId->Buffer[j], k, &vid))) {
                haveVid = TRUE;
            }
            break;
        }
    }

    for (i = 0; i + pidPrefix.Length / sizeof(WCHAR) < n; ++i) {
        if (UsbAsRamMatchPrefixCi(
                &HardwareId->Buffer[i],
                (USHORT)(n - i),
                pidPrefix.Buffer,
                (USHORT)(pidPrefix.Length / sizeof(WCHAR)))) {

            USHORT j = i + (USHORT)(pidPrefix.Length / sizeof(WCHAR));
            USHORT k = 0;

            while (j + k < n && k < 4) {
                WCHAR c = HardwareId->Buffer[j + k];
                if (c == L'&' || c == L'\0') {
                    break;
                }
                ++k;
            }

            if (NT_SUCCESS(UsbAsRamParseHexUlongBounded(&HardwareId->Buffer[j], k, &pid))) {
                havePid = TRUE;
            }
            break;
        }
    }

    if (!haveVid || !havePid) {
        return STATUS_NOT_FOUND;
    }

    *VendorId = vid & 0xFFFF;
    *ProductId = pid & 0xFFFF;
    return STATUS_SUCCESS;
}

static NTSTATUS
UsbAsRamRegisterMemoryArrayInterface(
    _In_ WDFDEVICE Device
    )
{
    NTSTATUS status;
    PUSBASRAM_DEVICE_CONTEXT ctx = UsbAsRamDeviceGetContext(Device);

    if (ctx->MemoryArrayInterfaceRegistered) {
        return STATUS_SUCCESS;
    }

    status = WdfDeviceCreateDeviceInterface(
        Device,
        &GUID_DEVINTERFACE_MEMORY_ARRAY,
        NULL);

    if (!NT_SUCCESS(status)) {
        return status;
    }

    ctx->MemoryArrayInterfaceRegistered = TRUE;
    UsbAsRamTraceInfo("%s: Registered GUID_DEVINTERFACE_MEMORY_ARRAY (NFIT/DAX integration pending)",
        __FUNCTION__);

    return STATUS_SUCCESS;
}
