#if !defined(_RTI2S_H_)
#define _RTI2S_H_

#pragma warning(disable:4200)  // suppress nameless struct/union warning
#pragma warning(disable:4201)  // suppress nameless struct/union warning
#pragma warning(disable:4214)  // suppress bit field types other than int warning
#include <initguid.h>
#include <wdm.h>

#pragma warning(default:4200)
#pragma warning(default:4201)
#pragma warning(default:4214)
#include <wdf.h>

#pragma warning(disable:4201)  // suppress nameless struct/union warning
#pragma warning(disable:4214)  // suppress bit field types other than int warning
#include <hidport.h>

#include "rt5677i2c.h"
#include "spb.h"

//
// String definitions
//

#define DRIVERNAME                 "rt5677i2c.sys: "

#define RTI2S_POOL_TAG            (ULONG) 'SIRT'
#define RTI2S_HARDWARE_IDS        L"CoolStar\\10EC5677\0\0"
#define RTI2S_HARDWARE_IDS_LENGTH sizeof(RTI2S_HARDWARE_IDS)

#define NTDEVICE_NAME_STRING       L"\\Device\\10EC5677"
#define SYMBOLIC_NAME_STRING       L"\\DosDevices\\10EC5677"

#define true 1
#define false 0

typedef struct _RTI2S_CONTEXT
{

	//
	// Handle back to the WDFDEVICE
	//

	WDFDEVICE FxDevice;

	WDFQUEUE ReportQueue;

	SPB_CONTEXT I2CContext;

	WDFTIMER Timer;

	BOOLEAN ConnectInterrupt;

	BOOLEAN HeadphonesConnected;

	BOOLEAN HeadsetMicConnected;

} RTI2S_CONTEXT, *PRTI2S_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(RTI2S_CONTEXT, GetDeviceContext)

//
// Function definitions
//

DRIVER_INITIALIZE DriverEntry;

EVT_WDF_DRIVER_UNLOAD RtI2SDriverUnload;

EVT_WDF_DRIVER_DEVICE_ADD RtI2SEvtDeviceAdd;

EVT_WDFDEVICE_WDM_IRP_PREPROCESS RtI2SEvtWdmPreprocessMnQueryId;

EVT_WDF_IO_QUEUE_IO_INTERNAL_DEVICE_CONTROL RtI2SEvtInternalDeviceControl;

//
// Helper macros
//

#define DEBUG_LEVEL_ERROR   1
#define DEBUG_LEVEL_INFO    2
#define DEBUG_LEVEL_VERBOSE 3

#define DBG_INIT  1
#define DBG_PNP   2
#define DBG_IOCTL 4

#if 0
#define RtI2SPrint(dbglevel, dbgcatagory, fmt, ...) {          \
    if (RtI2SDebugLevel >= dbglevel &&                         \
        (RtI2SDebugCatagories && dbgcatagory))                 \
		    {                                                           \
        DbgPrint(DRIVERNAME);                                   \
        DbgPrint(fmt, __VA_ARGS__);                             \
		    }                                                           \
}
#else
#define RtI2SPrint(dbglevel, fmt, ...) {                       \
}
#endif
#endif