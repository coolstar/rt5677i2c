#define DESCRIPTOR_DEF
#include "driver.h"
#include "stdint.h"

#define bool int
#define MS_IN_US 1000

static ULONG RtI2SDebugLevel = 100;
static ULONG RtI2SDebugCatagories = DBG_INIT || DBG_PNP || DBG_IOCTL;

NTSTATUS
DriverEntry(
__in PDRIVER_OBJECT  DriverObject,
__in PUNICODE_STRING RegistryPath
)
{
	NTSTATUS               status = STATUS_SUCCESS;
	WDF_DRIVER_CONFIG      config;
	WDF_OBJECT_ATTRIBUTES  attributes;

	RtI2SPrint(DEBUG_LEVEL_INFO, DBG_INIT,
		"Driver Entry\n");

	WDF_DRIVER_CONFIG_INIT(&config, RtI2SEvtDeviceAdd);

	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);

	//
	// Create a framework driver object to represent our driver.
	//

	status = WdfDriverCreate(DriverObject,
		RegistryPath,
		&attributes,
		&config,
		WDF_NO_HANDLE
		);

	if (!NT_SUCCESS(status))
	{
		RtI2SPrint(DEBUG_LEVEL_ERROR, DBG_INIT,
			"WdfDriverCreate failed with status 0x%x\n", status);
	}

	return status;
}

/* RT5677 has 256 8-bit register addresses, and 16-bit register data */
struct rt5677_init_reg {
	uint8_t reg;
	uint16_t val;
};

#define RT5677_PR_RANGE_BASE (0xff + 1)
#define RT5677_PR_SPACING 0x100

#define RT5677_PR_BASE (RT5677_PR_RANGE_BASE + (0 * RT5677_PR_SPACING))

static struct rt5677_init_reg init_list[] = {
	{RT5677_LOUT1, RT5677_LOUT1_L_DF | RT5677_LOUT2_L_DF | RT5677_LOUT3_L_DF | RT5677_LOUT3_L_MUTE},
	{RT5677_IN1, RT5677_IN_DF1 | RT5677_IN_DF2},
	{RT5677_MICBIAS, RT5677_MICBIAS1_CTRL_VDD_3_3V},

	{RT5677_SIDETONE_CTRL,	  0x000b},
	{RT5677_STO1_ADC_DIG_VOL, 0x7F7F},
	{RT5677_STO2_ADC_MIXER,	  0xD4C0},
	{RT5677_STO1_ADC_MIXER,	  0x5480}, //0x5480 for internal mic, 0x9440 for external mic
	{RT5677_STO1_DAC_MIXER,	  0x8A8A},
	{RT5677_MONO_ADC_MIXER,   0x54D1},
	{RT5677_PWR_DIG1,	  RT5677_PWR_I2S1 | RT5677_PWR_DAC1 | RT5677_PWR_DAC2}, //add RT5677_PWR_DAC1 | RT5677_PWR_DAC2 for headphones, add RT5677_PWR_ADCFED1 | RT5677_PWR_ADC_L for external mic
	{RT5677_PWR_DIG2,	  RT5677_PWR_ADC_S1F | RT5677_PWR_ADC_MF_L | RT5677_PWR_DAC_S1F | RT5677_PWR_PDM1}, //add RT5677_PWR_PDM1 for speaker
	{RT5677_PWR_ANLG1,	  0xFDD5},
	{RT5677_PWR_ANLG2,	  0x2CC0}, //0x2CC0 for internal mic, 0xACE0 for external mic
	{RT5677_PWR_DSP2,	  0x0C00},
	{RT5677_I2S2_SDP,	  0x0000},
	{RT5677_CLK_TREE_CTRL1,	  0x1111},
	{RT5677_PLL1_CTRL1,	  0x0000},
	{RT5677_PLL1_CTRL2,	  0x0000},
	{RT5677_GPIO_CTRL2,       0x0030},
	{RT5677_DIG_MISC,	  0x0029},
	{RT5677_GEN_CTRL1,	  0x00FF},
	{RT5677_PDM_OUT_CTRL,	  0x0088}, //0x8888 for headphones only, 0x0088 for both
	{RT5677_DAC1_DIG_VOL,	  0x9f9f}, //0x9f9f for headphones, 0xafaf for speakers
	{RT5677_DAC2_DIG_VOL,	  0xffff},
	{RT5677_STO1_ADC_DIG_VOL, 0x3f3f},
	{RT5677_DMIC_CTRL1, 0x9545},
	{RT5677_ANA_ADC_GAIN_CTRL, 0xa000},
	{RT5677_STO1_2_ADC_BST, 0xa000},
	{RT5677_ADC_BST_CTRL2, 0xa000},

	{RT5677_TDM1_CTRL1, 0x0300}, //0x0380 for ext mic, 0x0300 for int mic

	{RT5677_ASRC_1,	0x0001},
	{RT5677_ASRC_2,	0x4820}, //0x4020 to for ext mic, 0x4820 for int mic
	{RT5677_ASRC_3,	0x1000},
	{RT5677_ASRC_5,	0x1000},
	{RT5677_ASRC_6,	0x7000},

	{RT5677_VAD_CTRL4,	0x010c},

	{RT5677_DSP_INB_CTRL1,	0x4000},

	{RT5677_JD_CTRL1,	0x2c00},
	{RT5677_IRQ_CTRL1,	0x0504},

	{RT5677_GPIO_CTRL1, 0x8000},

	{RT5677_ASRC_12,	0x0018},

	{RT5677_PRIV_INDEX,	  RT5677_PR_BASE + 0x3d},
	{RT5677_PRIV_DATA,	  0x364D},

	{RT5677_PRIV_INDEX,	  RT5677_PR_BASE + 0x17},
	{RT5677_PRIV_DATA,	  0x4fc0},

	{RT5677_PRIV_INDEX,	  RT5677_PR_BASE + 0x13},
	{RT5677_PRIV_DATA,	  0x0312},

	{RT5677_PRIV_INDEX,	  RT5677_PR_BASE + 0x1e},
	{RT5677_PRIV_DATA,	  0x0000},

	{RT5677_PRIV_INDEX,	  RT5677_PR_BASE + 0x12},
	{RT5677_PRIV_DATA,	  0x0eaa},

	{RT5677_PRIV_INDEX,	  RT5677_PR_BASE + 0x14},
	{RT5677_PRIV_DATA,	  0x018a},

	{RT5677_PRIV_INDEX,	  RT5677_PR_BASE + 0x15},
	{RT5677_PRIV_DATA,	  0x0490},

	{RT5677_PRIV_INDEX,	  RT5677_PR_BASE + 0x38},
	{RT5677_PRIV_DATA,	  0x0f71},

	{RT5677_PRIV_INDEX,	  RT5677_PR_BASE + 0x39},
	{RT5677_PRIV_DATA,	  0x0f71}, 

	//Private register, no doc [rt5677_set_vad_source]
	{RT5677_PRIV_INDEX,	  RT5677_PR_BASE + RT5677_BIAS_CUR4},
	{RT5677_PRIV_DATA,	  0x0f8a},
};

NTSTATUS rt5677_reg_read(
	_In_ PRTI2S_CONTEXT pDevice,
	uint8_t reg,
	uint16_t* data
) {
	uint16_t raw_data = 0;
	NTSTATUS status = SpbXferDataSynchronously(&pDevice->I2CContext, &reg, sizeof(uint8_t), &raw_data, sizeof(uint16_t));
	*data = raw_data;
	return status;
}

NTSTATUS rt5677_reg_write(
	_In_ PRTI2S_CONTEXT pDevice,
	uint8_t reg,
	uint16_t data
) {
	uint8_t buf[3];
	buf[0] = reg;
	buf[1] = (data >> 8);
	buf[2] = data & 0xFF;
	return SpbWriteDataSynchronously(&pDevice->I2CContext, buf, sizeof(buf));
}

NTSTATUS rt5677_reg_update(
	_In_ PRTI2S_CONTEXT pDevice,
	uint8_t reg,
	uint16_t mask,
	uint16_t val
) {
	uint16_t tmp = 0, orig = 0;

	NTSTATUS status = rt5677_reg_read(pDevice, reg, &orig);
	if (!NT_SUCCESS(status)) {
		return status;
	}

	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig) {
		uint8_t buf[3];
		buf[0] = reg;
		buf[1] = (tmp >> 8);
		buf[2] = tmp & 0xFF;
		status = SpbWriteDataSynchronously(&pDevice->I2CContext, buf, sizeof(buf));
	}
	return status;
}

NTSTATUS rt5677_reset(PRTI2S_CONTEXT pDevice) {
	NTSTATUS status = rt5677_reg_write(pDevice, RT5677_RESET, RT5677_SW_RESET);
	if (!NT_SUCCESS(status)) {
		RtI2SPrint(DEBUG_LEVEL_ERROR, DBG_PNP,
			"Error resetting codec!\n");
		return status;
	}
	return status;
}

/*static void debug_dump_5677_regs(PRTI2S_CONTEXT pDevice)
{
	uint16_t i, reg_word;
	// Show all 16-bit codec regs
	for (i = 0; i < RT5677_REG_CNT; i += 8) {
		uint32_t regs[8];
		for (int j = 0; j < 8; j++) {
			rt5677_reg_read(pDevice, (uint8_t)(i + j), &reg_word);
			regs[j] = reg_word;
		}
		DbgPrint("%02x: %04x %04x %04x %04x %04x %04x %04x %04x \n", i, regs[0], regs[1], regs[2], regs[3], regs[4], regs[5], regs[6], regs[7]);
	}
}*/

VOID
RtI2SBootWorkItem(
	IN WDFWORKITEM  WorkItem
)
{
	WDFDEVICE Device = (WDFDEVICE)WdfWorkItemGetParentObject(WorkItem);
	PRTI2S_CONTEXT pDevice = GetDeviceContext(Device);

	pDevice->ConnectInterrupt = true;

	uint16_t id, reg;

	if (!NT_SUCCESS(rt5677_reg_read(pDevice, RT5677_VENDOR_ID1, &id))) {
		RtI2SPrint(DEBUG_LEVEL_ERROR, DBG_PNP,
			"Error reading vendor ID!\n");
		goto end;
	}
	RtI2SPrint(DEBUG_LEVEL_INFO, DBG_PNP,
		"Hardware ID: 0x%x\n", id);

	if (!NT_SUCCESS(rt5677_reg_read(pDevice, RT5677_VENDOR_ID2, &reg))) {
		RtI2SPrint(DEBUG_LEVEL_ERROR, DBG_PNP,
			"Error reading vendor rev!\n");
		goto end;
	}
	RtI2SPrint(DEBUG_LEVEL_INFO, DBG_PNP,
		"Hardware revision: 0x%x\n", reg);

	/* Initialize codec regs with static base/values */
	for (int i = 0; i < ARRAYSIZE(init_list); i++) {
		rt5677_reg_write(pDevice, init_list[i].reg, init_list[i].val);
	}

	//16 bits per sample
	if (!NT_SUCCESS(rt5677_reg_update(pDevice, RT5677_I2S1_SDP, RT5677_I2S_DL_MASK, 0))) {
		RtI2SPrint(DEBUG_LEVEL_ERROR, DBG_PNP,
			"Error updating I2S1 interface Ctrl reg!\n");
		goto end;
	}

	NTSTATUS status;
	/* Set format here: Assumes I2S, NB_NF, CBS_CFS */

	/* CBS_CFS (Codec Bit Slave/Codec Frame Slave) */
	status = rt5677_reg_update(pDevice, RT5677_I2S1_SDP, RT5677_I2S_MS_MASK,
		RT5677_I2S_MS_S);
	if (!NT_SUCCESS(status)) {
		goto end;
	}

	/* NB_NF (Normal Bit/Normal Frame) */
	status = rt5677_reg_update(pDevice, RT5677_I2S1_SDP, RT5677_I2S_BP_MASK,
		RT5677_I2S_BP_NOR);
	if (!NT_SUCCESS(status)) {
		goto end;
	}

	/* I2S mode */
	status = rt5677_reg_update(pDevice, RT5677_I2S1_SDP, RT5677_I2S_DF_MASK,
		RT5677_I2S_DF_I2S);
	if (!NT_SUCCESS(status)) {
		goto end;
	}

	/* NB_NF (Normal Bit/Normal Frame) */
	status = rt5677_reg_update(pDevice, RT5677_I2S2_SDP, RT5677_I2S_BP_MASK,
		RT5677_I2S_BP_NOR);
	if (!NT_SUCCESS(status)) {
		goto end;
	}

	/* I2S mode */
	status = rt5677_reg_update(pDevice, RT5677_I2S2_SDP, RT5677_I2S_DF_MASK,
		RT5677_I2S_DF_I2S);
	if (!NT_SUCCESS(status)) {
		goto end;
	}

	/* Master Select */
	status = rt5677_reg_update(pDevice, RT5677_I2S2_SDP, RT5677_I2S_MS_MASK,
		RT5677_I2S_MS_S);
	if (!NT_SUCCESS(status)) {
		goto end;
	}

	//Set GPIO pull states
	int gpios[] = {RT5677_GPIO5, RT5677_GPIO6 };
	for (int i = 0; i < sizeof(gpios) / sizeof(int); i++) {
		int offset = gpios[i];
		int value = 2; //Pull up

		status = rt5677_reg_write(pDevice, RT5677_PRIV_INDEX, RT5677_PR_BASE + RT5677_DIG_IN_PIN_ST_CTRL3);
		if (!NT_SUCCESS(status)) {
			continue;
		}

		int shift = 2 * (9 - offset);
		status = rt5677_reg_update(pDevice, RT5677_PRIV_DATA, 0x3 << shift, (value & 0x3) << shift);
		if (!NT_SUCCESS(status)) {
			RtI2SPrint(DEBUG_LEVEL_ERROR, DBG_PNP,
				"Failed to set pullup state for %d\n", offset);
		}
	}

	status = rt5677_reg_update(pDevice, RT5677_GPIO_CTRL2,
		0x1 << (RT5677_GPIO5 * 3 + 2), 0x0);
	if (!NT_SUCCESS(status)) {
		RtI2SPrint(DEBUG_LEVEL_ERROR, DBG_PNP,
			"Failed to set gpio in for %d\n", RT5677_GPIO5);
	}

	status = rt5677_reg_update(pDevice, RT5677_GPIO_CTRL3,
		RT5677_GPIO6_DIR_MASK, RT5677_GPIO6_DIR_IN);
	if (!NT_SUCCESS(status)) {
		RtI2SPrint(DEBUG_LEVEL_ERROR, DBG_PNP,
			"Failed to set gpio in for %d\n", RT5677_GPIO6);
	}

	/*debug_dump_5677_regs(pDevice);

	DbgPrint("Dumped Registers!\n");*/
end:
	WdfObjectDelete(WorkItem);
}

void RtI2SBootTimer(_In_ WDFTIMER hTimer) {
	WDFDEVICE Device = (WDFDEVICE)WdfTimerGetParentObject(hTimer);
	PRTI2S_CONTEXT pDevice = GetDeviceContext(Device);

	WDF_OBJECT_ATTRIBUTES attributes;
	WDF_WORKITEM_CONFIG workitemConfig;
	WDFWORKITEM hWorkItem;

	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
	WDF_OBJECT_ATTRIBUTES_SET_CONTEXT_TYPE(&attributes, RTI2S_CONTEXT);
	attributes.ParentObject = Device;
	WDF_WORKITEM_CONFIG_INIT(&workitemConfig, RtI2SBootWorkItem);

	WdfWorkItemCreate(&workitemConfig,
		&attributes,
		&hWorkItem);

	WdfWorkItemEnqueue(hWorkItem);
	WdfTimerStop(hTimer, FALSE);
}

NTSTATUS BOOTCODEC(
	_In_  PRTI2S_CONTEXT  pDevice
	)
{
	NTSTATUS status = 0;

	pDevice->ConnectInterrupt = false;

	pDevice->HeadphonesConnected = false;

	uint16_t reg;
	RtI2SPrint(DEBUG_LEVEL_INFO, DBG_PNP,
		"Initializing ALC5677!\n");

	/* Read status reg */
	rt5677_reg_read(pDevice, RT5677_RESET, &reg);
	RtI2SPrint(DEBUG_LEVEL_INFO, DBG_PNP,
		"%s: reg 00h, Software Reset & Status = 0x%X\n", __func__,
		reg);

	status = rt5677_reset(pDevice);
	if (!NT_SUCCESS(status)) {
		return status;
	}

	WDF_TIMER_CONFIG              timerConfig;
	WDFTIMER                      hTimer;
	WDF_OBJECT_ATTRIBUTES         attributes;

	WDF_TIMER_CONFIG_INIT(&timerConfig, RtI2SBootTimer);

	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
	attributes.ParentObject = pDevice->FxDevice;
	status = WdfTimerCreate(&timerConfig, &attributes, &hTimer);

	WdfTimerStart(hTimer, WDF_REL_TIMEOUT_IN_MS(20));

	return status;
}

NTSTATUS
OnPrepareHardware(
_In_  WDFDEVICE     FxDevice,
_In_  WDFCMRESLIST  FxResourcesRaw,
_In_  WDFCMRESLIST  FxResourcesTranslated
)
/*++

Routine Description:

This routine caches the SPB resource connection ID.

Arguments:

FxDevice - a handle to the framework device object
FxResourcesRaw - list of translated hardware resources that
the PnP manager has assigned to the device
FxResourcesTranslated - list of raw hardware resources that
the PnP manager has assigned to the device

Return Value:

Status

--*/
{
	PRTI2S_CONTEXT pDevice = GetDeviceContext(FxDevice);
	BOOLEAN fSpbResourceFound = FALSE;
	BOOLEAN fJackDetectResourceFound = FALSE;
	NTSTATUS status = STATUS_INSUFFICIENT_RESOURCES;

	UNREFERENCED_PARAMETER(FxResourcesRaw);

	//
	// Parse the peripheral's resources.
	//

	ULONG resourceCount = WdfCmResourceListGetCount(FxResourcesTranslated);

	for (ULONG i = 0; i < resourceCount; i++)
	{
		PCM_PARTIAL_RESOURCE_DESCRIPTOR pDescriptor;
		UCHAR Class;
		UCHAR Type;

		pDescriptor = WdfCmResourceListGetDescriptor(
			FxResourcesTranslated, i);

		switch (pDescriptor->Type)
		{
		case CmResourceTypeConnection:
			//
			// Look for I2C or SPI resource and save connection ID.
			//
			Class = pDescriptor->u.Connection.Class;
			Type = pDescriptor->u.Connection.Type;
			if (Class == CM_RESOURCE_CONNECTION_CLASS_SERIAL &&
				Type == CM_RESOURCE_CONNECTION_TYPE_SERIAL_I2C)
			{
				if (fSpbResourceFound == FALSE)
				{
					status = STATUS_SUCCESS;
					pDevice->I2CContext.I2cResHubId.LowPart = pDescriptor->u.Connection.IdLowPart;
					pDevice->I2CContext.I2cResHubId.HighPart = pDescriptor->u.Connection.IdHighPart;
					fSpbResourceFound = TRUE;
				}
				else
				{
				}
			}
			break;
		default:
			//
			// Ignoring all other resource types.
			//
			break;
		}
	}

	//
	// An SPB resource is required.
	//

	if (fSpbResourceFound == FALSE)
	{
		status = STATUS_NOT_FOUND;
		return status;
	}

	status = SpbTargetInitialize(FxDevice, &pDevice->I2CContext);
	if (!NT_SUCCESS(status))
	{
		return status;
	}

	return status;
}

NTSTATUS
OnReleaseHardware(
_In_  WDFDEVICE     FxDevice,
_In_  WDFCMRESLIST  FxResourcesTranslated
)
/*++

Routine Description:

Arguments:

FxDevice - a handle to the framework device object
FxResourcesTranslated - list of raw hardware resources that
the PnP manager has assigned to the device

Return Value:

Status

--*/
{
	PRTI2S_CONTEXT pDevice = GetDeviceContext(FxDevice);
	NTSTATUS status = STATUS_SUCCESS;

	UNREFERENCED_PARAMETER(FxResourcesTranslated);

	SpbTargetDeinitialize(FxDevice, &pDevice->I2CContext);

	return status;
}

NTSTATUS
OnD0Entry(
_In_  WDFDEVICE               FxDevice,
_In_  WDF_POWER_DEVICE_STATE  FxPreviousState
)
/*++

Routine Description:

This routine allocates objects needed by the driver.

Arguments:

FxDevice - a handle to the framework device object
FxPreviousState - previous power state

Return Value:

Status

--*/
{
	UNREFERENCED_PARAMETER(FxPreviousState);

	PRTI2S_CONTEXT pDevice = GetDeviceContext(FxDevice);
	NTSTATUS status = STATUS_SUCCESS;

	WdfTimerStart(pDevice->Timer, WDF_REL_TIMEOUT_IN_MS(10));

	BOOTCODEC(pDevice);

	return status;
}

NTSTATUS
OnD0Exit(
_In_  WDFDEVICE               FxDevice,
_In_  WDF_POWER_DEVICE_STATE  FxPreviousState
)
/*++

Routine Description:

This routine destroys objects needed by the driver.

Arguments:

FxDevice - a handle to the framework device object
FxPreviousState - previous power state

Return Value:

Status

--*/
{
	UNREFERENCED_PARAMETER(FxPreviousState);

	PRTI2S_CONTEXT pDevice = GetDeviceContext(FxDevice);

	WdfTimerStop(pDevice->Timer, TRUE);

	pDevice->ConnectInterrupt = false;

	return STATUS_SUCCESS;
}

VOID
CodecJackSwitchWorkItem(
	IN WDFWORKITEM  WorkItem
)
{
	WDFDEVICE Device = (WDFDEVICE)WdfWorkItemGetParentObject(WorkItem);
	PRTI2S_CONTEXT pDevice = GetDeviceContext(Device);

	uint16_t value;
	NTSTATUS status = rt5677_reg_read(pDevice, RT5677_GPIO_ST, &value);
	if (NT_SUCCESS(status)) {
		value = value >> 8;

		BOOLEAN isJDet = (value & (1 << 4)) != 0;
		if (isJDet != pDevice->HeadphonesConnected) {
			pDevice->HeadphonesConnected = isJDet;
			pDevice->HeadsetMicConnected = pDevice->HeadphonesConnected;

			uint16_t dig1 = RT5677_PWR_I2S1;
			if (pDevice->HeadphonesConnected) {
				dig1 |= RT5677_PWR_DAC1 | RT5677_PWR_DAC2;
			}
			if (pDevice->HeadsetMicConnected) {
				dig1 |= RT5677_PWR_ADCFED1 | RT5677_PWR_ADC_L;
			}

			if (pDevice->HeadphonesConnected) {
				rt5677_reg_write(pDevice, RT5677_PWR_DIG2, RT5677_PWR_ADC_S1F | RT5677_PWR_ADC_MF_L | RT5677_PWR_DAC_S1F);
			}
			else {
				rt5677_reg_write(pDevice, RT5677_PWR_DIG2, RT5677_PWR_ADC_S1F | RT5677_PWR_ADC_MF_L | RT5677_PWR_DAC_S1F | RT5677_PWR_PDM1);
			}

			if (pDevice->HeadsetMicConnected) {
				static struct rt5677_init_reg update_list[] = {
					{ RT5677_STO1_ADC_MIXER,	  0x9440 }, //0x5480 for internal mic, 0x9440 for external mic
					{ RT5677_PWR_ANLG2,	  0xACE0 }, //0x2CC0 for internal mic, 0xACE0 for external mic
					{ RT5677_TDM1_CTRL1, 0x0380 }, //0x0380 for ext mic, 0x0300 for int mic
					{ RT5677_ASRC_2,	0x4020 } //0x4020 to for ext mic, 0x4820 for int mic
				};
				for (int i = 0; i < ARRAYSIZE(update_list); i++) {
					rt5677_reg_write(pDevice, update_list[i].reg, update_list[i].val);
				}

			}
			else {
				static struct rt5677_init_reg update_list[] = {
					{ RT5677_STO1_ADC_MIXER,	  0x5480 }, //0x5480 for internal mic, 0x9440 for external mic
					{ RT5677_PWR_ANLG2,	  0x2CC0 }, //0x2CC0 for internal mic, 0xACE0 for external mic
					{ RT5677_TDM1_CTRL1, 0x0300 }, //0x0380 for ext mic, 0x0300 for int mic
					{ RT5677_ASRC_2,	0x4820 } //0x4020 to for ext mic, 0x4820 for int mic
				};
				for (int i = 0; i < ARRAYSIZE(update_list); i++) {
					rt5677_reg_write(pDevice, update_list[i].reg, update_list[i].val);
				}
			}

			rt5677_reg_write(pDevice, RT5677_PWR_DIG1, dig1);
		}
	}

	WdfObjectDelete(WorkItem);
}

void RtI2SJDetTimer(_In_ WDFTIMER hTimer) {
	WDFDEVICE Device = (WDFDEVICE)WdfTimerGetParentObject(hTimer);
	PRTI2S_CONTEXT pDevice = GetDeviceContext(Device);

	if (!pDevice->ConnectInterrupt)
		return;

	WDF_OBJECT_ATTRIBUTES attributes;
	WDF_WORKITEM_CONFIG workitemConfig;
	WDFWORKITEM hWorkItem;

	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
	WDF_OBJECT_ATTRIBUTES_SET_CONTEXT_TYPE(&attributes, RTI2S_CONTEXT);
	attributes.ParentObject = Device;
	WDF_WORKITEM_CONFIG_INIT(&workitemConfig, CodecJackSwitchWorkItem);

	WdfWorkItemCreate(&workitemConfig,
		&attributes,
		&hWorkItem);

	WdfWorkItemEnqueue(hWorkItem);
}

NTSTATUS
RtI2SEvtDeviceAdd(
IN WDFDRIVER       Driver,
IN PWDFDEVICE_INIT DeviceInit
)
{
	NTSTATUS                      status = STATUS_SUCCESS;
	WDF_IO_QUEUE_CONFIG           queueConfig;
	WDF_OBJECT_ATTRIBUTES         attributes;
	WDFDEVICE                     device;
	WDF_INTERRUPT_CONFIG interruptConfig;
	WDFQUEUE                      queue;
	UCHAR                         minorFunction;
	PRTI2S_CONTEXT               devContext;

	UNREFERENCED_PARAMETER(Driver);

	PAGED_CODE();

	RtI2SPrint(DEBUG_LEVEL_INFO, DBG_PNP,
		"RtI2SEvtDeviceAdd called\n");

	{
		WDF_PNPPOWER_EVENT_CALLBACKS pnpCallbacks;
		WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpCallbacks);

		pnpCallbacks.EvtDevicePrepareHardware = OnPrepareHardware;
		pnpCallbacks.EvtDeviceReleaseHardware = OnReleaseHardware;
		pnpCallbacks.EvtDeviceD0Entry = OnD0Entry;
		pnpCallbacks.EvtDeviceD0Exit = OnD0Exit;

		WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpCallbacks);
	}

	//
	// Because we are a virtual device the root enumerator would just put null values 
	// in response to IRP_MN_QUERY_ID. Lets override that.
	//

	minorFunction = IRP_MN_QUERY_ID;

	status = WdfDeviceInitAssignWdmIrpPreprocessCallback(
		DeviceInit,
		RtI2SEvtWdmPreprocessMnQueryId,
		IRP_MJ_PNP,
		&minorFunction,
		1
		);
	if (!NT_SUCCESS(status))
	{
		RtI2SPrint(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfDeviceInitAssignWdmIrpPreprocessCallback failed Status 0x%x\n", status);

		return status;
	}

	//
	// Setup the device context
	//

	WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, RTI2S_CONTEXT);

	//
	// Create a framework device object.This call will in turn create
	// a WDM device object, attach to the lower stack, and set the
	// appropriate flags and attributes.
	//

	status = WdfDeviceCreate(&DeviceInit, &attributes, &device);

	if (!NT_SUCCESS(status))
	{
		RtI2SPrint(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfDeviceCreate failed with status code 0x%x\n", status);

		return status;
	}

	WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&queueConfig, WdfIoQueueDispatchParallel);

	queueConfig.EvtIoInternalDeviceControl = RtI2SEvtInternalDeviceControl;

	status = WdfIoQueueCreate(device,
		&queueConfig,
		WDF_NO_OBJECT_ATTRIBUTES,
		&queue
		);

	if (!NT_SUCCESS(status))
	{
		RtI2SPrint(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfIoQueueCreate failed 0x%x\n", status);

		return status;
	}

	//
	// Create manual I/O queue to take care of hid report read requests
	//

	devContext = GetDeviceContext(device);

	WDF_IO_QUEUE_CONFIG_INIT(&queueConfig, WdfIoQueueDispatchManual);

	queueConfig.PowerManaged = WdfFalse;

	status = WdfIoQueueCreate(device,
		&queueConfig,
		WDF_NO_OBJECT_ATTRIBUTES,
		&devContext->ReportQueue
		);

	if (!NT_SUCCESS(status))
	{
		RtI2SPrint(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfIoQueueCreate failed 0x%x\n", status);

		return status;
	}

	WDF_TIMER_CONFIG              timerConfig;
	WDFTIMER                      hTimer;

	WDF_TIMER_CONFIG_INIT_PERIODIC(&timerConfig, RtI2SJDetTimer, 10);

	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
	attributes.ParentObject = device;
	status = WdfTimerCreate(&timerConfig, &attributes, &hTimer);
	devContext->Timer = hTimer;
	if (!NT_SUCCESS(status))
	{
		RtI2SPrint(DEBUG_LEVEL_ERROR, DBG_PNP, "(%!FUNC!) WdfTimerCreate failed status:%!STATUS!\n", status);
		return status;
	}

	devContext->FxDevice = device;

	return status;
}

NTSTATUS
RtI2SEvtWdmPreprocessMnQueryId(
WDFDEVICE Device,
PIRP Irp
)
{
	NTSTATUS            status;
	PIO_STACK_LOCATION  IrpStack, previousSp;
	PDEVICE_OBJECT      DeviceObject;
	PWCHAR              buffer;

	PAGED_CODE();

	//
	// Get a pointer to the current location in the Irp
	//

	IrpStack = IoGetCurrentIrpStackLocation(Irp);

	//
	// Get the device object
	//
	DeviceObject = WdfDeviceWdmGetDeviceObject(Device);


	RtI2SPrint(DEBUG_LEVEL_VERBOSE, DBG_PNP,
		"RtI2SEvtWdmPreprocessMnQueryId Entry\n");

	//
	// This check is required to filter out QUERY_IDs forwarded
	// by the HIDCLASS for the parent FDO. These IDs are sent
	// by PNP manager for the parent FDO if you root-enumerate this driver.
	//
	previousSp = ((PIO_STACK_LOCATION)((UCHAR *)(IrpStack)+
		sizeof(IO_STACK_LOCATION)));

	if (previousSp->DeviceObject == DeviceObject)
	{
		//
		// Filtering out this basically prevents the Found New Hardware
		// popup for the root-enumerated RtI2S on reboot.
		//
		status = Irp->IoStatus.Status;
	}
	else
	{
		switch (IrpStack->Parameters.QueryId.IdType)
		{
		case BusQueryDeviceID:
		case BusQueryHardwareIDs:
			//
			// HIDClass is asking for child deviceid & hardwareids.
			// Let us just make up some id for our child device.
			//
			buffer = (PWCHAR)ExAllocatePoolWithTag(
				NonPagedPool,
				RTI2S_HARDWARE_IDS_LENGTH,
				RTI2S_POOL_TAG
				);

			if (buffer)
			{
				//
				// Do the copy, store the buffer in the Irp
				//
				RtlCopyMemory(buffer,
					RTI2S_HARDWARE_IDS,
					RTI2S_HARDWARE_IDS_LENGTH
					);

				Irp->IoStatus.Information = (ULONG_PTR)buffer;
				status = STATUS_SUCCESS;
			}
			else
			{
				//
				//  No memory
				//
				status = STATUS_INSUFFICIENT_RESOURCES;
			}

			Irp->IoStatus.Status = status;
			//
			// We don't need to forward this to our bus. This query
			// is for our child so we should complete it right here.
			// fallthru.
			//
			IoCompleteRequest(Irp, IO_NO_INCREMENT);

			break;

		default:
			status = Irp->IoStatus.Status;
			IoCompleteRequest(Irp, IO_NO_INCREMENT);
			break;
		}
	}

	RtI2SPrint(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"RtI2SEvtWdmPreprocessMnQueryId Exit = 0x%x\n", status);

	return status;
}

VOID
RtI2SEvtInternalDeviceControl(
IN WDFQUEUE     Queue,
IN WDFREQUEST   Request,
IN size_t       OutputBufferLength,
IN size_t       InputBufferLength,
IN ULONG        IoControlCode
)
{
	NTSTATUS            status = STATUS_SUCCESS;
	WDFDEVICE           device;
	PRTI2S_CONTEXT     devContext;

	UNREFERENCED_PARAMETER(OutputBufferLength);
	UNREFERENCED_PARAMETER(InputBufferLength);

	device = WdfIoQueueGetDevice(Queue);
	devContext = GetDeviceContext(device);

	switch (IoControlCode)
	{
	default:
		status = STATUS_NOT_SUPPORTED;
		break;
	}

	WdfRequestComplete(Request, status);

	return;
}
