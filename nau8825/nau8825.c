#define DESCRIPTOR_DEF
#include "nau8825.h"
#include "registers.h"

#define bool int

static ULONG Nau8825DebugLevel = 100;
static ULONG Nau8825DebugCatagories = DBG_INIT || DBG_PNP || DBG_IOCTL;

NTSTATUS
DriverEntry(
	__in PDRIVER_OBJECT  DriverObject,
	__in PUNICODE_STRING RegistryPath
)
{
	NTSTATUS               status = STATUS_SUCCESS;
	WDF_DRIVER_CONFIG      config;
	WDF_OBJECT_ATTRIBUTES  attributes;

	Nau8825Print(DEBUG_LEVEL_INFO, DBG_INIT,
		"Driver Entry\n");

	WDF_DRIVER_CONFIG_INIT(&config, Nau8825EvtDeviceAdd);

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
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_INIT,
			"WdfDriverCreate failed with status 0x%x\n", status);
	}

	return status;
}

NTSTATUS nau8825_reg_read(
	_In_ PNAU8825_CONTEXT pDevice,
	uint16_t reg,
	unsigned int* data
) {
	reg = RtlUshortByteSwap(reg);
	uint16_t raw_data = 0;
	NTSTATUS status = SpbXferDataSynchronously(&pDevice->I2CContext, &reg, sizeof(uint16_t), &raw_data, sizeof(uint16_t));
	raw_data = RtlUshortByteSwap(raw_data);
	*data = raw_data;
	return status;
}

NTSTATUS nau8825_reg_write(
	_In_ PNAU8825_CONTEXT pDevice,
	uint16_t reg,
	unsigned int data
) {
	reg = RtlUshortByteSwap(reg);
	data = RtlUshortByteSwap(data);

	uint16_t buf[2];
	buf[0] = reg;
	buf[1] = data;
	return SpbWriteDataSynchronously(&pDevice->I2CContext, buf, sizeof(buf));
}

NTSTATUS nau8825_reg_update(
	_In_ PNAU8825_CONTEXT pDevice,
	uint16_t reg,
	unsigned int mask,
	unsigned int val
) {
	unsigned int tmp = 0, orig = 0;

	NTSTATUS status = nau8825_reg_read(pDevice, reg, &orig);
	if (!NT_SUCCESS(status)) {
		return status;
	}

	tmp = orig & ~mask;
	tmp |= val & mask;

	if (tmp != orig) {
		status = nau8825_reg_write(pDevice, reg, tmp);
	}
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
	PNAU8825_CONTEXT pDevice = GetDeviceContext(FxDevice);
	BOOLEAN fSpbResourceFound = FALSE;
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
	PNAU8825_CONTEXT pDevice = GetDeviceContext(FxDevice);
	NTSTATUS status = STATUS_SUCCESS;

	UNREFERENCED_PARAMETER(FxResourcesTranslated);

	SpbTargetDeinitialize(FxDevice, &pDevice->I2CContext);

	return status;
}

static NTSTATUS GetIntegerProperty(
	_In_ WDFDEVICE FxDevice,
	char* propertyStr,
	UINT8* property
) {
	PNAU8825_CONTEXT pDevice = GetDeviceContext(FxDevice);
	WDFMEMORY outputMemory = WDF_NO_HANDLE;

	NTSTATUS status = STATUS_ACPI_NOT_INITIALIZED;

	size_t inputBufferLen = sizeof(ACPI_GET_DEVICE_SPECIFIC_DATA) + strlen(propertyStr) + 1;
	ACPI_GET_DEVICE_SPECIFIC_DATA* inputBuffer = ExAllocatePoolWithTag(NonPagedPool, inputBufferLen, NAU8825_POOL_TAG);
	if (!inputBuffer) {
		goto Exit;
	}
	RtlZeroMemory(inputBuffer, inputBufferLen);

	inputBuffer->Signature = IOCTL_ACPI_GET_DEVICE_SPECIFIC_DATA_SIGNATURE;

	unsigned char uuidend[] = { 0x8a, 0x91, 0xbc, 0x9b, 0xbf, 0x4a, 0xa3, 0x01 };

	inputBuffer->Section.Data1 = 0xdaffd814;
	inputBuffer->Section.Data2 = 0x6eba;
	inputBuffer->Section.Data3 = 0x4d8c;
	memcpy(inputBuffer->Section.Data4, uuidend, sizeof(uuidend)); //Avoid Windows defender false positive

	strcpy(inputBuffer->PropertyName, propertyStr);
	inputBuffer->PropertyNameLength = strlen(propertyStr) + 1;

	PACPI_EVAL_OUTPUT_BUFFER outputBuffer;
	size_t outputArgumentBufferSize = 8;
	size_t outputBufferSize = FIELD_OFFSET(ACPI_EVAL_OUTPUT_BUFFER, Argument) + sizeof(ACPI_METHOD_ARGUMENT_V1) + outputArgumentBufferSize;
	sizeof(ACPI_EVAL_OUTPUT_BUFFER_V1);

	WDF_OBJECT_ATTRIBUTES attributes;
	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
	attributes.ParentObject = FxDevice;
	status = WdfMemoryCreate(&attributes,
		NonPagedPoolNx,
		0,
		outputBufferSize,
		&outputMemory,
		&outputBuffer);
	if (!NT_SUCCESS(status)) {
		goto Exit;
	}

	WDF_MEMORY_DESCRIPTOR inputMemDesc;
	WDF_MEMORY_DESCRIPTOR outputMemDesc;
	WDF_MEMORY_DESCRIPTOR_INIT_BUFFER(&inputMemDesc, inputBuffer, (ULONG)inputBufferLen);
	WDF_MEMORY_DESCRIPTOR_INIT_HANDLE(&outputMemDesc, outputMemory, NULL);

	status = WdfIoTargetSendInternalIoctlSynchronously(
		WdfDeviceGetIoTarget(FxDevice),
		NULL,
		IOCTL_ACPI_GET_DEVICE_SPECIFIC_DATA,
		&inputMemDesc,
		&outputMemDesc,
		NULL,
		NULL
	);
	if (!NT_SUCCESS(status)) {
		Nau8825Print(
			DEBUG_LEVEL_ERROR,
			DBG_IOCTL,
			"Error getting device data for key %s - 0x%x\n",
			propertyStr,
			status);
		goto Exit;
	}

	if (outputBuffer->Signature != ACPI_EVAL_OUTPUT_BUFFER_SIGNATURE_V1 &&
		outputBuffer->Count < 1 &&
		outputBuffer->Argument->Type != ACPI_METHOD_ARGUMENT_INTEGER &&
		outputBuffer->Argument->DataLength < 1) {
		status = STATUS_ACPI_INVALID_ARGUMENT;
		goto Exit;
	}

	if (property) {
		*property = outputBuffer->Argument->Data[0] & 0xF;
	}

Exit:
	if (inputBuffer) {
		ExFreePoolWithTag(inputBuffer, NAU8825_POOL_TAG);
	}
	if (outputMemory != WDF_NO_HANDLE) {
		WdfObjectDelete(outputMemory);
	}
	return status;
}

static NTSTATUS GetIntegerArrayProperty(
	_In_ WDFDEVICE FxDevice,
	char* propertyStr,
	UINT8 arrayCnt,
	UINT8 *propertyArray
) {
	PNAU8825_CONTEXT pDevice = GetDeviceContext(FxDevice);
	WDFMEMORY outputMemory = WDF_NO_HANDLE;

	NTSTATUS status = STATUS_ACPI_NOT_INITIALIZED;

	size_t inputBufferLen = sizeof(ACPI_GET_DEVICE_SPECIFIC_DATA) + strlen(propertyStr) + 1;
	ACPI_GET_DEVICE_SPECIFIC_DATA* inputBuffer = ExAllocatePoolWithTag(NonPagedPool, inputBufferLen, NAU8825_POOL_TAG);
	if (!inputBuffer) {
		goto Exit;
	}
	RtlZeroMemory(inputBuffer, inputBufferLen);

	inputBuffer->Signature = IOCTL_ACPI_GET_DEVICE_SPECIFIC_DATA_SIGNATURE;

	unsigned char uuidend[] = { 0x8a, 0x91, 0xbc, 0x9b, 0xbf, 0x4a, 0xa3, 0x01 };

	inputBuffer->Section.Data1 = 0xdaffd814;
	inputBuffer->Section.Data2 = 0x6eba;
	inputBuffer->Section.Data3 = 0x4d8c;
	memcpy(inputBuffer->Section.Data4, uuidend, sizeof(uuidend)); //Avoid Windows defender false positive

	strcpy(inputBuffer->PropertyName, propertyStr);
	inputBuffer->PropertyNameLength = strlen(propertyStr) + 1;

	PACPI_EVAL_OUTPUT_BUFFER outputBuffer;
	size_t outputArgumentBufferSize = (sizeof(ACPI_METHOD_ARGUMENT_V1) + 4) * arrayCnt;
	size_t outputBufferSize = FIELD_OFFSET(ACPI_EVAL_OUTPUT_BUFFER, Argument) + outputArgumentBufferSize;

	WDF_OBJECT_ATTRIBUTES attributes;
	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
	attributes.ParentObject = FxDevice;
	status = WdfMemoryCreate(&attributes,
		NonPagedPoolNx,
		0,
		outputBufferSize,
		&outputMemory,
		&outputBuffer);
	if (!NT_SUCCESS(status)) {
		goto Exit;
	}

	WDF_MEMORY_DESCRIPTOR inputMemDesc;
	WDF_MEMORY_DESCRIPTOR outputMemDesc;
	WDF_MEMORY_DESCRIPTOR_INIT_BUFFER(&inputMemDesc, inputBuffer, (ULONG)inputBufferLen);
	WDF_MEMORY_DESCRIPTOR_INIT_HANDLE(&outputMemDesc, outputMemory, NULL);

	status = WdfIoTargetSendInternalIoctlSynchronously(
		WdfDeviceGetIoTarget(FxDevice),
		NULL,
		IOCTL_ACPI_GET_DEVICE_SPECIFIC_DATA,
		&inputMemDesc,
		&outputMemDesc,
		NULL,
		NULL
	);
	if (!NT_SUCCESS(status)) {
		Nau8825Print(
			DEBUG_LEVEL_ERROR,
			DBG_IOCTL,
			"Error getting device data for key %s - 0x%x\n",
			propertyStr,
			status);
		goto Exit;
	}

	if (outputBuffer->Signature != ACPI_EVAL_OUTPUT_BUFFER_SIGNATURE_V1 &&
		outputBuffer->Count < arrayCnt &&
		outputBuffer->Argument->Type != ACPI_METHOD_ARGUMENT_PACKAGE) {
		status = STATUS_ACPI_INVALID_ARGUMENT;
		goto Exit;
	}

	UINT8 i = 0;
	FOR_EACH_ACPI_METHOD_ARGUMENT(arrParameter, outputBuffer->Argument, (UINT8 *)outputBuffer + outputBuffer->Length){
		if (propertyArray) {
			propertyArray[i] = arrParameter->Data[0] & 0xFF;
		}
		i++;
	}

Exit:
	if (inputBuffer) {
		ExFreePoolWithTag(inputBuffer, NAU8825_POOL_TAG);
	}
	if (outputMemory != WDF_NO_HANDLE) {
		WdfObjectDelete(outputMemory);
	}
	return status;
}

static void nau8825_reset_chip(PNAU8825_CONTEXT pDevice) {
	nau8825_reg_write(pDevice, NAU8825_REG_RESET, 0x00);
	nau8825_reg_write(pDevice, NAU8825_REG_RESET, 0x00);
}

static void nau8825_setup_buttons(PNAU8825_CONTEXT pDevice) {
	nau8825_reg_update(pDevice, NAU8825_REG_SAR_CTRL,
		NAU8825_SAR_TRACKING_GAIN_MASK,
		pDevice->sar_voltage << NAU8825_SAR_TRACKING_GAIN_SFT);
	nau8825_reg_update(pDevice, NAU8825_REG_SAR_CTRL,
		NAU8825_SAR_COMPARE_TIME_MASK,
		pDevice->sar_compare_time << NAU8825_SAR_COMPARE_TIME_SFT);
	nau8825_reg_update(pDevice, NAU8825_REG_SAR_CTRL,
		NAU8825_SAR_SAMPLING_TIME_MASK,
		pDevice->sar_sampling_time << NAU8825_SAR_SAMPLING_TIME_SFT);

	nau8825_reg_update(pDevice, NAU8825_REG_KEYDET_CTRL,
		NAU8825_KEYDET_LEVELS_NR_MASK,
		(pDevice->sar_threshold_num - 1) << NAU8825_KEYDET_LEVELS_NR_SFT);
	nau8825_reg_update(pDevice, NAU8825_REG_KEYDET_CTRL,
		NAU8825_KEYDET_HYSTERESIS_MASK,
		pDevice->sar_hysteresis << NAU8825_KEYDET_HYSTERESIS_SFT);
	nau8825_reg_update(pDevice, NAU8825_REG_KEYDET_CTRL,
		NAU8825_KEYDET_SHORTKEY_DEBOUNCE_MASK,
		pDevice->key_debounce << NAU8825_KEYDET_SHORTKEY_DEBOUNCE_SFT);

	nau8825_reg_write(pDevice, NAU8825_REG_VDET_THRESHOLD_1,
		(pDevice->sar_threshold[0] << 8) | pDevice->sar_threshold[1]);
	nau8825_reg_write(pDevice, NAU8825_REG_VDET_THRESHOLD_2,
		(pDevice->sar_threshold[2] << 8) | pDevice->sar_threshold[3]);
	nau8825_reg_write(pDevice, NAU8825_REG_VDET_THRESHOLD_3,
		(pDevice->sar_threshold[4] << 8) | pDevice->sar_threshold[5]);
	nau8825_reg_write(pDevice, NAU8825_REG_VDET_THRESHOLD_4,
		(pDevice->sar_threshold[6] << 8) | pDevice->sar_threshold[7]);

	/* Enable short press and release interruptions */
	nau8825_reg_update(pDevice, NAU8825_REG_INTERRUPT_MASK,
		NAU8825_IRQ_KEY_SHORT_PRESS_EN | NAU8825_IRQ_KEY_RELEASE_EN,
		0);
}

static bool nau8825_is_jack_inserted(PNAU8825_CONTEXT pDevice) {
	bool active_high, is_high;
	int status, jkdet;

	nau8825_reg_read(pDevice, NAU8825_REG_JACK_DET_CTRL, &jkdet);
	active_high = jkdet & NAU8825_JACK_POLARITY;
	nau8825_reg_read(pDevice, NAU8825_REG_I2C_DEVICE_ID, &status);
	is_high = status & NAU8825_GPIO2JD1;
	/* return jack connection status according to jack insertion logic
	 * active high or active low.
	 */
	return active_high == is_high;
}

static void nau8825_configure_mclk_as_sysclk(PNAU8825_CONTEXT pDevice)
{
	nau8825_reg_update(pDevice, NAU8825_REG_CLK_DIVIDER,
		NAU8825_CLK_SRC_MASK, NAU8825_CLK_SRC_MCLK);
	nau8825_reg_update(pDevice, NAU8825_REG_FLL6,
		NAU8825_DCO_EN, 0);
	/* Make DSP operate as default setting for power saving. */
	nau8825_reg_update(pDevice, NAU8825_REG_FLL1,
		NAU8825_ICTRL_LATCH_MASK, 0);
}

/**
 * nau8825_enable_jack_detect - Specify a jack for event reporting
 *
 * @component:  component to register the jack with
 * @jack: jack to use to report headset and button events on
 *
 * After this function has been called the headset insert/remove and button
 * events will be routed to the given jack.  Jack can be null to stop
 * reporting.
 */
NTSTATUS nau8825_enable_jack_detect(PNAU8825_CONTEXT pDevice)
{
	/* Ground HP Outputs[1:0], needed for headset auto detection
	 * Enable Automatic Mic/Gnd switching reading on insert interrupt[6]
	 */
	nau8825_reg_update(pDevice, NAU8825_REG_HSD_CTRL,
		NAU8825_HSD_AUTO_MODE | NAU8825_SPKR_DWN1R | NAU8825_SPKR_DWN1L,
		NAU8825_HSD_AUTO_MODE | NAU8825_SPKR_DWN1R | NAU8825_SPKR_DWN1L);

	return STATUS_SUCCESS;
}

void nau8825_int_status_clear_all(PNAU8825_CONTEXT pDevice)
{
	int active_irq, clear_irq, i;

	/* Reset the intrruption status from rightmost bit if the corres-
	 * ponding irq event occurs.
	 */
	nau8825_reg_read(pDevice, NAU8825_REG_IRQ_STATUS, &active_irq);
	for (i = 0; i < NAU8825_REG_DATA_LEN; i++) {
		clear_irq = (0x1 << i);
		if (active_irq & clear_irq)
			nau8825_reg_write(pDevice,
				NAU8825_REG_INT_CLR_KEY_STATUS, clear_irq);
	}
}

static NTSTATUS nau8825_load_settings(PNAU8825_CONTEXT pDevice) {
	NTSTATUS status;
	RtlZeroMemory(&pDevice->sar_threshold, sizeof(pDevice->sar_threshold));

	//Default from Lars (for now)
	/*pDevice->jkdet_enable = 1;
	pDevice->jkdet_pull_enable = 1;
	pDevice->jkdet_pull_up = 1;
	pDevice->jkdet_polarity = 1;
	pDevice->vref_impedance = 2;
	pDevice->micbias_voltage = 6;
	pDevice->sar_threshold_num = 4;
	pDevice->sar_threshold[0] = 0x08;
	pDevice->sar_threshold[1] = 0x12;
	pDevice->sar_threshold[2] = 0x26;
	pDevice->sar_threshold[3] = 0x73;
	pDevice->sar_hysteresis = 0;
	pDevice->sar_voltage = 6;
	pDevice->sar_compare_time = 1;
	pDevice->sar_sampling_time = 1;
	pDevice->key_debounce = 3;
	pDevice->jack_insert_debounce = 7;
	pDevice->jack_eject_debounce = 0;*/

	status = GetIntegerProperty(pDevice->FxDevice, "nuvoton,jkdet-enable", &pDevice->jkdet_enable);
	if (!NT_SUCCESS(status)) {
		return status;
	}
	status = GetIntegerProperty(pDevice->FxDevice, "nuvoton,jkdet-pull-enable", &pDevice->jkdet_pull_enable);
	if (!NT_SUCCESS(status)) {
		return status;
	}
	status = GetIntegerProperty(pDevice->FxDevice, "nuvoton,jkdet-pull-up", &pDevice->jkdet_pull_up);
	if (!NT_SUCCESS(status)) {
		return status;
	}
	status = GetIntegerProperty(pDevice->FxDevice, "nuvoton,jkdet-polarity", &pDevice->jkdet_polarity);
	if (!NT_SUCCESS(status)) {
		return status;
	}
	status = GetIntegerProperty(pDevice->FxDevice, "nuvoton,vref-impedance", &pDevice->vref_impedance);
	if (!NT_SUCCESS(status)) {
		return status;
	}
	status = GetIntegerProperty(pDevice->FxDevice, "nuvoton,micbias-voltage", &pDevice->micbias_voltage);
	if (!NT_SUCCESS(status)) {
		return status;
	}
	status = GetIntegerProperty(pDevice->FxDevice, "nuvoton,sar-threshold-num", &pDevice->sar_threshold_num);
	if (!NT_SUCCESS(status)) {
		return status;
	}

	status = GetIntegerArrayProperty(pDevice->FxDevice, "nuvoton,sar-threshold", pDevice->sar_threshold_num, pDevice->sar_threshold);
	if (!NT_SUCCESS(status)) {
		return status;
	}

	//TODO: Get SAR threshold

	status = GetIntegerProperty(pDevice->FxDevice, "nuvoton,sar-hysteresis", &pDevice->sar_hysteresis);
	if (!NT_SUCCESS(status)) {
		return status;
	}
	status = GetIntegerProperty(pDevice->FxDevice, "nuvoton,sar-voltage", &pDevice->sar_voltage);
	if (!NT_SUCCESS(status)) {
		return status;
	}
	status = GetIntegerProperty(pDevice->FxDevice, "nuvoton,sar-compare-time", &pDevice->sar_compare_time);
	if (!NT_SUCCESS(status)) {
		return status;
	}
	status = GetIntegerProperty(pDevice->FxDevice, "nuvoton,sar-sampling-time", &pDevice->sar_sampling_time);
	if (!NT_SUCCESS(status)) {
		return status;
	}
	status = GetIntegerProperty(pDevice->FxDevice, "nuvoton,short-key-debounce", &pDevice->key_debounce);
	if (!NT_SUCCESS(status)) {
		return status;
	}
	status = GetIntegerProperty(pDevice->FxDevice, "nuvoton,jack-insert-debounce", &pDevice->jack_insert_debounce);
	if (!NT_SUCCESS(status)) {
		return status;
	}
	status = GetIntegerProperty(pDevice->FxDevice, "nuvoton,jack-eject-debounce", &pDevice->jack_eject_debounce);
	if (!NT_SUCCESS(status)) {
		status = GetIntegerProperty(pDevice->FxDevice, "nuvoton,jack-eject-deboune", &pDevice->jack_eject_debounce); //Google made a typo lmaooo
		if (!NT_SUCCESS(status)) {
			return status;
		}
	}
	return status;
}

static void nau8825_init_regs(PNAU8825_CONTEXT pDevice) {
	/* Latch IIC LSB value */
	nau8825_reg_write(pDevice, NAU8825_REG_IIC_ADDR_SET, 0x0001);
	/* Enable Bias/Vmid */
	nau8825_reg_update(pDevice, NAU8825_REG_BIAS_ADJ,
		NAU8825_BIAS_VMID, NAU8825_BIAS_VMID);
	nau8825_reg_update(pDevice, NAU8825_REG_BOOST,
		NAU8825_GLOBAL_BIAS_EN, NAU8825_GLOBAL_BIAS_EN);

	/* VMID Tieoff */
	nau8825_reg_update(pDevice, NAU8825_REG_BIAS_ADJ,
		NAU8825_BIAS_VMID_SEL_MASK,
		pDevice->vref_impedance << NAU8825_BIAS_VMID_SEL_SFT);
	/* Disable Boost Driver, Automatic Short circuit protection enable */
	nau8825_reg_update(pDevice, NAU8825_REG_BOOST,
		NAU8825_PRECHARGE_DIS | NAU8825_HP_BOOST_DIS |
		NAU8825_HP_BOOST_G_DIS | NAU8825_SHORT_SHUTDOWN_EN,
		NAU8825_PRECHARGE_DIS | NAU8825_HP_BOOST_DIS |
		NAU8825_HP_BOOST_G_DIS | NAU8825_SHORT_SHUTDOWN_EN);

	nau8825_reg_update(pDevice, NAU8825_REG_GPIO12_CTRL,
		NAU8825_JKDET_OUTPUT_EN,
		pDevice->jkdet_enable ? 0 : NAU8825_JKDET_OUTPUT_EN);
	nau8825_reg_update(pDevice, NAU8825_REG_GPIO12_CTRL,
		NAU8825_JKDET_PULL_EN,
		pDevice->jkdet_pull_enable ? 0 : NAU8825_JKDET_PULL_EN);
	nau8825_reg_update(pDevice, NAU8825_REG_GPIO12_CTRL,
		NAU8825_JKDET_PULL_UP,
		pDevice->jkdet_pull_up ? NAU8825_JKDET_PULL_UP : 0);
	nau8825_reg_update(pDevice, NAU8825_REG_JACK_DET_CTRL,
		NAU8825_JACK_POLARITY,
		/* jkdet_polarity - 1  is for active-low */
		pDevice->jkdet_polarity ? 0 : NAU8825_JACK_POLARITY);

	nau8825_reg_update(pDevice, NAU8825_REG_JACK_DET_CTRL,
		NAU8825_JACK_INSERT_DEBOUNCE_MASK,
		pDevice->jack_insert_debounce << NAU8825_JACK_INSERT_DEBOUNCE_SFT);
	nau8825_reg_update(pDevice, NAU8825_REG_JACK_DET_CTRL,
		NAU8825_JACK_EJECT_DEBOUNCE_MASK,
		pDevice->jack_eject_debounce << NAU8825_JACK_EJECT_DEBOUNCE_SFT);

	/* Pull up IRQ pin */
	nau8825_reg_update(pDevice, NAU8825_REG_INTERRUPT_MASK,
		NAU8825_IRQ_PIN_PULLUP | NAU8825_IRQ_PIN_PULL_EN,
		NAU8825_IRQ_PIN_PULLUP | NAU8825_IRQ_PIN_PULL_EN);
	/* Mask unneeded IRQs: 1 - disable, 0 - enable */
	nau8825_reg_update(pDevice, NAU8825_REG_INTERRUPT_MASK, 0x7ff, 0x7ff);

	nau8825_reg_update(pDevice, NAU8825_REG_MIC_BIAS,
		NAU8825_MICBIAS_VOLTAGE_MASK, pDevice->micbias_voltage);

	if (pDevice->sar_threshold_num)
		nau8825_setup_buttons(pDevice);

	/* Default oversampling/decimations settings are unusable
	 * (audible hiss). Set it to something better.
	 */
	nau8825_reg_update(pDevice, NAU8825_REG_ADC_RATE,
		NAU8825_ADC_SYNC_DOWN_MASK | NAU8825_ADC_SINC4_EN,
		NAU8825_ADC_SYNC_DOWN_64);
	nau8825_reg_update(pDevice, NAU8825_REG_DAC_CTRL1,
		NAU8825_DAC_OVERSAMPLE_MASK, NAU8825_DAC_OVERSAMPLE_64);
	/* Disable DACR/L power */
	nau8825_reg_update(pDevice, NAU8825_REG_CHARGE_PUMP,
		NAU8825_POWER_DOWN_DACR | NAU8825_POWER_DOWN_DACL,
		NAU8825_POWER_DOWN_DACR | NAU8825_POWER_DOWN_DACL);
	/* Enable TESTDAC. This sets the analog DAC inputs to a '0' input
	 * signal to avoid any glitches due to power up transients in both
	 * the analog and digital DAC circuit.
	 */
	nau8825_reg_update(pDevice, NAU8825_REG_BIAS_ADJ,
		NAU8825_BIAS_TESTDAC_EN, NAU8825_BIAS_TESTDAC_EN);
	/* CICCLP off */
	nau8825_reg_update(pDevice, NAU8825_REG_DAC_CTRL1,
		NAU8825_DAC_CLIP_OFF, NAU8825_DAC_CLIP_OFF);

	/* Class AB bias current to 2x, DAC Capacitor enable MSB/LSB */
	nau8825_reg_update(pDevice, NAU8825_REG_ANALOG_CONTROL_2,
		NAU8825_HP_NON_CLASSG_CURRENT_2xADJ |
		NAU8825_DAC_CAPACITOR_MSB | NAU8825_DAC_CAPACITOR_LSB,
		NAU8825_HP_NON_CLASSG_CURRENT_2xADJ |
		NAU8825_DAC_CAPACITOR_MSB | NAU8825_DAC_CAPACITOR_LSB);
	/* Class G timer 64ms */
	nau8825_reg_update(pDevice, NAU8825_REG_CLASSG_CTRL,
		NAU8825_CLASSG_TIMER_MASK,
		0x20 << NAU8825_CLASSG_TIMER_SFT);
	/* DAC clock delay 2ns, VREF */
	nau8825_reg_update(pDevice, NAU8825_REG_RDAC,
		NAU8825_RDAC_CLK_DELAY_MASK | NAU8825_RDAC_VREF_MASK,
		(0x2 << NAU8825_RDAC_CLK_DELAY_SFT) |
		(0x3 << NAU8825_RDAC_VREF_SFT));
	/* Config L/R channel */
	nau8825_reg_update(pDevice, NAU8825_REG_DACL_CTRL,
		NAU8825_DACL_CH_SEL_MASK, NAU8825_DACL_CH_SEL_L);
	nau8825_reg_update(pDevice, NAU8825_REG_DACR_CTRL,
		NAU8825_DACL_CH_SEL_MASK, NAU8825_DACL_CH_SEL_R);
	/* Disable short Frame Sync detection logic */
	nau8825_reg_update(pDevice, NAU8825_REG_LEFT_TIME_SLOT,
		NAU8825_DIS_FS_SHORT_DET, NAU8825_DIS_FS_SHORT_DET);

	{
		//Resume setup

		/* Clock provided externally and disable internal VCO clock */
		nau8825_configure_mclk_as_sysclk(pDevice);

		nau8825_int_status_clear_all(pDevice);

		/* Enable both insertion and ejection interruptions, and then
		 * bypass de-bounce circuit.
		 */
		nau8825_reg_update(pDevice, NAU8825_REG_INTERRUPT_MASK,
			NAU8825_IRQ_OUTPUT_EN | NAU8825_IRQ_HEADSET_COMPLETE_EN |
			NAU8825_IRQ_EJECT_EN | NAU8825_IRQ_INSERT_EN,
			NAU8825_IRQ_OUTPUT_EN | NAU8825_IRQ_HEADSET_COMPLETE_EN);
		nau8825_reg_update(pDevice, NAU8825_REG_JACK_DET_CTRL,
			NAU8825_JACK_DET_DB_BYPASS, NAU8825_JACK_DET_DB_BYPASS);
		nau8825_reg_update(pDevice, NAU8825_REG_INTERRUPT_DIS_CTRL,
			NAU8825_IRQ_INSERT_DIS | NAU8825_IRQ_EJECT_DIS, 0);
	}

	nau8825_enable_jack_detect(pDevice);

	{
		//Set sane defaults from Linux
		nau8825_reg_update(pDevice, NAU8825_REG_ENA_CTRL, 0x0700, 0x0700); //nothing running is 0x013f, headphones is 0x077f, both running is 0x07ff

		nau8825_reg_update(pDevice, NAU8825_REG_CLK_DIVIDER, 0xf0, 0x90);

		nau8825_reg_update(pDevice, NAU8825_REG_INTERRUPT_MASK, 0xf, 0xe);

		nau8825_reg_update(pDevice, NAU8825_REG_I2S_PCM_CTRL1, 0xf, 0xa);

		nau8825_reg_update(pDevice, NAU8825_REG_ADC_DGAIN_CTRL, 0xf0, 0xf0);

		nau8825_reg_update(pDevice, NAU8825_REG_CHARGE_PUMP, 0xf00, 0x300);
	}
}

VOID
NAU8825BootWorkItem(
	IN WDFWORKITEM  WorkItem
)
{
	WDFDEVICE Device = (WDFDEVICE)WdfWorkItemGetParentObject(WorkItem);
	PNAU8825_CONTEXT pDevice = GetDeviceContext(Device);

	nau8825_reset_chip(pDevice);
	int value;
	NTSTATUS status = nau8825_reg_read(pDevice, NAU8825_REG_I2C_DEVICE_ID, &value);

	if ((value & NAU8825_SOFTWARE_ID_MASK) !=
		NAU8825_SOFTWARE_ID_NAU8825) {
		DbgPrint("Not a NAU8825 chip\n");
		return;
	}

	nau8825_init_regs(pDevice);

	pDevice->DevicePoweredOn = TRUE;

end:
	WdfObjectDelete(WorkItem);
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

	PNAU8825_CONTEXT pDevice = GetDeviceContext(FxDevice);
	NTSTATUS status = nau8825_load_settings(pDevice);

	if (!NT_SUCCESS(status)) {
		return status;
	}

	WDF_OBJECT_ATTRIBUTES attributes;
	WDF_WORKITEM_CONFIG workitemConfig;
	WDFWORKITEM hWorkItem;

	WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
	WDF_OBJECT_ATTRIBUTES_SET_CONTEXT_TYPE(&attributes, NAU8825_CONTEXT);
	attributes.ParentObject = pDevice->FxDevice;
	WDF_WORKITEM_CONFIG_INIT(&workitemConfig, NAU8825BootWorkItem);

	WdfWorkItemCreate(&workitemConfig,
		&attributes,
		&hWorkItem);

	WdfWorkItemEnqueue(hWorkItem);

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

	PNAU8825_CONTEXT pDevice = GetDeviceContext(FxDevice);
	NTSTATUS status = STATUS_SUCCESS;

	pDevice->DevicePoweredOn = FALSE;

	return STATUS_SUCCESS;
}

static void nau8825_restart_jack_detection(PNAU8825_CONTEXT pDevice)
{
	/* this will restart the entire jack detection process including MIC/GND
	 * switching and create interrupts. We have to go from 0 to 1 and back
	 * to 0 to restart.
	 */
	nau8825_reg_update(pDevice, NAU8825_REG_JACK_DET_CTRL,
		NAU8825_JACK_DET_RESTART, NAU8825_JACK_DET_RESTART);
	nau8825_reg_update(pDevice, NAU8825_REG_JACK_DET_CTRL,
		NAU8825_JACK_DET_RESTART, 0);
}

static NTSTATUS nau8825_configure_sysclk(PNAU8825_CONTEXT pDevice, int clk_id,
	unsigned int freq)
{
	NTSTATUS status;

	switch (clk_id) {
	case NAU8825_CLK_DIS:
		/* Clock provided externally and disable internal VCO clock */
		nau8825_configure_mclk_as_sysclk(pDevice);
		break;
	case NAU8825_CLK_MCLK:
		/* Acquire the semaphore to synchronize the playback and
		 * interrupt handler. In order to avoid the playback inter-
		 * fered by cross talk process, the driver make the playback
		 * preparation halted until cross talk process finish.
		 */
		nau8825_configure_mclk_as_sysclk(pDevice);
		/* MCLK not changed by clock tree */
		nau8825_reg_update(pDevice, NAU8825_REG_CLK_DIVIDER,
			NAU8825_CLK_MCLK_SRC_MASK, 0);

		break;
	case NAU8825_CLK_INTERNAL:
		if (nau8825_is_jack_inserted(pDevice)) {
			nau8825_reg_update(pDevice, NAU8825_REG_FLL6,
				NAU8825_DCO_EN, NAU8825_DCO_EN);
			nau8825_reg_update(pDevice, NAU8825_REG_CLK_DIVIDER,
				NAU8825_CLK_SRC_MASK, NAU8825_CLK_SRC_VCO);
			/* Decrease the VCO frequency and make DSP operate
			 * as default setting for power saving.
			 */
			nau8825_reg_update(pDevice, NAU8825_REG_CLK_DIVIDER,
				NAU8825_CLK_MCLK_SRC_MASK, 0xf);
			nau8825_reg_update(pDevice, NAU8825_REG_FLL1,
				NAU8825_ICTRL_LATCH_MASK |
				NAU8825_FLL_RATIO_MASK, 0x10);
			nau8825_reg_update(pDevice, NAU8825_REG_FLL6,
				NAU8825_SDM_EN, NAU8825_SDM_EN);
		}
		else {
			/* The clock turns off intentionally for power saving
			 * when no headset connected.
			 */
			nau8825_configure_mclk_as_sysclk(pDevice);
			Nau8825Print(DEBUG_LEVEL_INFO, DBG_IOCTL, "Disable clock for power saving when no headset connected\n");
		}
		break;
	case NAU8825_CLK_FLL_MCLK:
		/* Higher FLL reference input frequency can only set lower
		 * gain error, such as 0000 for input reference from MCLK
		 * 12.288Mhz.
		 */
		nau8825_reg_update(pDevice, NAU8825_REG_FLL3,
			NAU8825_FLL_CLK_SRC_MASK | NAU8825_GAIN_ERR_MASK,
			NAU8825_FLL_CLK_SRC_MCLK | 0);

		break;
	case NAU8825_CLK_FLL_BLK:
		/* If FLL reference input is from low frequency source,
		 * higher error gain can apply such as 0xf which has
		 * the most sensitive gain error correction threshold,
		 * Therefore, FLL has the most accurate DCO to
		 * target frequency.
		 */
		nau8825_reg_update(pDevice, NAU8825_REG_FLL3,
			NAU8825_FLL_CLK_SRC_MASK | NAU8825_GAIN_ERR_MASK,
			NAU8825_FLL_CLK_SRC_BLK |
			(0xf << NAU8825_GAIN_ERR_SFT));
		break;
	case NAU8825_CLK_FLL_FS:
		/* If FLL reference input is from low frequency source,
		 * higher error gain can apply such as 0xf which has
		 * the most sensitive gain error correction threshold,
		 * Therefore, FLL has the most accurate DCO to
		 * target frequency.
		 */
		nau8825_reg_update(pDevice, NAU8825_REG_FLL3,
			NAU8825_FLL_CLK_SRC_MASK | NAU8825_GAIN_ERR_MASK,
			NAU8825_FLL_CLK_SRC_FS |
			(0xf << NAU8825_GAIN_ERR_SFT));
		break;
	default:
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL, "Invalid clock id (%d)\n", clk_id);
		return STATUS_INVALID_DEVICE_STATE;
	}

	Nau8825Print(DEBUG_LEVEL_INFO, DBG_IOCTL, "Sysclk is %dHz and clock id is %d\n", freq,
		clk_id);
	return STATUS_SUCCESS;
}

/* Enable audo mode interruptions with internal clock. */
static void nau8825_setup_auto_irq(PNAU8825_CONTEXT pDevice)
{
	/* Enable headset jack type detection complete interruption and
	 * jack ejection interruption.
	 */
	nau8825_reg_update(pDevice, NAU8825_REG_INTERRUPT_MASK,
		NAU8825_IRQ_HEADSET_COMPLETE_EN | NAU8825_IRQ_EJECT_EN, 0);

	/* Enable internal VCO needed for interruptions */
	nau8825_configure_sysclk(pDevice, NAU8825_CLK_INTERNAL, 0);

	/* Enable ADC needed for interruptions */
	nau8825_reg_update(pDevice, NAU8825_REG_ENA_CTRL,
		NAU8825_ENABLE_ADC, NAU8825_ENABLE_ADC);

	/* Chip needs one FSCLK cycle in order to generate interruptions,
	 * as we cannot guarantee one will be provided by the system. Turning
	 * master mode on then off enables us to generate that FSCLK cycle
	 * with a minimum of contention on the clock bus.
	 */
	nau8825_reg_update(pDevice, NAU8825_REG_I2S_PCM_CTRL2,
		NAU8825_I2S_MS_MASK, NAU8825_I2S_MS_MASTER);
	nau8825_reg_update(pDevice, NAU8825_REG_I2S_PCM_CTRL2,
		NAU8825_I2S_MS_MASK, NAU8825_I2S_MS_SLAVE);

	/* Not bypass de-bounce circuit */
	nau8825_reg_update(pDevice, NAU8825_REG_JACK_DET_CTRL,
		NAU8825_JACK_DET_DB_BYPASS, 0);

	/* Unmask all interruptions */
	nau8825_reg_write(pDevice, NAU8825_REG_INTERRUPT_DIS_CTRL, 0);

	/* Restart the jack detection process at auto mode */
	nau8825_restart_jack_detection(pDevice);
}

static int nau8825_button_decode(int value)
{
	int buttons = 0;

	/* The chip supports up to 8 buttons, but HID defines only 4 buttons */
	buttons = value & 0xF;

	return buttons;
}

static void nau8825_eject_jack(PNAU8825_CONTEXT pDevice)
{
	{
		//Unset 2nd defaults set from Linux
		nau8825_reg_update(pDevice, NAU8825_REG_SAR_CTRL, NAU8825_SAR_ADC_EN, 0);

		nau8825_reg_update(pDevice, NAU8825_REG_CLASSG_CTRL, NAU8825_CLASSG_LDAC_EN | NAU8825_CLASSG_RDAC_EN | NAU8825_CLASSG_EN, 0);

		nau8825_reg_update(pDevice, NAU8825_REG_BIAS_ADJ, NAU8825_BIAS_TESTDAC_EN, NAU8825_BIAS_TESTDAC_EN);

		nau8825_reg_update(pDevice, NAU8825_REG_ANALOG_ADC_2, NAU8825_POWERUP_ADCL, 0);

		nau8825_reg_update(pDevice, NAU8825_REG_RDAC, NAU8825_RDAC_EN | NAU8825_RDAC_CLK_EN, 0);

		nau8825_reg_update(pDevice, NAU8825_REG_MIC_BIAS,
			(1 << 8),
			(0 << 8)); //Disable Mic Bias

		nau8825_reg_update(pDevice, NAU8825_REG_BOOST, NAU8825_HP_BOOST_DIS, NAU8825_HP_BOOST_DIS);

		nau8825_reg_update(pDevice, NAU8825_REG_POWER_UP_CONTROL,
			NAU8825_POWERUP_INTEGR_R | NAU8825_POWERUP_INTEGR_L | NAU8825_POWERUP_DRV_IN_R | NAU8825_POWERUP_DRV_IN_L | NAU8825_POWERUP_HP_DRV_R | NAU8825_POWERUP_HP_DRV_L,
			0);

		nau8825_reg_update(pDevice, NAU8825_REG_POWER_UP_CONTROL,
			(1 << 14),
			(0 << 14)); //Frontend PGA Disable

		nau8825_reg_update(pDevice, NAU8825_REG_CHARGE_PUMP,
			NAU8825_JAMNODCLOW | NAU8825_CHANRGE_PUMP_EN,
			0);

		nau8825_reg_update(pDevice, NAU8825_REG_CHARGE_PUMP,
			NAU8825_POWER_DOWN_DACR | NAU8825_POWER_DOWN_DACL,
			NAU8825_POWER_DOWN_DACR | NAU8825_POWER_DOWN_DACL);
	}

	/* Detach 2kOhm Resistors from MICBIAS to MICGND1/2 */
	nau8825_reg_update(pDevice, NAU8825_REG_MIC_BIAS,
		NAU8825_MICBIAS_JKSLV | NAU8825_MICBIAS_JKR2, 0);
	/* ground HPL/HPR, MICGRND1/2 */
	nau8825_reg_update(pDevice, NAU8825_REG_HSD_CTRL, 0xf, 0xf);

	/* Clear all interruption status */
	nau8825_int_status_clear_all(pDevice);

	/* Enable the insertion interruption, disable the ejection inter-
	 * ruption, and then bypass de-bounce circuit.
	 */
	nau8825_reg_update(pDevice, NAU8825_REG_INTERRUPT_DIS_CTRL,
		NAU8825_IRQ_EJECT_DIS | NAU8825_IRQ_INSERT_DIS,
		NAU8825_IRQ_EJECT_DIS);
	nau8825_reg_update(pDevice, NAU8825_REG_INTERRUPT_MASK,
		NAU8825_IRQ_OUTPUT_EN | NAU8825_IRQ_EJECT_EN |
		NAU8825_IRQ_HEADSET_COMPLETE_EN | NAU8825_IRQ_INSERT_EN,
		NAU8825_IRQ_OUTPUT_EN | NAU8825_IRQ_EJECT_EN |
		NAU8825_IRQ_HEADSET_COMPLETE_EN);
	nau8825_reg_update(pDevice, NAU8825_REG_JACK_DET_CTRL,
		NAU8825_JACK_DET_DB_BYPASS, NAU8825_JACK_DET_DB_BYPASS);

	/* Disable ADC needed for interruptions at audo mode */
	nau8825_reg_update(pDevice, NAU8825_REG_ENA_CTRL,
		NAU8825_ENABLE_ADC, 0);

	/* Close clock for jack type detection at manual mode */
	nau8825_configure_sysclk(pDevice, NAU8825_CLK_DIS, 0);
}

static int nau8825_jack_insert(PNAU8825_CONTEXT pDevice)
{
	int jack_status_reg, mic_detected;
	int type = 0;

	nau8825_reg_read(pDevice, NAU8825_REG_GENERAL_STATUS, &jack_status_reg);
	mic_detected = (jack_status_reg >> 10) & 3;

	switch (mic_detected) {
	case 0:
		/* no mic */
		type = SND_JACK_HEADPHONE;
		break;
	case 1:
		Nau8825Print(DEBUG_LEVEL_INFO, DBG_IOCTL, "OMTP (micgnd1) mic connected\n");
		type = SND_JACK_HEADSET;

		/* Unground MICGND1 */
		nau8825_reg_update(pDevice, NAU8825_REG_HSD_CTRL, 3 << 2,
			1 << 2);
		/* Attach 2kOhm Resistor from MICBIAS to MICGND1 */
		nau8825_reg_update(pDevice, NAU8825_REG_MIC_BIAS,
			NAU8825_MICBIAS_JKSLV | NAU8825_MICBIAS_JKR2,
			NAU8825_MICBIAS_JKR2);
		/* Attach SARADC to MICGND1 */
		nau8825_reg_update(pDevice, NAU8825_REG_SAR_CTRL,
			NAU8825_SAR_INPUT_MASK,
			NAU8825_SAR_INPUT_JKR2);
		break;
	case 2:
		Nau8825Print(DEBUG_LEVEL_INFO, DBG_IOCTL, "CTIA (micgnd2) mic connected\n");
		type = SND_JACK_HEADSET;

		/* Unground MICGND2 */
		nau8825_reg_update(pDevice, NAU8825_REG_HSD_CTRL, 3 << 2,
			2 << 2);
		/* Attach 2kOhm Resistor from MICBIAS to MICGND2 */
		nau8825_reg_update(pDevice, NAU8825_REG_MIC_BIAS,
			NAU8825_MICBIAS_JKSLV | NAU8825_MICBIAS_JKR2,
			NAU8825_MICBIAS_JKSLV);
		/* Attach SARADC to MICGND2 */
		nau8825_reg_update(pDevice, NAU8825_REG_SAR_CTRL,
			NAU8825_SAR_INPUT_MASK,
			NAU8825_SAR_INPUT_JKSLV);
		break;
	case 3:
		/* detect error case */
		DbgPrint("detection error; disable mic function\n");
		type = SND_JACK_HEADPHONE;
		break;
	} 

	{
		//Set 2nd defaults set from Linux
		nau8825_reg_update(pDevice, NAU8825_REG_CLK_DIVIDER, NAU8825_CLK_SRC_MASK | NAU8825_CLK_MCLK_SRC_MASK, 0x0);

		nau8825_reg_update(pDevice, NAU8825_REG_FLL6, NAU8825_DCO_EN, 0x0);

		nau8825_reg_update(pDevice, NAU8825_REG_HSD_CTRL, NAU8825_SPKR_DWN1R | NAU8825_SPKR_DWN1L, 0x0);

		nau8825_reg_update(pDevice, NAU8825_REG_SAR_CTRL, NAU8825_SAR_ADC_EN, NAU8825_SAR_ADC_EN);

		nau8825_reg_update(pDevice, NAU8825_REG_I2S_PCM_CTRL2, NAU8825_I2S_TRISTATE, 0);

		nau8825_reg_update(pDevice, NAU8825_REG_DAC_CTRL1, NAU8825_DAC_OVERSAMPLE_MASK, NAU8825_DAC_OVERSAMPLE_128);

		nau8825_reg_update(pDevice, NAU8825_REG_CLASSG_CTRL, NAU8825_CLASSG_LDAC_EN | NAU8825_CLASSG_RDAC_EN | NAU8825_CLASSG_EN, NAU8825_CLASSG_LDAC_EN | NAU8825_CLASSG_RDAC_EN | NAU8825_CLASSG_EN);
	
		nau8825_reg_update(pDevice, NAU8825_REG_BIAS_ADJ, NAU8825_BIAS_TESTDAC_EN, 0);

		nau8825_reg_update(pDevice, NAU8825_REG_ANALOG_ADC_2, NAU8825_POWERUP_ADCL, NAU8825_POWERUP_ADCL);

		nau8825_reg_update(pDevice, NAU8825_REG_RDAC, NAU8825_RDAC_EN | NAU8825_RDAC_CLK_EN, NAU8825_RDAC_EN | NAU8825_RDAC_CLK_EN);

		nau8825_reg_update(pDevice, NAU8825_REG_MIC_BIAS,
			(1 << 8),
			(1 << 8)); //Enable Mic Bias

		nau8825_reg_update(pDevice, NAU8825_REG_BOOST, NAU8825_HP_BOOST_DIS, 0);

		nau8825_reg_update(pDevice, NAU8825_REG_POWER_UP_CONTROL,
			NAU8825_POWERUP_INTEGR_R | NAU8825_POWERUP_INTEGR_L | NAU8825_POWERUP_DRV_IN_R | NAU8825_POWERUP_DRV_IN_L | NAU8825_POWERUP_HP_DRV_R | NAU8825_POWERUP_HP_DRV_L,
			NAU8825_POWERUP_INTEGR_R | NAU8825_POWERUP_INTEGR_L | NAU8825_POWERUP_DRV_IN_R | NAU8825_POWERUP_DRV_IN_L | NAU8825_POWERUP_HP_DRV_R | NAU8825_POWERUP_HP_DRV_L);

		nau8825_reg_update(pDevice, NAU8825_REG_POWER_UP_CONTROL,
			(1 << 14),
			(1 << 14)); //Frontend PGA Enable

		nau8825_reg_update(pDevice, NAU8825_REG_CHARGE_PUMP,
			NAU8825_JAMNODCLOW | NAU8825_CHANRGE_PUMP_EN,
			NAU8825_JAMNODCLOW | NAU8825_CHANRGE_PUMP_EN);

		nau8825_reg_update(pDevice, NAU8825_REG_CHARGE_PUMP,
			NAU8825_POWER_DOWN_DACR | NAU8825_POWER_DOWN_DACL,
			0);
	}

	/* Leaving HPOL/R grounded after jack insert by default. They will be
	 * ungrounded as part of the widget power up sequence at the beginning
	 * of playback to reduce pop.
	 */
	return type;
}

BOOLEAN OnInterruptIsr(
	WDFINTERRUPT Interrupt,
	ULONG MessageID) {
	UNREFERENCED_PARAMETER(MessageID);

	WDFDEVICE Device = WdfInterruptGetDevice(Interrupt);
	PNAU8825_CONTEXT pDevice = GetDeviceContext(Device);

	if (!pDevice->DevicePoweredOn)
		return true;

	int active_irq, clear_irq = 0, event = 0, event_mask = 0;
	NTSTATUS status = nau8825_reg_read(pDevice, NAU8825_REG_IRQ_STATUS, &active_irq);
	if (!NT_SUCCESS(status)) {
		DbgPrint("failed to read irq status\n");
		return false;
	}

	if ((active_irq & NAU8825_JACK_EJECTION_IRQ_MASK) ==
		NAU8825_JACK_EJECTION_DETECTED) {

		nau8825_eject_jack(pDevice);

		CsAudioSpecialKeyReport report;
		report.ReportID = REPORTID_SPECKEYS;
		report.ControlCode = CONTROL_CODE_JACK_TYPE;
		report.ControlValue = 0;

		size_t bytesWritten;
		Nau8825ProcessVendorReport(pDevice, &report, sizeof(report), &bytesWritten);

		clear_irq = NAU8825_JACK_EJECTION_IRQ_MASK;
	}
	else if (active_irq & NAU8825_KEY_SHORT_PRESS_IRQ) {
		clear_irq = NAU8825_KEY_SHORT_PRESS_IRQ;
	}
	else if (active_irq & NAU8825_KEY_RELEASE_IRQ) {
		clear_irq = NAU8825_KEY_RELEASE_IRQ;

		if (nau8825_is_jack_inserted(pDevice)) {
			int key_status;

			nau8825_reg_read(pDevice, NAU8825_REG_INT_CLR_KEY_STATUS,
				&key_status);

			Nau8825MediaReport report;
			report.ReportID = REPORTID_MEDIA;
			report.ControlCode = nau8825_button_decode(key_status >> 8);

			size_t bytesWritten;
			Nau8825ProcessVendorReport(pDevice, &report, sizeof(report), &bytesWritten);
		}
	}
	else if (active_irq & NAU8825_HEADSET_COMPLETION_IRQ) {
		if (nau8825_is_jack_inserted(pDevice)) {
			CsAudioSpecialKeyReport report;
			report.ReportID = REPORTID_SPECKEYS;
			report.ControlCode = CONTROL_CODE_JACK_TYPE;
			report.ControlValue = nau8825_jack_insert(pDevice);

			size_t bytesWritten;
			Nau8825ProcessVendorReport(pDevice, &report, sizeof(report), &bytesWritten);
		}
		clear_irq = NAU8825_HEADSET_COMPLETION_IRQ;
	}
	else if (active_irq & NAU8825_IMPEDANCE_MEAS_IRQ) {
		clear_irq = NAU8825_IMPEDANCE_MEAS_IRQ;
	}
	else if ((active_irq & NAU8825_JACK_INSERTION_IRQ_MASK) ==
		NAU8825_JACK_INSERTION_DETECTED) {
		/* One more step to check GPIO status directly. Thus, the
		 * driver can confirm the real insertion interruption because
		 * the intrruption at manual mode has bypassed debounce
		 * circuit which can get rid of unstable status.
		 */
		if (nau8825_is_jack_inserted(pDevice)) {
			/* Turn off insertion interruption at manual mode */
			nau8825_reg_update(pDevice,
				NAU8825_REG_INTERRUPT_DIS_CTRL,
				NAU8825_IRQ_INSERT_DIS,
				NAU8825_IRQ_INSERT_DIS);
			nau8825_reg_update(pDevice, NAU8825_REG_INTERRUPT_MASK,
				NAU8825_IRQ_INSERT_EN, NAU8825_IRQ_INSERT_EN);
			/* Enable interruption for jack type detection at audo
			 * mode which can detect microphone and jack type.
			 */
			nau8825_setup_auto_irq(pDevice);
		}
	}

	if (!clear_irq)
		clear_irq = active_irq;
	/* clears the rightmost interruption */
	nau8825_reg_write(pDevice, NAU8825_REG_INT_CLR_KEY_STATUS, clear_irq);

	return true;
}

NTSTATUS
Nau8825EvtDeviceAdd(
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
	PNAU8825_CONTEXT               devContext;

	UNREFERENCED_PARAMETER(Driver);

	PAGED_CODE();

	Nau8825Print(DEBUG_LEVEL_INFO, DBG_PNP,
		"Nau8825EvtDeviceAdd called\n");

	//
	// Tell framework this is a filter driver. Filter drivers by default are  
	// not power policy owners. This works well for this driver because
	// HIDclass driver is the power policy owner for HID minidrivers.
	//

	WdfFdoInitSetFilter(DeviceInit);

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
	// Setup the device context
	//

	WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&attributes, NAU8825_CONTEXT);

	//
	// Create a framework device object.This call will in turn create
	// a WDM device object, attach to the lower stack, and set the
	// appropriate flags and attributes.
	//

	status = WdfDeviceCreate(&DeviceInit, &attributes, &device);

	if (!NT_SUCCESS(status))
	{
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfDeviceCreate failed with status code 0x%x\n", status);

		return status;
	}

	{
		WDF_DEVICE_STATE deviceState;
		WDF_DEVICE_STATE_INIT(&deviceState);

		deviceState.NotDisableable = WdfFalse;
		WdfDeviceSetDeviceState(device, &deviceState);
	}

	WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&queueConfig, WdfIoQueueDispatchParallel);

	queueConfig.EvtIoInternalDeviceControl = Nau8825EvtInternalDeviceControl;

	status = WdfIoQueueCreate(device,
		&queueConfig,
		WDF_NO_OBJECT_ATTRIBUTES,
		&queue
	);

	if (!NT_SUCCESS(status))
	{
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfIoQueueCreate failed 0x%x\n", status);

		return status;
	}

	//
	// Create manual I/O queue to take care of hid report read requests
	//

	devContext = GetDeviceContext(device);

	devContext->FxDevice = device;

	WDF_IO_QUEUE_CONFIG_INIT(&queueConfig, WdfIoQueueDispatchManual);

	queueConfig.PowerManaged = WdfFalse;

	status = WdfIoQueueCreate(device,
		&queueConfig,
		WDF_NO_OBJECT_ATTRIBUTES,
		&devContext->ReportQueue
	);

	if (!NT_SUCCESS(status))
	{
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_PNP,
			"WdfIoQueueCreate failed 0x%x\n", status);

		return status;
	}

	//
	// Create an interrupt object for hardware notifications
	//
	WDF_INTERRUPT_CONFIG_INIT(
		&interruptConfig,
		OnInterruptIsr,
		NULL);
	interruptConfig.PassiveHandling = TRUE;

	status = WdfInterruptCreate(
		device,
		&interruptConfig,
		WDF_NO_OBJECT_ATTRIBUTES,
		&devContext->Interrupt);

	if (!NT_SUCCESS(status))
	{
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_PNP,
			"Error creating WDF interrupt object - %!STATUS!",
			status);

		return status;
	}

	return status;
}

VOID
Nau8825EvtInternalDeviceControl(
	IN WDFQUEUE     Queue,
	IN WDFREQUEST   Request,
	IN size_t       OutputBufferLength,
	IN size_t       InputBufferLength,
	IN ULONG        IoControlCode
)
{
	NTSTATUS            status = STATUS_SUCCESS;
	WDFDEVICE           device;
	PNAU8825_CONTEXT     devContext;
	BOOLEAN             completeRequest = TRUE;

	UNREFERENCED_PARAMETER(OutputBufferLength);
	UNREFERENCED_PARAMETER(InputBufferLength);

	device = WdfIoQueueGetDevice(Queue);
	devContext = GetDeviceContext(device);

	Nau8825Print(DEBUG_LEVEL_INFO, DBG_IOCTL,
		"%s, Queue:0x%p, Request:0x%p\n",
		DbgHidInternalIoctlString(IoControlCode),
		Queue,
		Request
	);

	//
	// Please note that HIDCLASS provides the buffer in the Irp->UserBuffer
	// field irrespective of the ioctl buffer type. However, framework is very
	// strict about type checking. You cannot get Irp->UserBuffer by using
	// WdfRequestRetrieveOutputMemory if the ioctl is not a METHOD_NEITHER
	// internal ioctl. So depending on the ioctl code, we will either
	// use retreive function or escape to WDM to get the UserBuffer.
	//

	switch (IoControlCode)
	{

	case IOCTL_HID_GET_DEVICE_DESCRIPTOR:
		//
		// Retrieves the device's HID descriptor.
		//
		status = Nau8825GetHidDescriptor(device, Request);
		break;

	case IOCTL_HID_GET_DEVICE_ATTRIBUTES:
		//
		//Retrieves a device's attributes in a HID_DEVICE_ATTRIBUTES structure.
		//
		status = Nau8825GetDeviceAttributes(Request);
		break;

	case IOCTL_HID_GET_REPORT_DESCRIPTOR:
		//
		//Obtains the report descriptor for the HID device.
		//
		status = Nau8825GetReportDescriptor(device, Request);
		break;

	case IOCTL_HID_GET_STRING:
		//
		// Requests that the HID minidriver retrieve a human-readable string
		// for either the manufacturer ID, the product ID, or the serial number
		// from the string descriptor of the device. The minidriver must send
		// a Get String Descriptor request to the device, in order to retrieve
		// the string descriptor, then it must extract the string at the
		// appropriate index from the string descriptor and return it in the
		// output buffer indicated by the IRP. Before sending the Get String
		// Descriptor request, the minidriver must retrieve the appropriate
		// index for the manufacturer ID, the product ID or the serial number
		// from the device extension of a top level collection associated with
		// the device.
		//
		status = Nau8825GetString(Request);
		break;

	case IOCTL_HID_WRITE_REPORT:
	case IOCTL_HID_SET_OUTPUT_REPORT:
		//
		//Transmits a class driver-supplied report to the device.
		//
		status = Nau8825WriteReport(devContext, Request);
		break;

	case IOCTL_HID_READ_REPORT:
	case IOCTL_HID_GET_INPUT_REPORT:
		//
		// Returns a report from the device into a class driver-supplied buffer.
		// 
		status = Nau8825ReadReport(devContext, Request, &completeRequest);
		break;

	case IOCTL_HID_SET_FEATURE:
		//
		// This sends a HID class feature report to a top-level collection of
		// a HID class device.
		//
		status = Nau8825SetFeature(devContext, Request, &completeRequest);
		break;

	case IOCTL_HID_GET_FEATURE:
		//
		// returns a feature report associated with a top-level collection
		status = Nau8825GetFeature(devContext, Request, &completeRequest);
		break;

	case IOCTL_HID_ACTIVATE_DEVICE:
		//
		// Makes the device ready for I/O operations.
		//
	case IOCTL_HID_DEACTIVATE_DEVICE:
		//
		// Causes the device to cease operations and terminate all outstanding
		// I/O requests.
		//
	default:
		status = STATUS_NOT_SUPPORTED;
		break;
	}

	if (completeRequest)
	{
		WdfRequestComplete(Request, status);

		Nau8825Print(DEBUG_LEVEL_INFO, DBG_IOCTL,
			"%s completed, Queue:0x%p, Request:0x%p\n",
			DbgHidInternalIoctlString(IoControlCode),
			Queue,
			Request
		);
	}
	else
	{
		Nau8825Print(DEBUG_LEVEL_INFO, DBG_IOCTL,
			"%s deferred, Queue:0x%p, Request:0x%p\n",
			DbgHidInternalIoctlString(IoControlCode),
			Queue,
			Request
		);
	}

	return;
}

NTSTATUS
Nau8825GetHidDescriptor(
	IN WDFDEVICE Device,
	IN WDFREQUEST Request
)
{
	NTSTATUS            status = STATUS_SUCCESS;
	size_t              bytesToCopy = 0;
	WDFMEMORY           memory;

	UNREFERENCED_PARAMETER(Device);

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825GetHidDescriptor Entry\n");

	//
	// This IOCTL is METHOD_NEITHER so WdfRequestRetrieveOutputMemory
	// will correctly retrieve buffer from Irp->UserBuffer. 
	// Remember that HIDCLASS provides the buffer in the Irp->UserBuffer
	// field irrespective of the ioctl buffer type. However, framework is very
	// strict about type checking. You cannot get Irp->UserBuffer by using
	// WdfRequestRetrieveOutputMemory if the ioctl is not a METHOD_NEITHER
	// internal ioctl.
	//
	status = WdfRequestRetrieveOutputMemory(Request, &memory);

	if (!NT_SUCCESS(status))
	{
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"WdfRequestRetrieveOutputMemory failed 0x%x\n", status);

		return status;
	}

	//
	// Use hardcoded "HID Descriptor" 
	//
	bytesToCopy = DefaultHidDescriptor.bLength;

	if (bytesToCopy == 0)
	{
		status = STATUS_INVALID_DEVICE_STATE;

		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"DefaultHidDescriptor is zero, 0x%x\n", status);

		return status;
	}

	status = WdfMemoryCopyFromBuffer(memory,
		0, // Offset
		(PVOID)&DefaultHidDescriptor,
		bytesToCopy);

	if (!NT_SUCCESS(status))
	{
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"WdfMemoryCopyFromBuffer failed 0x%x\n", status);

		return status;
	}

	//
	// Report how many bytes were copied
	//
	WdfRequestSetInformation(Request, bytesToCopy);

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825GetHidDescriptor Exit = 0x%x\n", status);

	return status;
}

NTSTATUS
Nau8825GetReportDescriptor(
	IN WDFDEVICE Device,
	IN WDFREQUEST Request
)
{
	NTSTATUS            status = STATUS_SUCCESS;
	ULONG_PTR           bytesToCopy;
	WDFMEMORY           memory;

	PNAU8825_CONTEXT devContext = GetDeviceContext(Device);

	UNREFERENCED_PARAMETER(Device);

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825GetReportDescriptor Entry\n");

	//
	// This IOCTL is METHOD_NEITHER so WdfRequestRetrieveOutputMemory
	// will correctly retrieve buffer from Irp->UserBuffer. 
	// Remember that HIDCLASS provides the buffer in the Irp->UserBuffer
	// field irrespective of the ioctl buffer type. However, framework is very
	// strict about type checking. You cannot get Irp->UserBuffer by using
	// WdfRequestRetrieveOutputMemory if the ioctl is not a METHOD_NEITHER
	// internal ioctl.
	//
	status = WdfRequestRetrieveOutputMemory(Request, &memory);
	if (!NT_SUCCESS(status))
	{
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"WdfRequestRetrieveOutputMemory failed 0x%x\n", status);

		return status;
	}

	//
	// Use hardcoded Report descriptor
	//
	bytesToCopy = DefaultHidDescriptor.DescriptorList[0].wReportLength;

	if (bytesToCopy == 0)
	{
		status = STATUS_INVALID_DEVICE_STATE;

		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"DefaultHidDescriptor's reportLength is zero, 0x%x\n", status);

		return status;
	}

	status = WdfMemoryCopyFromBuffer(memory,
		0,
		(PVOID)DefaultReportDescriptor,
		bytesToCopy);
	if (!NT_SUCCESS(status))
	{
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"WdfMemoryCopyFromBuffer failed 0x%x\n", status);

		return status;
	}

	//
	// Report how many bytes were copied
	//
	WdfRequestSetInformation(Request, bytesToCopy);

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825GetReportDescriptor Exit = 0x%x\n", status);

	return status;
}


NTSTATUS
Nau8825GetDeviceAttributes(
	IN WDFREQUEST Request
)
{
	NTSTATUS                 status = STATUS_SUCCESS;
	PHID_DEVICE_ATTRIBUTES   deviceAttributes = NULL;

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825GetDeviceAttributes Entry\n");

	//
	// This IOCTL is METHOD_NEITHER so WdfRequestRetrieveOutputMemory
	// will correctly retrieve buffer from Irp->UserBuffer. 
	// Remember that HIDCLASS provides the buffer in the Irp->UserBuffer
	// field irrespective of the ioctl buffer type. However, framework is very
	// strict about type checking. You cannot get Irp->UserBuffer by using
	// WdfRequestRetrieveOutputMemory if the ioctl is not a METHOD_NEITHER
	// internal ioctl.
	//
	status = WdfRequestRetrieveOutputBuffer(Request,
		sizeof(HID_DEVICE_ATTRIBUTES),
		(PVOID*)&deviceAttributes,
		NULL);
	if (!NT_SUCCESS(status))
	{
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"WdfRequestRetrieveOutputBuffer failed 0x%x\n", status);

		return status;
	}

	//
	// Set USB device descriptor
	//

	deviceAttributes->Size = sizeof(HID_DEVICE_ATTRIBUTES);
	deviceAttributes->VendorID = NAU8825_VID;
	deviceAttributes->ProductID = NAU8825_PID;
	deviceAttributes->VersionNumber = NAU8825_VERSION;

	//
	// Report how many bytes were copied
	//
	WdfRequestSetInformation(Request, sizeof(HID_DEVICE_ATTRIBUTES));

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825GetDeviceAttributes Exit = 0x%x\n", status);

	return status;
}

NTSTATUS
Nau8825GetString(
	IN WDFREQUEST Request
)
{

	NTSTATUS status = STATUS_SUCCESS;
	PWSTR pwstrID;
	size_t lenID;
	WDF_REQUEST_PARAMETERS params;
	void* pStringBuffer = NULL;

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825GetString Entry\n");

	WDF_REQUEST_PARAMETERS_INIT(&params);
	WdfRequestGetParameters(Request, &params);

	switch ((ULONG_PTR)params.Parameters.DeviceIoControl.Type3InputBuffer & 0xFFFF)
	{
	case HID_STRING_ID_IMANUFACTURER:
		pwstrID = L"Nau8825.\0";
		break;

	case HID_STRING_ID_IPRODUCT:
		pwstrID = L"MaxTouch Touch Screen\0";
		break;

	case HID_STRING_ID_ISERIALNUMBER:
		pwstrID = L"123123123\0";
		break;

	default:
		pwstrID = NULL;
		break;
	}

	lenID = pwstrID ? wcslen(pwstrID) * sizeof(WCHAR) + sizeof(UNICODE_NULL) : 0;

	if (pwstrID == NULL)
	{

		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"Nau8825GetString Invalid request type\n");

		status = STATUS_INVALID_PARAMETER;

		return status;
	}

	status = WdfRequestRetrieveOutputBuffer(Request,
		lenID,
		&pStringBuffer,
		&lenID);

	if (!NT_SUCCESS(status))
	{

		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"Nau8825GetString WdfRequestRetrieveOutputBuffer failed Status 0x%x\n", status);

		return status;
	}

	RtlCopyMemory(pStringBuffer, pwstrID, lenID);

	WdfRequestSetInformation(Request, lenID);

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825GetString Exit = 0x%x\n", status);

	return status;
}

NTSTATUS
Nau8825WriteReport(
	IN PNAU8825_CONTEXT DevContext,
	IN WDFREQUEST Request
)
{
	NTSTATUS status = STATUS_SUCCESS;
	WDF_REQUEST_PARAMETERS params;
	PHID_XFER_PACKET transferPacket = NULL;
	size_t bytesWritten = 0;

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825WriteReport Entry\n");

	WDF_REQUEST_PARAMETERS_INIT(&params);
	WdfRequestGetParameters(Request, &params);

	if (params.Parameters.DeviceIoControl.InputBufferLength < sizeof(HID_XFER_PACKET))
	{
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"Nau8825WriteReport Xfer packet too small\n");

		status = STATUS_BUFFER_TOO_SMALL;
	}
	else
	{

		transferPacket = (PHID_XFER_PACKET)WdfRequestWdmGetIrp(Request)->UserBuffer;

		if (transferPacket == NULL)
		{
			Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
				"Nau8825WriteReport No xfer packet\n");

			status = STATUS_INVALID_DEVICE_REQUEST;
		}
		else
		{
			//
			// switch on the report id
			//

			switch (transferPacket->reportId)
			{
			default:

				Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
					"Nau8825WriteReport Unhandled report type %d\n", transferPacket->reportId);

				status = STATUS_INVALID_PARAMETER;

				break;
			}
		}
	}

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825WriteReport Exit = 0x%x\n", status);

	return status;

}

NTSTATUS
Nau8825ProcessVendorReport(
	IN PNAU8825_CONTEXT DevContext,
	IN PVOID ReportBuffer,
	IN ULONG ReportBufferLen,
	OUT size_t* BytesWritten
)
{
	NTSTATUS status = STATUS_SUCCESS;
	WDFREQUEST reqRead;
	PVOID pReadReport = NULL;
	size_t bytesReturned = 0;

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825ProcessVendorReport Entry\n");

	status = WdfIoQueueRetrieveNextRequest(DevContext->ReportQueue,
		&reqRead);

	if (NT_SUCCESS(status))
	{
		status = WdfRequestRetrieveOutputBuffer(reqRead,
			ReportBufferLen,
			&pReadReport,
			&bytesReturned);

		if (NT_SUCCESS(status))
		{
			//
			// Copy ReportBuffer into read request
			//

			if (bytesReturned > ReportBufferLen)
			{
				bytesReturned = ReportBufferLen;
			}

			RtlCopyMemory(pReadReport,
				ReportBuffer,
				bytesReturned);

			//
			// Complete read with the number of bytes returned as info
			//

			WdfRequestCompleteWithInformation(reqRead,
				status,
				bytesReturned);

			Nau8825Print(DEBUG_LEVEL_INFO, DBG_IOCTL,
				"Nau8825ProcessVendorReport %d bytes returned\n", bytesReturned);

			//
			// Return the number of bytes written for the write request completion
			//

			*BytesWritten = bytesReturned;

			Nau8825Print(DEBUG_LEVEL_INFO, DBG_IOCTL,
				"%s completed, Queue:0x%p, Request:0x%p\n",
				DbgHidInternalIoctlString(IOCTL_HID_READ_REPORT),
				DevContext->ReportQueue,
				reqRead);
		}
		else
		{
			Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
				"WdfRequestRetrieveOutputBuffer failed Status 0x%x\n", status);
		}
	}
	else
	{
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"WdfIoQueueRetrieveNextRequest failed Status 0x%x\n", status);
	}

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825ProcessVendorReport Exit = 0x%x\n", status);

	return status;
}

NTSTATUS
Nau8825ReadReport(
	IN PNAU8825_CONTEXT DevContext,
	IN WDFREQUEST Request,
	OUT BOOLEAN* CompleteRequest
)
{
	NTSTATUS status = STATUS_SUCCESS;

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825ReadReport Entry\n");

	//
	// Forward this read request to our manual queue
	// (in other words, we are going to defer this request
	// until we have a corresponding write request to
	// match it with)
	//

	status = WdfRequestForwardToIoQueue(Request, DevContext->ReportQueue);

	if (!NT_SUCCESS(status))
	{
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"WdfRequestForwardToIoQueue failed Status 0x%x\n", status);
	}
	else
	{
		*CompleteRequest = FALSE;
	}

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825ReadReport Exit = 0x%x\n", status);

	return status;
}

NTSTATUS
Nau8825SetFeature(
	IN PNAU8825_CONTEXT DevContext,
	IN WDFREQUEST Request,
	OUT BOOLEAN* CompleteRequest
)
{
	NTSTATUS status = STATUS_SUCCESS;
	WDF_REQUEST_PARAMETERS params;
	PHID_XFER_PACKET transferPacket = NULL;

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825SetFeature Entry\n");

	WDF_REQUEST_PARAMETERS_INIT(&params);
	WdfRequestGetParameters(Request, &params);

	if (params.Parameters.DeviceIoControl.InputBufferLength < sizeof(HID_XFER_PACKET))
	{
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"Nau8825SetFeature Xfer packet too small\n");

		status = STATUS_BUFFER_TOO_SMALL;
	}
	else
	{

		transferPacket = (PHID_XFER_PACKET)WdfRequestWdmGetIrp(Request)->UserBuffer;

		if (transferPacket == NULL)
		{
			Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
				"Nau8825WriteReport No xfer packet\n");

			status = STATUS_INVALID_DEVICE_REQUEST;
		}
		else
		{
			//
			// switch on the report id
			//

			switch (transferPacket->reportId)
			{
			default:

				Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
					"Nau8825SetFeature Unhandled report type %d\n", transferPacket->reportId);

				status = STATUS_INVALID_PARAMETER;

				break;
			}
		}
	}

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825SetFeature Exit = 0x%x\n", status);

	return status;
}

NTSTATUS
Nau8825GetFeature(
	IN PNAU8825_CONTEXT DevContext,
	IN WDFREQUEST Request,
	OUT BOOLEAN* CompleteRequest
)
{
	NTSTATUS status = STATUS_SUCCESS;
	WDF_REQUEST_PARAMETERS params;
	PHID_XFER_PACKET transferPacket = NULL;

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825GetFeature Entry\n");

	WDF_REQUEST_PARAMETERS_INIT(&params);
	WdfRequestGetParameters(Request, &params);

	if (params.Parameters.DeviceIoControl.OutputBufferLength < sizeof(HID_XFER_PACKET))
	{
		Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
			"Nau8825GetFeature Xfer packet too small\n");

		status = STATUS_BUFFER_TOO_SMALL;
	}
	else
	{

		transferPacket = (PHID_XFER_PACKET)WdfRequestWdmGetIrp(Request)->UserBuffer;

		if (transferPacket == NULL)
		{
			Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
				"Nau8825GetFeature No xfer packet\n");

			status = STATUS_INVALID_DEVICE_REQUEST;
		}
		else
		{
			//
			// switch on the report id
			//

			switch (transferPacket->reportId)
			{
			default:

				Nau8825Print(DEBUG_LEVEL_ERROR, DBG_IOCTL,
					"Nau8825GetFeature Unhandled report type %d\n", transferPacket->reportId);

				status = STATUS_INVALID_PARAMETER;

				break;
			}
		}
	}

	Nau8825Print(DEBUG_LEVEL_VERBOSE, DBG_IOCTL,
		"Nau8825GetFeature Exit = 0x%x\n", status);

	return status;
}

PCHAR
DbgHidInternalIoctlString(
	IN ULONG IoControlCode
)
{
	switch (IoControlCode)
	{
	case IOCTL_HID_GET_DEVICE_DESCRIPTOR:
		return "IOCTL_HID_GET_DEVICE_DESCRIPTOR";
	case IOCTL_HID_GET_REPORT_DESCRIPTOR:
		return "IOCTL_HID_GET_REPORT_DESCRIPTOR";
	case IOCTL_HID_READ_REPORT:
		return "IOCTL_HID_READ_REPORT";
	case IOCTL_HID_GET_DEVICE_ATTRIBUTES:
		return "IOCTL_HID_GET_DEVICE_ATTRIBUTES";
	case IOCTL_HID_WRITE_REPORT:
		return "IOCTL_HID_WRITE_REPORT";
	case IOCTL_HID_SET_FEATURE:
		return "IOCTL_HID_SET_FEATURE";
	case IOCTL_HID_GET_FEATURE:
		return "IOCTL_HID_GET_FEATURE";
	case IOCTL_HID_GET_STRING:
		return "IOCTL_HID_GET_STRING";
	case IOCTL_HID_ACTIVATE_DEVICE:
		return "IOCTL_HID_ACTIVATE_DEVICE";
	case IOCTL_HID_DEACTIVATE_DEVICE:
		return "IOCTL_HID_DEACTIVATE_DEVICE";
	case IOCTL_HID_SEND_IDLE_NOTIFICATION_REQUEST:
		return "IOCTL_HID_SEND_IDLE_NOTIFICATION_REQUEST";
	case IOCTL_HID_SET_OUTPUT_REPORT:
		return "IOCTL_HID_SET_OUTPUT_REPORT";
	case IOCTL_HID_GET_INPUT_REPORT:
		return "IOCTL_HID_GET_INPUT_REPORT";
	default:
		return "Unknown IOCTL";
	}
}