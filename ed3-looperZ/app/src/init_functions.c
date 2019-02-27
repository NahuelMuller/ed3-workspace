/*====== LIBRARIES ======*/
#include	"main.h"			// Header

/*====== FUNCTION DECLARATIONS ======*/

void init_Hardware(void){

	printf("Inicializando Hardware...\n");

	// EDU-CIAA
	SystemCoreClockUpdate();	// Update system core clock rate
	Board_Init();				// Inicia LEDs y Pulsadores

	// CAMBIO EL CLOCK BASE DEL ADC Y EL DAC (para obtener 8kHz de sampling)
	Chip_Clock_SetDivider((CHIP_CGU_IDIV_T) CLK_IDIV_D, (CHIP_CGU_CLKIN_T) CLKIN_MAINPLL, 16);	// CLKIN_IDIVD = CLKIN_MAINPLL / 16 = 204MHz / 4 = 12.75MHz
	Chip_Clock_SetBaseClock(CLK_BASE_APB3, CLKIN_IDIVD, FALSE, FALSE);							// Seteo el CLK base del ADC y el DAC como la salida del divisor

	// ADC
	printf("Inicializando el ADC...\n");
	Chip_SCU_ADC_Channel_Config(0, ADC_CH1);								// Inicia el pin (analog function) ADC0_CH1
	ADC_CLOCK_SETUP_T ADC_Config;											// Estructura default de config
	Chip_ADC_Init(LPC_ADC0, &ADC_Config);									// Inicializacion (400kHz - 10bits)
	Chip_ADC_SetSampleRate(LPC_ADC0, &ADC_Config, 8000);					// Tasa sampleo: 8kHz
	Chip_ADC_SetResolution(LPC_ADC0, &ADC_Config, ADC_10BITS);				// Resolucion: 10 bits
	Chip_ADC_SetBurstCmd(LPC_ADC0, ENABLE);									// Enable burst => Repeated conversions
	Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH1, ENABLE);						// Enable channel
	Chip_ADC_Int_SetChannelCmd(LPC_ADC0, ADC_CH1, ENABLE);					// Enable interrupt
	Chip_ADC_SetStartMode(LPC_ADC0, ADC_NO_START, ADC_TRIGGERMODE_RISING);	// ADC_NO_START: Must be set for Burst mode
	Chip_ADC_Int_SetGlobalCmd(LPC_ADC0, DISABLE);							// Disable global interrupt

	uint32_t clk_adc = Chip_Clock_GetRate(CLK_APB3_ADC0);
	uint32_t clk_div = (LPC_ADC0->CR >> 8) & 0xFF;
	float fs_adc = (float) clk_adc / ((float) (clk_div + 1) * 11);			// 11 = Resolucion + 1
	printf("Clock del ADC0_CH1: %d\n", clk_adc);
	printf("Frecuencia de sampleo del ADC0_CH1: %f\n", fs_adc);

	// DAC
	printf("Inicializando el DAC...\n");
	Chip_SCU_DAC_Analog_Config();							// Inicia el pin (analog function)
	Chip_DAC_Init(LPC_DAC);									// Inicializacion
	Chip_DAC_ConfigDAConverterControl(LPC_DAC,				// Control register
										DAC_DBLBUF_ENA |	// Enable double-buffering
										DAC_CNT_ENA |		// Enable timer out counter
										DAC_DMA_ENA);		// Enable DMA access
	Chip_DAC_SetBias(LPC_DAC, 1);							// Set maximum update rate for DAC (400kHz)
	Chip_DAC_SetDMATimeOut(LPC_DAC, 1595);					// Reload value for the DAC interrupt/DMA timer (12.75MHz / 1595 = 7993.730469Hz)

	uint32_t clk_dac = Chip_Clock_GetRate(CLK_APB3_DAC);
	float fs_dac = (float) clk_dac / (float) 1595;
	printf("Clock del DAC: %d\n", clk_dac);
	printf("Frecuencia de sampleo del DAC: %f\n", fs_dac);

	// DMA_ADC
	printf("Inicializando el DMA_ADC...\n");
	ADC_DMA_Channel = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_ADC_0);	// Get a free GPDMA channel

	Chip_GPDMA_PrepareDescriptor(LPC_GPDMA,								// Prepare a single DMA descriptor
									&ADC_DMA_Descriptor_0,					// DMA Descriptor to be initialized
									GPDMA_CONN_ADC_0,						// Source
									(uint32_t) &ADC_BUF_0,					// Destination
									SAMPLES_BUFFER,							// Size: The number of DMA transfers
									GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,	// Type of DMA controller (Flow control)
									&ADC_DMA_Descriptor_1);					// Pointer to next descriptor

	Chip_GPDMA_PrepareDescriptor(LPC_GPDMA,								// Prepare a single DMA descriptor
									&ADC_DMA_Descriptor_1,					// DMA Descriptor to be initialized
									GPDMA_CONN_ADC_0,						// Source
									(uint32_t) &ADC_BUF_1,					// Destination
									SAMPLES_BUFFER,							// Size: The number of DMA transfers
									GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,	// Type of DMA controller (Flow control)
									&ADC_DMA_Descriptor_0);					// Pointer to next descriptor

	ADC_DMA_Descriptor_0.ctrl = (ADC_DMA_Descriptor_0.ctrl & 0x7F000FFF) |			// Mask para modificar los bits 12 a 23 y 31
								GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1) |			// Source burst size = 1 (default = 4)
								GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1) |			// Destination burst size = 1 (default = 4)
								GPDMA_DMACCxControl_SWidth(GPDMA_WIDTH_HALFWORD) |	// Source transfer width = 16 bit (default 32 bit)
								GPDMA_DMACCxControl_DWidth(GPDMA_WIDTH_HALFWORD) |	// Destination transfer width = 16 bit (default 32 bit)
								GPDMA_DMACCxControl_I;								// The terminal count interrupt enabled (entre cada salto de LLI)

	ADC_DMA_Descriptor_1.ctrl = ADC_DMA_Descriptor_0.ctrl;

	ADC_DMA_Descriptor_0.src = (uint32_t) &(LPC_ADC0->DR[ADC_CH1]);		// La config default del descriptor
	ADC_DMA_Descriptor_1.src = (uint32_t) &(LPC_ADC0->DR[ADC_CH1]);		// apunta a &LPC_ADC0->GDR

	// Bug hotfix de la API del DMA de las librerias LPCOpen (ver https://community.nxp.com/thread/422356)
	DMA_TransferDescriptor_t ADC_DMA_Descriptor_temp;
	ADC_DMA_Descriptor_temp.ctrl = ADC_DMA_Descriptor_0.ctrl;
	ADC_DMA_Descriptor_temp.lli = (uint32_t) &ADC_DMA_Descriptor_0;
	ADC_DMA_Descriptor_temp.src = GPDMA_CONN_ADC_0;
	ADC_DMA_Descriptor_temp.dst = ADC_DMA_Descriptor_0.dst;
	Chip_GPDMA_SGTransfer(LPC_GPDMA, ADC_DMA_Channel, &ADC_DMA_Descriptor_temp, GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA);

	// DMA_DAC
	printf("Inicializando el DMA_DAC...\n");
	DAC_DMA_Channel = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_DAC);		// Get a free GPDMA channel

	Chip_GPDMA_PrepareDescriptor(LPC_GPDMA,								// Prepare a single DMA descriptor
									&DAC_DMA_Descriptor_0,					// DMA Descriptor to be initialized
									(uint32_t) &DAC_BUF_0,					// Source
									GPDMA_CONN_DAC,							// Destination
									SAMPLES_BUFFER,							// Size: The number of DMA transfers
									GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,	// Type of DMA controller (Flow control)
									&DAC_DMA_Descriptor_1);					// Pointer to next descriptor

	Chip_GPDMA_PrepareDescriptor(LPC_GPDMA,								// Prepare a single DMA descriptor
									&DAC_DMA_Descriptor_1,					// DMA Descriptor to be initialized
									(uint32_t) &DAC_BUF_1,					// Source
									GPDMA_CONN_DAC,							// Destination
									SAMPLES_BUFFER,							// Size: The number of DMA transfers
									GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,	// Type of DMA controller (Flow control)
									&DAC_DMA_Descriptor_0);					// Pointer to next descriptor

	DAC_DMA_Descriptor_0.ctrl =	(DAC_DMA_Descriptor_0.ctrl & 0x7F000FFF) |			// Mask para modificar los bits 12 a 23 y 31
								GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1) |			// Source burst size = 1 (default = 4)
								GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1) |			// Destination burst size = 1 (default = 1)
								GPDMA_DMACCxControl_SWidth(GPDMA_WIDTH_HALFWORD) |	// Source transfer width = 16 bit (default 32 bit)
								GPDMA_DMACCxControl_DWidth(GPDMA_WIDTH_HALFWORD) |	// Destination transfer width = 16 bit (default 32 bit)
								GPDMA_DMACCxControl_I;								// The terminal count interrupt enabled (entre cada salto de LLI)

	DAC_DMA_Descriptor_1.ctrl = DAC_DMA_Descriptor_0.ctrl;

	// Bug hotfix de la API del DMA de las librerias LPCOpen (ver https://community.nxp.com/thread/422356)
	DMA_TransferDescriptor_t DAC_DMA_Descriptor_temp;
	DAC_DMA_Descriptor_temp.ctrl = DAC_DMA_Descriptor_0.ctrl;
	DAC_DMA_Descriptor_temp.lli = (uint32_t) &DAC_DMA_Descriptor_0;
	DAC_DMA_Descriptor_temp.src = DAC_DMA_Descriptor_0.src;
	DAC_DMA_Descriptor_temp.dst = GPDMA_CONN_DAC;
	Chip_GPDMA_SGTransfer(LPC_GPDMA, DAC_DMA_Channel, &DAC_DMA_Descriptor_temp, GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA);

	// SPI: SSP1
	printf("Inicializando el SSP en SPI mode...\n");
	Chip_SCU_PinMuxSet(0xF, 4, (SCU_MODE_PULLUP | SCU_MODE_FUNC0));												// SSP1_SCK
	Chip_SCU_PinMuxSet(0x1, 3, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC5));		// SSP1_MISO
	Chip_SCU_PinMuxSet(0x1, 4, (SCU_MODE_PULLUP | SCU_MODE_FUNC5));												// SSP1_MOSI
	Chip_SCU_PinMuxSet(0x4, 10, (SCU_MODE_PULLUP | SCU_MODE_FUNC4));	// GPIO5[14] usado como CS
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 5, 14);					// GPIO5[14] seteado como salida
	Chip_SSP_Init(LPC_SSP1);		// Inicializacion SSP1
	Chip_SSP_Enable(LPC_SSP1);		// SSP1 enable

	printf("Systems up and running\n");

}


void init_Interrupts(void){

	Chip_SCU_GPIOIntPinSel(0, 0, 4);		// GPIO Interrupt Pin Select
	Chip_SCU_GPIOIntPinSel(1, 0, 8);
	Chip_SCU_GPIOIntPinSel(2, 0, 9);
	Chip_SCU_GPIOIntPinSel(3, 1, 9);

	Chip_PININT_Init(LPC_GPIO_PIN_INT);				// Inicializa el bloque PIN_INT
	Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT,	// Interrupcion por flanco
								PININTCH0 |
								PININTCH1 |
								PININTCH2 |
								PININTCH3);
	Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT,		// Interrupcion por flanco ascendente
								PININTCH0 |
								PININTCH1 |
								PININTCH2 |
								PININTCH3) ;

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH0);	// Clear PIN interrupts
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH1);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH2);
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH3);
	NVIC_ClearPendingIRQ(PIN_INT0_IRQn);	// Clear PIN interrupts (NVIC)
	NVIC_ClearPendingIRQ(PIN_INT1_IRQn);
	NVIC_ClearPendingIRQ(PIN_INT2_IRQn);
	NVIC_ClearPendingIRQ(PIN_INT3_IRQn);
	NVIC_EnableIRQ(PIN_INT0_IRQn);			// Enable PIN interrupts
	NVIC_EnableIRQ(PIN_INT1_IRQn);
	NVIC_EnableIRQ(PIN_INT2_IRQn);
	NVIC_EnableIRQ(PIN_INT3_IRQn);

	NVIC_ClearPendingIRQ(DMA_IRQn);		// Clear DMA interrupt
	NVIC_EnableIRQ(DMA_IRQn);			// Enable DMA interrupt

}
