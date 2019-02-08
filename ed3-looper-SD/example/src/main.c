/*****************************************************************************
*   Materia:      Electronica Digital III - 2018 - UNSAM ECyT                *
*   Estudiantes:  Fonseca, Maximiliano - Muller, Nahuel                      *
*   Proyecto:     Loopera en FreeRTOS basada en el kit EDU-CIAA              *
*****************************************************************************/

/*
______               _                     _               _  _
| ___ \             (_)                   | |             | |(_)
| |_/ /  ___ __   __ _  ___   __ _  _ __  | |  __ _  ___  | | _  _ __    ___   __ _  ___    ___   ___   _ __
|    /  / _ \\ \ / /| |/ __| / _` || '__| | | / _` |/ __| | || || '_ \  / _ \ / _` |/ __|  / __| / _ \ | '_ \
| |\ \ |  __/ \ V / | |\__ \| (_| || |    | || (_| |\__ \ | || || | | ||  __/| (_| |\__ \ | (__ | (_) || | | |
\_| \_| \___|  \_/  |_||___/ \__,_||_|    |_| \__,_||___/ |_||_||_| |_| \___| \__,_||___/  \___| \___/ |_| |_|


     __    _             ______  _____  _   _  _____  _____   ___  ______              _         __
    / / /\| |/\          | ___ \|  ___|| | | ||_   _|/  ___| / _ \ | ___ \          /\| |/\     / /
   / /  \ ` ' /  ______  | |_/ /| |__  | | | |  | |  \ `--. / /_\ \| |_/ /  ______  \ ` ' /    / /
  / /  |_     _||______| |    / |  __| | | | |  | |   `--. \|  _  ||    /  |______||_     _|  / /
 / /    / , . \          | |\ \ | |___ \ \_/ / _| |_ /\__/ /| | | || |\ \           / , . \  / /
/_/     \/|_|\/          \_| \_|\____/  \___/  \___/ \____/ \_| |_/\_| \_|          \/|_|\/ /_/

*/

/*====== LIBRARIES ======*/

#include "sd_spi.h"
#include "sapi_spi.h"
#include "sapi_gpio.h"
#include "sapi_board.h"
#include "sapi_debugprint.h"

#include "ff.h"

#include		<stdlib.h>

#include		"board.h"			// LPCOpen

#include		"arm_math.h"	// CMSIS DSP

#include		"FreeRTOS.h"	// FreeRTOS
#include		"task.h"
#include		"queue.h"
#include		"semphr.h"

/*====== DEFINITIONS ======*/
#define		SAMPLE_RATE				44100						/*- REVISAR -*/
#define		DAC_DIV				1155		// (204MHz/2552 = 79.9KHz)		(51MHz/1155 = 44.1KHz)
#define		SAMPLES_BUFFER		256						/*- REVISAR -*/
#define		CHANNEL_ADC0			ADC_CH1					/*- REVISAR -*/
#define		FILENAME			"lograbamo.txt"

/*====== VARIABLES ======*/
uint32_t		ADC_BUF_0[SAMPLES_BUFFER], ADC_BUF_1[SAMPLES_BUFFER],					// Buffers DAC y ADC
				DAC_BUF_0[SAMPLES_BUFFER], DAC_BUF_1[SAMPLES_BUFFER];

uint8_t			ADC_DMA_Channel, DAC_DMA_Channel;															// Canales del DMA usados

uint32_t		debug = 0;

static FATFS fs;           // <-- FatFs work area needed for each volume
static FIL fp;             // <-- File object needed for each open file

DMA_TransferDescriptor_t	ADC_DMA_Descriptor_0, ADC_DMA_Descriptor_1,			// Transfer Descriptor structs
													DAC_DMA_Descriptor_0, DAC_DMA_Descriptor_1,
													ADC_DMA_Descriptor_temp, DAC_DMA_Descriptor_temp;

SemaphoreHandle_t			ADC_BUF_0_libre		= NULL,
											ADC_BUF_1_libre				= NULL,
											DAC_BUF_0_libre				= NULL,
											DAC_BUF_1_libre				= NULL,
											Active_TEC1					= NULL,
											Active_TEC2					= NULL,
											queue_intermedia1_ready		= NULL,
											queue_intermedia2_ready		= NULL;

QueueHandle_t					queue_intermedia1	= NULL,
								queue_intermedia2	= NULL;
								//queue_grabo;

/*====== FUNCTIONS ======*/
void init_Hardware(void){	// Inicializacion de hardware

	SystemCoreClockUpdate();	// Update system core clock rate
	Board_Init();	// EDU CIAA: Inicia LEDs y Pulsadores

	// Configuracion ADC
	printf("Inicializando el ADC\n");
	Chip_SCU_ADC_Channel_Config(0, CHANNEL_ADC0);	// Inicia el pin (analog function)
	ADC_CLOCK_SETUP_T ADC_Config;									// Estructura default de config
	Chip_ADC_Init(LPC_ADC0, &ADC_Config);					// Inicializacion (400kHz - 10bits)
	Chip_ADC_SetResolution(LPC_ADC0, &ADC_Config, ADC_10BITS);		// Resolucion
	Chip_ADC_SetBurstCmd(LPC_ADC0, ENABLE);				// Enable burst => Repeated conversions
	Chip_ADC_EnableChannel(LPC_ADC0, CHANNEL_ADC0, ENABLE);						// Enable channel

	Chip_Clock_SetDivider((CHIP_CGU_IDIV_T)CLK_IDIV_A, (CHIP_CGU_CLKIN_T)CLKIN_MAINPLL, 4);		// CLKIN_IDIVA = CLKIN_MAINPLL (204MHz) / 4 = 51MHz
	Chip_Clock_SetBaseClock(CLK_BASE_APB3, CLKIN_IDIVA, FALSE, FALSE);				// Seteo el CLK base del ADC y el DAC como la salida del divisor
	Chip_ADC_SetSampleRate(LPC_ADC0, &ADC_Config, SAMPLE_RATE);		// Tasa sampleo							/*- REVISAR -*/

	Chip_ADC_Int_SetChannelCmd(LPC_ADC0, CHANNEL_ADC0, ENABLE);				// Enable interrupt
	Chip_ADC_SetStartMode(LPC_ADC0, ADC_NO_START, ADC_TRIGGERMODE_RISING);	// ADC_NO_START: Must be set for Burst mode
	Chip_ADC_Int_SetGlobalCmd(LPC_ADC0, DISABLE);	// Disable global interrupt

	// Configuracion DAC
	printf("Inicializando el DAC\n");
	Chip_SCU_DAC_Analog_Config();		// Inicia el pin (analog function)
	Chip_DAC_Init(LPC_DAC);					// Inicializacion
	Chip_DAC_ConfigDAConverterControl(LPC_DAC,					// Control register
																		DAC_DBLBUF_ENA |	// Enable double-buffering
																		DAC_CNT_ENA |			// Enable timer out counter
																		DAC_DMA_ENA);			// Enable DMA access
	Chip_DAC_SetBias(LPC_DAC, 1);		// Set maximum update rate for DAC (400kHz)								/*- REVISAR -*/
	Chip_DAC_SetDMATimeOut(LPC_DAC, DAC_DIV);	// Reload value for the DAC interrupt/DMA timer					/*- REVISAR -*/

	// Configuracion DMA_ADC
	printf("Inicializando el DMA_ADC\n");
	ADC_DMA_Channel = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_ADC_0);		// Get a free GPDMA channel
	Chip_GPDMA_PrepareDescriptor(LPC_GPDMA,															// Prepare a single DMA descriptor
															&ADC_DMA_Descriptor_0,									// DMA Descriptor to be initialized
															GPDMA_CONN_ADC_0,												// Source
															(uint32_t) &ADC_BUF_0,									// Destination
															SAMPLES_BUFFER,													// Size: The number of DMA transfers
															GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,	// Type of DMA controller (Flow control)
															&ADC_DMA_Descriptor_1);									// Pointer to next descriptor
	Chip_GPDMA_PrepareDescriptor(LPC_GPDMA,															// Prepare a single DMA descriptor
															&ADC_DMA_Descriptor_1,									// DMA Descriptor to be initialized
															GPDMA_CONN_ADC_0,												// Source
															(uint32_t) &ADC_BUF_1,									// Destination
															SAMPLES_BUFFER,													// Size: The number of DMA transfers
															GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,	// Type of DMA controller (Flow control)
															&ADC_DMA_Descriptor_0);									// Pointer to next descriptor

	ADC_DMA_Descriptor_0.ctrl &= (GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1) |		// Source burst size = 1
																GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1) |		// Destination burst size = 1
																0xFFFC0FFF);		// Mask para modificar los bits 12 a 17
	ADC_DMA_Descriptor_1.ctrl &= (GPDMA_DMACCxControl_SBSize(GPDMA_BSIZE_1) |		// Source burst size = 1
																GPDMA_DMACCxControl_DBSize(GPDMA_BSIZE_1) |		// Destination burst size = 1
																0xFFFC0FFF);		// Mask para modificar los bits 12 a 17

	ADC_DMA_Descriptor_0.ctrl |= GPDMA_DMACCxControl_I;		// The terminal count interrupt is enabled
	ADC_DMA_Descriptor_1.ctrl |= GPDMA_DMACCxControl_I;		// (interrupcion entre cada salto de LLI)

	ADC_DMA_Descriptor_0.src = (uint32_t) &(LPC_ADC0->DR[CHANNEL_ADC0]);		// La config default del descriptor
	ADC_DMA_Descriptor_1.src = (uint32_t) &(LPC_ADC0->DR[CHANNEL_ADC0]);		// apunta a &LPC_ADCx->GDR

	// Configuracion DMA_DAC
	printf("Inicializando el DMA_DAC\n");
	DAC_DMA_Channel = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_DAC);		// Get a free GPDMA channel
	Chip_GPDMA_PrepareDescriptor(LPC_GPDMA,															// Prepare a single DMA descriptor
															&DAC_DMA_Descriptor_0,									// DMA Descriptor to be initialized
															(uint32_t) &DAC_BUF_0,									// Source
															GPDMA_CONN_DAC,													// Destination
															SAMPLES_BUFFER,													// Size: The number of DMA transfers
															GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,	// Type of DMA controller (Flow control)
															&DAC_DMA_Descriptor_1);									// Pointer to next descriptor
	Chip_GPDMA_PrepareDescriptor(LPC_GPDMA,															// Prepare a single DMA descriptor
															&DAC_DMA_Descriptor_1,									// DMA Descriptor to be initialized
															(uint32_t) &DAC_BUF_1,									// Source
															GPDMA_CONN_DAC,													// Destination
															SAMPLES_BUFFER,													// Size: The number of DMA transfers
															GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA,	// Type of DMA controller (Flow control)
															&DAC_DMA_Descriptor_0);									// Pointer to next descriptor

	DAC_DMA_Descriptor_0.ctrl |= GPDMA_DMACCxControl_I;		// The terminal count interrupt is enabled
	DAC_DMA_Descriptor_1.ctrl |= GPDMA_DMACCxControl_I;		// (interrupcion entre cada salto de LLI)

	// Fix bug de la API del DMA de las librerias LPCOpen (ver https://community.nxp.com/thread/422356)
	ADC_DMA_Descriptor_temp.ctrl = ADC_DMA_Descriptor_0.ctrl;
	ADC_DMA_Descriptor_temp.lli = (uint32_t)&ADC_DMA_Descriptor_0;
	ADC_DMA_Descriptor_temp.src = GPDMA_CONN_ADC_0;
	ADC_DMA_Descriptor_temp.dst = ADC_DMA_Descriptor_0.dst;
	Chip_GPDMA_SGTransfer(LPC_GPDMA,
												ADC_DMA_Channel,
												&ADC_DMA_Descriptor_temp,
												GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA);

	DAC_DMA_Descriptor_temp.ctrl = DAC_DMA_Descriptor_0.ctrl;
	DAC_DMA_Descriptor_temp.lli = (uint32_t)&DAC_DMA_Descriptor_0;
	DAC_DMA_Descriptor_temp.src = DAC_DMA_Descriptor_0.src;
	DAC_DMA_Descriptor_temp.dst = GPDMA_CONN_DAC;
	Chip_GPDMA_SGTransfer(LPC_GPDMA,
												DAC_DMA_Channel,
												&DAC_DMA_Descriptor_temp,
												GPDMA_TRANSFERTYPE_M2P_CONTROLLER_DMA);

	// ALGO DE DEBUGGING
	uint32_t clk = Chip_Clock_GetRate(CLK_APB3_ADC0);
	uint32_t div = (LPC_ADC0->CR >> 8)&0xFF;
	float fs = (float)clk / ( (float)(div+1) * 11 );
	uint32_t clk_dac = Chip_Clock_GetRate(CLK_APB3_DAC);
	float fdac = (float)clk_dac / (float)DAC_DIV;
	printf("Chip_Clock_GetRate(CLK_APB3_ADC0) = %d Hz\n",clk);
	printf("ADC0 div = %d\n",div);
	printf("Frecuencia de muestreo real = ADC0 = %f\n", fs);
	printf("Chip_Clock_GetRate(CLK_APB3_DAC) = %d Hz\n",clk_dac);
	printf("Frecuencia de muestreo real = DAC = %f\n", fdac);

	//IRQ GPIO
	Chip_SCU_PinMuxSet(1, 0,
	(SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC0) );

	Chip_SCU_PinMuxSet(1, 1,
		(SCU_MODE_INBUFF_EN | SCU_MODE_INACT | SCU_MODE_FUNC0) );

	/* Configure GPIO pin as input */
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 4);
	Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, 0, 8);

	/* Configure interrupt channel for the GPIO pin in SysCon block */
	Chip_SCU_GPIOIntPinSel(0, 0, 4);
	Chip_SCU_GPIOIntPinSel(1, 0, 8);

	/* Configure channel interrupt as edge sensitive and falling edge interrupt */

	//Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(0));//edge
	//Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(0));//rising


	//Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(1));//edge
	//Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(1));//rising

	LPC_GPIO_PIN_INT->IENR = 0x03;//set rising edge for pinint0 y 1

	/* Enable Group GPIO interrupt 0 */
	NVIC_ClearPendingIRQ(PIN_INT0_IRQn);
	NVIC_EnableIRQ(PIN_INT0_IRQn);

	NVIC_ClearPendingIRQ(PIN_INT1_IRQn);
	NVIC_EnableIRQ(PIN_INT1_IRQn);

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));


}

/*====== SEMAPHORES ======*/
void create_Semaforos(void){

	ADC_BUF_0_libre = xSemaphoreCreateBinary();
	ADC_BUF_1_libre = xSemaphoreCreateBinary();
	DAC_BUF_0_libre = xSemaphoreCreateBinary();
	DAC_BUF_1_libre = xSemaphoreCreateBinary();
	Active_TEC1		= xSemaphoreCreateBinary();
	Active_TEC2		= xSemaphoreCreateBinary();
	queue_intermedia1_ready = xSemaphoreCreateBinary();
	queue_intermedia2_ready = xSemaphoreCreateBinary();

	if(!ADC_BUF_0_libre || !ADC_BUF_1_libre ||
			!DAC_BUF_0_libre || !DAC_BUF_1_libre || !Active_TEC1 || !Active_TEC2
			|| !queue_intermedia1_ready || !queue_intermedia2_ready){
		while(1);			// Fallo creacion
	}

}

/*====== QUEUES ======*/
void create_Colas(void){

	queue_intermedia1 = xQueueCreate(SAMPLES_BUFFER, sizeof(uint32_t));

	if(!queue_intermedia1){
		while(1);			// Fallo creacion
	}

	queue_intermedia2 = xQueueCreate(SAMPLES_BUFFER, sizeof(uint32_t));

	if(!queue_intermedia2){
		while(1);			// Fallo creacion
	}


}

/*====== TASKS ======*/
void vADC_Task(void *pvParameters){

	uint16_t jota;	// Actores de reparto
	uint32_t ka;

	while(1){
		if(xSemaphoreTake(ADC_BUF_0_libre, (TickType_t) 0) == pdPASS){					/*- REVISAR -*/
			for(jota = 0; jota < SAMPLES_BUFFER; jota++){
				ka = ADC_BUF_0[jota];
				//ka = DAC_VALUE(768);
				xQueueSendToBack(queue_intermedia1, &ka, (TickType_t) 1);
			}
			xSemaphoreGive(queue_intermedia1_ready);
		}
		//vTaskDelay(1 / portTICK_RATE_MS);	/*- REVISAR -*/
		else if(xSemaphoreTake(ADC_BUF_1_libre, (TickType_t) 0) == pdPASS){					/*- REVISAR -*/
			for(jota = 0; jota < SAMPLES_BUFFER; jota++){
				ka = ADC_BUF_1[jota];
				//ka =DAC_VALUE(256);
				xQueueSendToBack(queue_intermedia1, &ka, (TickType_t) 1);
			}
			xSemaphoreGive(queue_intermedia1_ready);
		}
		vTaskDelay(1 / portTICK_RATE_MS);																													/*- REVISAR -*/
	}

}

void vDAC_Task(void *pvParameters){

	uint16_t jota;	// Actores de reparto
	uint32_t ka;

	while(1){
		if(xSemaphoreTake(queue_intermedia2_ready, (TickType_t) 0) == pdPASS){
			if(xSemaphoreTake(DAC_BUF_0_libre, (TickType_t) 0) == pdPASS){					/*- REVISAR -*/
				for(jota = 0; jota < SAMPLES_BUFFER; jota++){
					xQueueReceive(queue_intermedia2, &ka, (TickType_t) 1); // Successfully received items are removed from the queue.
					DAC_BUF_0[jota] = ka;
					//DAC_BUF_0[jota] = DAC_VALUE(768);
				}
			}
			//vTaskDelay(10);																													/*- REVISAR -*/
			else if(xSemaphoreTake(DAC_BUF_1_libre, (TickType_t) 0) == pdPASS){					/*- REVISAR -*/

				for(jota = 0; jota < SAMPLES_BUFFER; jota++){
					xQueueReceive(queue_intermedia2, &ka, (TickType_t) 1);
					DAC_BUF_1[jota] = ka;
					//DAC_BUF_1[jota] = DAC_VALUE(256);
				}
			}
		}
		vTaskDelay(1 / portTICK_RATE_MS);																													/*- REVISAR -*/
	}

}

void vMEM_Task(void *pvParameters){

	uint16_t jota;	// Actores de reparto
	uint32_t ka;
	uint16_t grabo=0;
	uint32_t current_bloque = 0;
	uint32_t max_bloque = 0;
	uint32_t vectorcito[SAMPLES_BUFFER];

	for(jota = 0; jota < SAMPLES_BUFFER; jota++){
		vectorcito[jota] = 0;
	}


	if( f_mount( &fs, "", 0 ) != FR_OK ){
	// If this fails, it means that the function could not register a file
	// system object. Check whether the SD card is correctly connected.
		printf( "Error intentando montar la tarjeta SD.\n" );
		while(1){}
	}

	if( f_open( &fp, FILENAME, FA_CREATE_ALWAYS | FA_WRITE | FA_READ ) == FR_OK ){
		printf( "OK crear/abrir el archivo\n" );
	} else{
		printf( "Error intentando crear/abrir el archivo\n" );
		while(1){}
	}


	while(1){

		if( xSemaphoreTake(Active_TEC1,( TickType_t ) 0) == pdPASS ){
			if(grabo == 0){
				grabo = 1;
				Board_LED_Set(4, TRUE);
				current_bloque = 0;
			} else {
				if(max_bloque == 0){
					max_bloque = current_bloque - 1;
					current_bloque = 0;
				}
				grabo = 0;
				Board_LED_Set(4, FALSE);
			}
		}
		if( xSemaphoreTake(queue_intermedia1_ready,( TickType_t ) 1) == pdPASS ){

			if(max_bloque != 0){
				// HACER: LEER SD en current_bloque y ponerlo en un vectorcito
				// vectorcito = SD[current_bloque]
			}

			for(jota = 0; jota < SAMPLES_BUFFER; jota++){
				xQueueReceive(queue_intermedia1, &ka, (TickType_t) 0);

				ka = ka + vectorcito[jota];

				xQueueSendToBack(queue_intermedia2, &ka, (TickType_t) 1);

				xSemaphoreGive(queue_intermedia2_ready);

				vectorcito[jota] = grabo * ka;
			}

			if(grabo){
				// HACER: ESCRIBIR lo de vectorcito en SD en current bloque
				// SD[current_bloque] = vectorcito
			}

			current_bloque++;
			if(current_bloque == max_bloque && max_bloque != 0 ){
				current_bloque  = 0;
				grabo = 0;
			}
		}
		//vTaskDelay(1 / portTICK_RATE_MS);
	}

}


void vLED_Task(void *pvParameters){

	while(1){
		Board_LED_Toggle(5);
		vTaskDelay(1000 / portTICK_RATE_MS);
	}

}

void vBorradoTask (void *pvParameters){

	while(1){
		if(xSemaphoreTake(Active_TEC2,( TickType_t ) 0) == pdTRUE){ // pulsador fue apretado
			Board_LED_Set(0, TRUE);

		f_close(&fp);

		if( f_open( &fp, FILENAME, FA_CREATE_ALWAYS | FA_WRITE | FA_READ ) == FR_OK ){
				printf( "OK crear/abrir el archivo\n" );
			} else{
				printf( "Error intentando crear/abrir el archivo\n" );
				while(1){}
			}

			vTaskDelay(1000);
			Board_LED_Set(0, FALSE);
		}
		vTaskDelay(10);
	}

}


/*====== TASKS DECLARATIONS ======*/
void create_Tareas(void){

	// Lectura del buffer libre del ADC
	xTaskCreate(vADC_Task, "vADC_Task", configMINIMAL_STACK_SIZE,
							NULL, (tskIDLE_PRIORITY + 3UL), (TaskHandle_t *) NULL);

	// Escritura del buffer libre del DAC
	xTaskCreate(vDAC_Task, "vDAC_Task", configMINIMAL_STACK_SIZE,
							NULL, (tskIDLE_PRIORITY + 3UL), (TaskHandle_t *) NULL);

	// Blink para saber si sigue vivo
	xTaskCreate(vLED_Task, "vLED_Task", configMINIMAL_STACK_SIZE,
							NULL, (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL);

	// Task para TEC2
	xTaskCreate(vBorradoTask, "vBorradoTask", configMINIMAL_STACK_SIZE,
							NULL, (tskIDLE_PRIORITY + 2UL), (TaskHandle_t *) NULL);

	// Task para MEMORIA
	xTaskCreate(vMEM_Task, "vMEM_Task", configMINIMAL_STACK_SIZE,
							NULL, (tskIDLE_PRIORITY + 3UL), (TaskHandle_t *) NULL);

}

/*====== INTERRUPTS ======*/
void DMA_IRQHandler(){    // DMA: Identificar entre DMA_ADC y DMA_DAC y avisar que buffer se puede usar

	if(Chip_GPDMA_IntGetStatus(LPC_GPDMA, GPDMA_STAT_INTTC, ADC_DMA_Channel) == SET){		// Request DMA_ADC
		if(((uint32_t)(LPC_GPDMA->CH[ADC_DMA_Channel].LLI) & 0xFFFFFFFC) == (uint32_t) &ADC_DMA_Descriptor_0){
			xSemaphoreGiveFromISR(ADC_BUF_0_libre, NULL);
		} else if(((uint32_t)(LPC_GPDMA->CH[ADC_DMA_Channel].LLI) & 0xFFFFFFFC) == (uint32_t) &ADC_DMA_Descriptor_1){
			xSemaphoreGiveFromISR(ADC_BUF_1_libre, NULL);
		}
		Chip_GPDMA_ClearIntPending(LPC_GPDMA, GPDMA_STATCLR_INTTC, ADC_DMA_Channel);	// Clear ADC_DMA IRQ
	} else {																																						// Request DMA_DAC
		if(((uint32_t)(LPC_GPDMA->CH[DAC_DMA_Channel].LLI) & 0xFFFFFFFC) == (uint32_t) &DAC_DMA_Descriptor_0){
			xSemaphoreGiveFromISR(DAC_BUF_0_libre, NULL);
		} else if(((uint32_t)(LPC_GPDMA->CH[DAC_DMA_Channel].LLI) & 0xFFFFFFFC) == (uint32_t) &DAC_DMA_Descriptor_1){
			xSemaphoreGiveFromISR(DAC_BUF_1_libre, NULL);
		}
		Chip_GPDMA_ClearIntPending(LPC_GPDMA, GPDMA_STATCLR_INTTC, DAC_DMA_Channel);	// Clear DAC_DMA IRQ
	}

}

void GPIO0_IRQHandler(void)//TEC1 fue apretado
{
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
	xSemaphoreGiveFromISR(Active_TEC1, NULL);
}

void GPIO1_IRQHandler(void)//TEC2 fue apretado
{
	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));
	xSemaphoreGiveFromISR(Active_TEC2, NULL);
}

/*====== MAIN ======*/
int main(void){

	init_Hardware();									// Inicializacion de hardware
	create_Semaforos();								// Creacion de semaforos
	create_Colas();										// Creacion de colas
	create_Tareas();									// Creacion de tareas

	NVIC_EnableIRQ(DMA_IRQn);					// Enable DMA IRQ
	NVIC_ClearPendingIRQ(DMA_IRQn);		// Clear DMA IRQ

	vTaskStartScheduler();						// Arranca el Scheduler
	return 1;													// Si llega hasta aca es porque fallo el Scheduler

}



/*====== THIS IS THE END ======*/

/*
 * This old vSemaphoreCreateBinary() macro is now deprecated in favour of the
 * xSemaphoreCreateBinary() function.  Note that binary semaphores created using
 * the vSemaphoreCreateBinary() macro are created in a state such that the
 * first call to 'take' the semaphore would pass, whereas binary semaphores
 * created using xSemaphoreCreateBinary() are created in a state such that the
 * the semaphore must first be 'given' before it can be 'taken'.
 */



/*====== THIS IS THE REAL END ======*/
