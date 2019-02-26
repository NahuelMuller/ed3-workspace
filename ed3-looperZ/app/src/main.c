/*****************************************************************************
*   Materia:      Electronica Digital III - 2018 - UNSAM ECyT                *
*   Estudiantes:  Fonseca, Maximiliano - Muller, Nahuel                      *
*   Proyecto:     Loopera en FreeRTOS basada en el kit EDU-CIAA              *
*****************************************************************************/

/*====== LIBRARIES ======*/
#include	"main.h"			// Header

/*====== VARIABLES ======*/
uint16_t					ADC_BUF_0[SAMPLES_BUFFER],	// Buffers DAC y ADC para el DMA
							ADC_BUF_1[SAMPLES_BUFFER],
							DAC_BUF_0[SAMPLES_BUFFER],
							DAC_BUF_1[SAMPLES_BUFFER];
uint8_t						ADC_DMA_Channel,			// Canales DMA
							DAC_DMA_Channel;
DMA_TransferDescriptor_t	ADC_DMA_Descriptor_0,		// Transfer Descriptor structs
							ADC_DMA_Descriptor_1,
							DAC_DMA_Descriptor_0,
							DAC_DMA_Descriptor_1;
SemaphoreHandle_t			ADC_BUF_0_libre,			// Semaforos que libera el DMA IRQ
							ADC_BUF_1_libre,			// Indican que buffer se termino de leer (ADC) o escribir (DAC)
							DAC_BUF_0_libre,
							DAC_BUF_1_libre,
							queue_intermedia1_ready,	// La tarea ADC lleno una queue
							queue_intermedia2_ready;	// La tarea MEM lleno una queue
QueueHandle_t				queue_intermedia1,
							queue_intermedia2;
StreamBufferHandle_t		ADC_Output,					// Pipes
							DAC_Input;

/*====== PRIVATE FUNCTIONS ======*/

static void create_Tareas(void){

	// Blinky LED 5 para dar signos de vida
	xTaskCreate(vLED_Status, "vLED_Status", configMINIMAL_STACK_SIZE * 1, NULL, (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL);
	// Lectura del buffer libre del ADC
	xTaskCreate(vADC_Task, "vADC_Task", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL), (TaskHandle_t *) NULL);
	// Escritura del buffer libre del DAC
	xTaskCreate(vDAC_Task, "vDAC_Task", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL), (TaskHandle_t *) NULL);
	// Task para MEMORIA
	xTaskCreate(vMEM_Task, "vMEM_Task", configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 2UL), (TaskHandle_t *) NULL);

}

static void create_Colas(void){

	queue_intermedia1 = xQueueCreate(SAMPLES_BUFFER, sizeof(uint16_t));
	queue_intermedia2 = xQueueCreate(SAMPLES_BUFFER, sizeof(uint16_t));

	if(!queue_intermedia1 || !queue_intermedia2){
		while(1){};			// Fallo creacion
	}

}

static void create_Semaforos(void){

	ADC_BUF_0_libre = xSemaphoreCreateBinary();
	ADC_BUF_1_libre = xSemaphoreCreateBinary();
	DAC_BUF_0_libre = xSemaphoreCreateBinary();
	DAC_BUF_1_libre = xSemaphoreCreateBinary();
	queue_intermedia1_ready = xSemaphoreCreateBinary();
	queue_intermedia2_ready = xSemaphoreCreateBinary();

	if(!ADC_BUF_0_libre || !ADC_BUF_1_libre ||
		!DAC_BUF_0_libre || !DAC_BUF_1_libre ||
		!queue_intermedia1_ready || !queue_intermedia2_ready){
		while(1){};		// Fallo creacion
	}

}

static void create_Pipes(void){

	uint32_t buffer_size = sizeof(uint16_t) * SAMPLES_BUFFER;

	ADC_Output = xStreamBufferCreate(buffer_size, buffer_size);
	DAC_Input = xStreamBufferCreate(buffer_size, buffer_size);

	if(!ADC_Output || !DAC_Input){
		while(1){};			// Fallo creacion
	}
}

/*====== MAIN ======*/

int main(void){

	init_Hardware();			// Inicializacion de hardware
	create_Tareas();			// Creacion de tareas
	create_Colas();				// Creacion de colas
	create_Semaforos();			// Creacion de semaforos
	create_Pipes();				// Creacion de pipes
	init_Interrupts();			// Inician las interrupciones (GPDMA y GPIO)
	vTaskStartScheduler();		// Arranca el Scheduler
	return 1;					// Si llega hasta aca es porque fallo el Scheduler

}

/*====== INTERRUPT HANDLERS ======*/

void DMA_IRQHandler(){    // DMA: Identificar entre DMA_ADC y DMA_DAC y avisar que buffer se puede usar

	if(Chip_GPDMA_IntGetStatus(LPC_GPDMA, GPDMA_STAT_INTTC, ADC_DMA_Channel) == SET){								// Request DMA_ADC
		if(((uint32_t)(LPC_GPDMA->CH[ADC_DMA_Channel].LLI) & 0xFFFFFFFC) == (uint32_t) &ADC_DMA_Descriptor_0){
			xSemaphoreGiveFromISR(ADC_BUF_0_libre, NULL);
		} else if(((uint32_t)(LPC_GPDMA->CH[ADC_DMA_Channel].LLI) & 0xFFFFFFFC) == (uint32_t) &ADC_DMA_Descriptor_1){
			xSemaphoreGiveFromISR(ADC_BUF_1_libre, NULL);
		}
		Chip_GPDMA_ClearIntPending(LPC_GPDMA, GPDMA_STATCLR_INTTC, ADC_DMA_Channel);	// Clear ADC_DMA IRQ
	} else {																										// Request DMA_DAC
		if(((uint32_t)(LPC_GPDMA->CH[DAC_DMA_Channel].LLI) & 0xFFFFFFFC) == (uint32_t) &DAC_DMA_Descriptor_0){
			xSemaphoreGiveFromISR(DAC_BUF_0_libre, NULL);
		} else if(((uint32_t)(LPC_GPDMA->CH[DAC_DMA_Channel].LLI) & 0xFFFFFFFC) == (uint32_t) &DAC_DMA_Descriptor_1){
			xSemaphoreGiveFromISR(DAC_BUF_1_libre, NULL);
		}
		Chip_GPDMA_ClearIntPending(LPC_GPDMA, GPDMA_STATCLR_INTTC, DAC_DMA_Channel);	// Clear DAC_DMA IRQ
	}

}

void GPIO0_IRQHandler(void){	// Asignar funcion

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH0);
	Board_LED_Toggle(1);

}

void GPIO1_IRQHandler(void){	// Asignar funcion

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH1);
	Board_LED_Toggle(2);

}

void GPIO2_IRQHandler(void){	// Asignar funcion

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH2);
	Board_LED_Toggle(3);

}

void GPIO3_IRQHandler(void){	// Asignar funcion

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH3);
	Board_LED_Toggle(4);

}
