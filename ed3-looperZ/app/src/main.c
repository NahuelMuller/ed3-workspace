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
							ADC_BUF_1_libre,			// Indican cual buffer se termino de leer (ADC) o escribir (DAC)
							DAC_BUF_0_libre,
							DAC_BUF_1_libre,
							queue_from_ADC_ready,	// La tarea ADC lleno una queue
							queue_to_DAC_ready,	// La tarea MEM lleno una queue
							finalizar_ejecucion;		// Aviso para graceful shutdown
QueueHandle_t				queue_from_ADC,
							queue_to_DAC;
TaskHandle_t				xHandle_FIN_Task;			// Handler de la tarea vFIN_Task

/*====== PRIVATE FUNCTIONS ======*/

static void create_Tareas(void){

	// Blinky LED 5 para dar signos de vida
	xTaskCreate(vLED_Task, "vLED_Task", configMINIMAL_STACK_SIZE * 1, NULL, (tskIDLE_PRIORITY + 1UL), NULL);
	// Lectura del buffer libre del ADC
	xTaskCreate(vADC_Task, "vADC_Task", configMINIMAL_STACK_SIZE * 1, NULL, (tskIDLE_PRIORITY + 2UL), NULL);
	// Escritura del buffer libre del DAC
	xTaskCreate(vDAC_Task, "vDAC_Task", configMINIMAL_STACK_SIZE * 1, NULL, (tskIDLE_PRIORITY + 2UL), NULL);
	// Task para MEMORIA
	xTaskCreate(vMEM_Task, "vMEM_Task", configMINIMAL_STACK_SIZE * 1, NULL, (tskIDLE_PRIORITY + 2UL), NULL);
	// Task para finalizar la ejecucion
	xTaskCreate(vFIN_Task, "vFIN_Task", configMINIMAL_STACK_SIZE * 1, NULL, (tskIDLE_PRIORITY + 1UL), &xHandle_FIN_Task);

}

static void create_Colas(void){

	queue_from_ADC = xQueueCreate(SAMPLES_BUFFER, sizeof(uint16_t));
	queue_to_DAC = xQueueCreate(SAMPLES_BUFFER, sizeof(uint16_t));

	if(!queue_from_ADC || !queue_to_DAC){
		while(1){};			// Fallo creacion
	}

}

static void create_Semaforos(void){

	ADC_BUF_0_libre = xSemaphoreCreateBinary();
	ADC_BUF_1_libre = xSemaphoreCreateBinary();
	DAC_BUF_0_libre = xSemaphoreCreateBinary();
	DAC_BUF_1_libre = xSemaphoreCreateBinary();
	queue_from_ADC_ready = xSemaphoreCreateBinary();
	queue_to_DAC_ready = xSemaphoreCreateBinary();
	finalizar_ejecucion = xSemaphoreCreateBinary();

	if(!ADC_BUF_0_libre || !ADC_BUF_1_libre ||
		!DAC_BUF_0_libre || !DAC_BUF_1_libre ||
		!queue_from_ADC_ready || !queue_to_DAC_ready ||
		!finalizar_ejecucion){
		while(1){};		// Fallo creacion
	}

}

static void create_Pipes(void){

}

/*====== MAIN ======*/

int main(void){

	init_Hardware();			// Inicializacion de hardware
	create_Tareas();			// Creacion de tareas
	create_Colas();				// Creacion de colas
	create_Semaforos();			// Creacion de semaforos
	create_Pipes();				// Creacion de pipes
	init_SD_Card();				// Inicializacion de la memoria externa
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

}

void GPIO1_IRQHandler(void){	// Asignar funcion

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH1);

}

void GPIO2_IRQHandler(void){	// Asignar funcion

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH2);

}

void GPIO3_IRQHandler(void){	// Graceful shutdown

	Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH3);
	xSemaphoreGiveFromISR(finalizar_ejecucion, NULL);

}
