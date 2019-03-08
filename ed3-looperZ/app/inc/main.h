/*****************************************************************************
*   Materia:      Electronica Digital III - 2018 - UNSAM ECyT                *
*   Estudiantes:  Fonseca, Maximiliano - Muller, Nahuel                      *
*   Proyecto:     Loopera en FreeRTOS basada en el kit EDU-CIAA              *
*****************************************************************************/

/*====== LIBRARIES ======*/
#include	"board.h"			// LPCOpen
#include	"arm_math.h"		// CMSIS DSP
#include	"FreeRTOS.h"		// FreeRTOS
#include	"task.h"				// Tareas
#include	"queue.h"				// Colas
#include	"semphr.h"				// Semaforos
#include	"stream_buffer.h"		// Pipes
#include	"ff.h"				// FAT FS

/*====== DEFINITIONS ======*/
#define		SAMPLES_BUFFER		255						// Calculado para que cada buffer tenga 510 bytes
#define		FILENAME			"looper.dat"

/*====== VARIABLES ======*/
extern	uint16_t					ADC_BUF_0[SAMPLES_BUFFER],	// Buffers DAC y ADC para el DMA
									ADC_BUF_1[SAMPLES_BUFFER],
									DAC_BUF_0[SAMPLES_BUFFER],
									DAC_BUF_1[SAMPLES_BUFFER];
extern	uint8_t						ADC_DMA_Channel,			// Canales DMA
									DAC_DMA_Channel;
extern	DMA_TransferDescriptor_t	ADC_DMA_Descriptor_0,		// Transfer Descriptor structs
									ADC_DMA_Descriptor_1,
									DAC_DMA_Descriptor_0,
									DAC_DMA_Descriptor_1;
extern	SemaphoreHandle_t			ADC_BUF_0_libre,			// Semaforos que libera el DMA IRQ
									ADC_BUF_1_libre,			// Indican que buffer se termino de leer (ADC) o escribir (DAC)
									DAC_BUF_0_libre,
									DAC_BUF_1_libre,
									queue_from_ADC_ready,		// La tarea ADC lleno una queue
									queue_to_DAC_ready,			// La tarea MEM lleno una queue
									finalizar_ejecucion,		// Aviso para graceful shutdown
									toggle_record,				// Iniciar o detener grabacion
									erase_record;				// Borrar grabacion
extern	QueueHandle_t				queue_from_ADC,
									queue_to_DAC;
extern	TaskHandle_t				xHandle_FIN_Task;			// Handler de la tarea vFIN_Task
extern	FATFS						filesystem;			// <-- FatFs work area needed for each volume
extern	FIL							file;				// <-- File object needed for each open file


/*====== FUNCTION PROTOTYPES ======*/
void init_Hardware(void);
void init_SD_Card(void);
void init_Interrupts(void);
void disk_timerproc(void);
void vLED_Task(void *);
void vDSK_Task(void *);
void vADC_Task(void *);
void vDAC_Task(void *);
void vMEM_Task(void *);
void vFIN_Task(void *);
void dsp_filter(uint16_t *, uint16_t *);

