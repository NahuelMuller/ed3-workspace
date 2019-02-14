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

/*====== DEFINITIONS ======*/
#define		SAMPLE_RATE			8000
#define		SAMPLES_BUFFER		256

/*====== VARIABLES ======*/
extern	uint16_t	ADC_BUF_0[SAMPLES_BUFFER],				// Buffers DAC y ADC para el DMA
					ADC_BUF_1[SAMPLES_BUFFER],
					DAC_BUF_0[SAMPLES_BUFFER],
					DAC_BUF_1[SAMPLES_BUFFER];
extern	uint8_t		ADC_DMA_Channel,						// Canales DMA
					DAC_DMA_Channel;
extern	DMA_TransferDescriptor_t	ADC_DMA_Descriptor_0,	// Transfer Descriptor structs
									ADC_DMA_Descriptor_1,
									DAC_DMA_Descriptor_0,
									DAC_DMA_Descriptor_1;
extern	SemaphoreHandle_t	ADC_BUF_0_libre,
							ADC_BUF_1_libre,
							DAC_BUF_0_libre,
							DAC_BUF_1_libre,
							queue_intermedia1_ready,
							queue_intermedia2_ready;
extern	QueueHandle_t		queue_intermedia1,
							queue_intermedia2;

/*====== FUNCTION PROTOTYPES ======*/
void init_Hardware(void);
void init_Interrupts(void);
void vLED_Status(void *);
void vADC_Task(void *);
void vDAC_Task(void *);
void vMEM_Task(void *);
