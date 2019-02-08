/*
 * @brief FreeRTOS ADC DMA example
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"

#include "arm_math.h" // CMSIS DSP

#include "FreeRTOS.h"

#include "task.h"
#include "queue.h"
#include "semphr.h"

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

#define FREC_MUESTREO		32 // Hz
#define MUESTRAS_BUFFER 	128
#define ADC0_CH				ADC_CH1


/*===========================================================================*/
/*===========================================================================*/

QueueHandle_t xQueue_ADC0_lli;
QueueHandle_t xQueue_ADC0;
SemaphoreHandle_t xSemaphore_ADC0_procesar_muestras;


/*===========================================================================*/
/*===========================================================================*/

void ADC0_setup(void) {

	Chip_SCU_ADC_Channel_Config(0, ADC0_CH);

	static ADC_CLOCK_SETUP_T ADCSetup;

	Chip_ADC_Init(LPC_ADC0, &ADCSetup);

	Chip_ADC_EnableChannel(LPC_ADC0, ADC0_CH, ENABLE);

	/*
	 * El ADC divide la frecuencia de APB3, segun:
	 *     - la cantidad de bits de resolucion (para 10 bits, se requieren 11 ciclos)
	 *     - divisor elegido (el divisor es de 8 bits, o sea el valor maximo es 255)
	 * La formula para calcular la frecuencia de muestreo es:
	 * Fs = Frec_ABP3 / ( (div + 1) * 11 ) ///// el 11 es por los 11 ciclos de la resolucion de 10 bits
	 *
	 * Cuidado con la frecuencia minima de muestreo.
	 * Por ejemplo, usando un reloj de 204 MHz, la frecuencia minima de muestreo es
	 * Fs_min = 204 Mhz / ((255 + 1) * 11)) = 72443.19 Hz
	 *
	 * Para lograr menores frecuencia, hay que cambiar el reloj que toma APB3.
	 * Por ejemplo, se puede tomar el cristal externo, de 12 MHz.
	 * Por ejemplo, tambien se tomar el cristal externo pasado por un divisor de hasta 256,
	 * o sea, Frec_ABP3 = 12 Mhz / 256 = 46875 Hz
	 * En ese ejemplo, la frecuencia minima de muestreo seria (con resolucion de 10 bits):
	 * Fs_min = 46875 Hz / ((255+1)*11) = 16.65 Hz
	 *
	 *  */


	/* Para verificar la frecuencia de los relojes */

	// 12 Mhz
	printf("Chip_Clock_GetClockInputHz ((CHIP_CGU_CLKIN_T) CLKIN_CRYSTAL) = %d Hz\n",
			Chip_Clock_GetClockInputHz ((CHIP_CGU_CLKIN_T) CLKIN_CRYSTAL));

	// 204 Mhz
	printf("Chip_Clock_GetClockInputHz ((CHIP_CGU_CLKIN_T) CLKIN_MAINPLL) = %d Hz\n",
			Chip_Clock_GetClockInputHz ((CHIP_CGU_CLKIN_T) CLKIN_MAINPLL));


	/* Elijo frecuencia div E = 12 Mhz / 256 = 46875 Hz */
	Chip_Clock_SetDivider((CHIP_CGU_IDIV_T)CLK_IDIV_E, (CHIP_CGU_CLKIN_T)CLKIN_CRYSTAL, 256);

	SystemCoreClockUpdate(); // ¿es necesario llamar a esta funcion despues de actualizar relojes?

	/* Verifico que frecuencia de div E sea 12 Mhz / 256 = 46875 Hz */
	printf("Chip_Clock_GetClockInputHz ((CHIP_CGU_CLKIN_T) CLKIN_IDIVE) = %d Hz\n",
			Chip_Clock_GetClockInputHz ((CHIP_CGU_CLKIN_T) CLKIN_IDIVE));


	Chip_Clock_SetBaseClock(CLK_BASE_APB3, CLKIN_IDIVE, false, false); // ¿que diferencia hay con autoblocken = true?

	Chip_ADC_SetSampleRate(LPC_ADC0, &ADCSetup, FREC_MUESTREO);

	Chip_ADC_SetResolution(LPC_ADC0, &ADCSetup, ADC_10BITS);

	Chip_ADC_SetBurstCmd(LPC_ADC0, ENABLE);

	Chip_ADC_SetStartMode(LPC_ADC0, ADC_NO_START, ADC_TRIGGERMODE_RISING);

	uint32_t clk = Chip_Clock_GetRate(CLK_APB3_ADC0);
	uint32_t div = (LPC_ADC0->CR >> 8)&0xFF;
	float fs = (float)clk / ( (float)(div+1) * 11 );

	printf("Chip_Clock_GetRate(CLK_APB3_ADC0) = %d Hz\n",clk);
	printf("ADC0 div = %d\n",div);
	printf("Frecuencia de muestreo real = ADC0 = %f\n", fs);
	printf("\n");


	Chip_ADC_Int_SetChannelCmd(LPC_ADC0, ADC0_CH, ENABLE);

}


/*===========================================================================*/
/*===========================================================================*/

uint32_t adc0_buffer_0[MUESTRAS_BUFFER];
uint32_t adc0_buffer_1[MUESTRAS_BUFFER];

DMA_TransferDescriptor_t ADC0_DMADescriptor_0, ADC0_DMADescriptor_1, temp_ADC0_DMADescriptor;

uint8_t ADC0_dmaChannelNum;

void configurar_DMA_ADC0(void) {

    int idx;

    for (idx=0; idx<MUESTRAS_BUFFER; idx++) {
        adc0_buffer_0[idx] = 0;
        adc0_buffer_1[idx] = 0;
    }


	ADC0_dmaChannelNum = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_ADC_0);


	Chip_GPDMA_PrepareDescriptor(LPC_GPDMA,
										&ADC0_DMADescriptor_0,
										GPDMA_CONN_ADC_0,
										(uint32_t) &adc0_buffer_0,
										MUESTRAS_BUFFER,
										GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
										&ADC0_DMADescriptor_1);


	Chip_GPDMA_PrepareDescriptor(LPC_GPDMA,
										&ADC0_DMADescriptor_1,
										GPDMA_CONN_ADC_0,
										(uint32_t) &adc0_buffer_1,
										MUESTRAS_BUFFER,
										GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA,
										&ADC0_DMADescriptor_0);

	/* Chip_GPDMA_PrepareDescriptor() por default solamente programa interrución para el último elemento de la lista.
	 * (ver linea 687 de Chip_GPDMA_PrepareDescriptor() en gpdma_18xx_43xx.c)
	 * Pero yo quiero que haya una interrupción cuando termine cada elemento de la lista.
	 * Asi que configuro para que haya una interrupcion cuando termine cada elemento de la lista:
	 */

	ADC0_DMADescriptor_0.ctrl |= GPDMA_DMACCxControl_I; // 0x80000000; // bit 31 = 1 --> Terminal count interrupt enable bit.
	ADC0_DMADescriptor_1.ctrl |= GPDMA_DMACCxControl_I; // 0x80000000; // bit 31 = 1 --> Terminal count interrupt enable bit.

	/* Por default, Chip_GPDMA_PrepareDescriptor() establece
	 * el Source burst size (bits 14:12) y Destination burst size (bits 17:15) en 4 en vez de 1.
	 * (ver lineas 68 y 69 de GPDMA_LUTPerBurst[] en de gpdma_18xx_43xx.c)
	 * Pero yo quiero el que burst size del ADC sea igual que el del DAC (que es 1)
	 * Asi que configuro los burst size del ADC en 1:
	 */

	ADC0_DMADescriptor_0.ctrl &= (GPDMA_DMACCxControl_SBSize(0) | GPDMA_DMACCxControl_DBSize(0) | 0xFFFC0FFF);
	ADC0_DMADescriptor_1.ctrl &= (GPDMA_DMACCxControl_SBSize(0) | GPDMA_DMACCxControl_DBSize(0) | 0xFFFC0FFF);


	/* Por default, Chip_GPDMA_PrepareDescriptor() utiliza &LPC_ADCx->GDR como direccion del resultado de la conversion
	 * (ver lineas 136 y 137 de GPDMA_LUTPerAddr[] en gpdma_18xx_43xx.c)
	 * En el manual de LPC, aclaran que cuando uno trabaja en modo BURST (que es este caso),
	 * NO debe leerse el resultado de &LPC_ADCx->GDR, sino de los DR de los correspondientes canales.
	 */

	ADC0_DMADescriptor_0.src = (uint32_t) &(LPC_ADC0->DR[ADC0_CH]);
	ADC0_DMADescriptor_1.src = (uint32_t) &(LPC_ADC0->DR[ADC0_CH]);

	/* Hay un bug en en las funciones Chip_GPDMA_PrepareDescriptor() y Chip_GPDMA_SGTransfer() de LPC Open 2.12
	 * Ver más info acá: https://community.nxp.com/thread/422356
	 * Este es un parche hasta que solucionen el bug:
	 */

	temp_ADC0_DMADescriptor.ctrl = ADC0_DMADescriptor_0.ctrl;
	temp_ADC0_DMADescriptor.lli = (uint32_t)&ADC0_DMADescriptor_0;
	temp_ADC0_DMADescriptor.src = GPDMA_CONN_ADC_0;
	temp_ADC0_DMADescriptor.dst = ADC0_DMADescriptor_0.dst;


	Chip_GPDMA_SGTransfer(LPC_GPDMA,
								 ADC0_dmaChannelNum,
								 &temp_ADC0_DMADescriptor,
								 GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA);

}


/*===========================================================================*/
/*===========================================================================*/

uint8_t lli_dma_adc0;

void DMA_IRQHandler(void) {

	/* Verifico que entro a la interrupcion de DMA */
	Board_LED_Toggle(4);

	// ADC0
    if ( Chip_GPDMA_IntGetStatus(LPC_GPDMA , GPDMA_STAT_INTTC, ADC0_dmaChannelNum) != RESET ) {

    	Chip_GPDMA_ClearIntPending(LPC_GPDMA, GPDMA_STATCLR_INTTC, ADC0_dmaChannelNum);

    	// El DMA acaba de escribir el contenido del puerto del ADC en el vector adc_buffer_0[]

		if ( ( (uint32_t)( LPC_GPDMA->CH[ADC0_dmaChannelNum].LLI ) & 0xFFFFFFFC ) == (uint32_t) &ADC0_DMADescriptor_0 ) {

			lli_dma_adc0 = 0;

			xQueueSendToBackFromISR( xQueue_ADC0_lli, &lli_dma_adc0, NULL );

		// El DMA acaba de escribir el contenido del puerto del ADC en el vector adc_buffer_1[]

	    } else if ( ( (uint32_t)( LPC_GPDMA->CH[ADC0_dmaChannelNum].LLI ) & 0xFFFFFFFC ) == (uint32_t) &ADC0_DMADescriptor_1 ) {

			lli_dma_adc0 = 1;

			xQueueSendToBackFromISR( xQueue_ADC0_lli, &lli_dma_adc0, NULL );

		}

    }

}


/*===========================================================================*/
/*===========================================================================*/

static void vADC0_Task(void *pvParameters) {

	uint8_t numero_lli_adc;
	uint16_t X;
	int idx;

	while (1) {

		if( xQueueReceive( xQueue_ADC0_lli, &( numero_lli_adc ), ( TickType_t ) portMAX_DELAY ) == pdTRUE ) {

			switch(numero_lli_adc) {

				case 0:

					for(idx=0; idx<MUESTRAS_BUFFER; idx++){

						X = (uint16_t) ( ADC_DR_RESULT(adc0_buffer_0[idx]));

						xQueueSendToBack( xQueue_ADC0, (void *) &X, ( TickType_t ) 0 );

					}

					xSemaphoreGive(xSemaphore_ADC0_procesar_muestras);

				case 1:

					for(idx=0; idx<MUESTRAS_BUFFER; idx++){

						X = (uint16_t) ( ADC_DR_RESULT(adc0_buffer_1[idx]) );

						xQueueSendToBack( xQueue_ADC0, (void *) &X, ( TickType_t ) 0 );

					}

					xSemaphoreGive(xSemaphore_ADC0_procesar_muestras);
			}


		}

	}
}


/*===========================================================================*/
/*===========================================================================*/

uint32_t pos_max = 0, pos_min = 0;
q15_t maximo = 0, minimo = 0, media = 0;
q15_t ADC0[MUESTRAS_BUFFER];

void vProcesarMuestras_Task(void *pvParameters) {

	int idx;
	q15_t X0;

	while(1) {

		if( xSemaphoreTake(xSemaphore_ADC0_procesar_muestras,( TickType_t ) portMAX_DELAY) == pdTRUE ) {

			for(idx=0; idx<MUESTRAS_BUFFER; idx++){

				xQueueReceive( xQueue_ADC0, &(X0), ( TickType_t ) 0 );
				ADC0[idx] = (q15_t)X0;

			}

			// Por ejemplo, voy a usar CMSIS DSP para obtener el minimo, maximo, y valor medio del vector

			// Ver documentacion CMSIS DSP:
			//     http://arm-software.github.io/CMSIS_5/DSP/html/index.html
			//     http://www.keil.com/pack/doc/CMSIS/DSP/html/index.html

			arm_max_q15((q15_t*)ADC0, (uint32_t)MUESTRAS_BUFFER, (q15_t*)&maximo, (uint32_t*)&pos_max);
			arm_min_q15((q15_t*)ADC0, (uint32_t)MUESTRAS_BUFFER, (q15_t*)&minimo, (uint32_t*)&pos_min);
			arm_mean_q15((q15_t*)ADC0, (uint32_t)MUESTRAS_BUFFER, (q15_t*)&media);

			printf("max=%d [pos=%d]; min=%d [pos=%d]; media=%d\n",maximo,pos_max,minimo,pos_min,media);

		}
	}
}


/*===========================================================================*/
/*===========================================================================*/

// Esta tarea la pongo para mostrar que el micro sigue vivo
static void vLED_Task(void *pvParameters) {

	while (1) {

		Board_LED_Toggle(0);

		vTaskDelay(1000);
	}
}


/*===========================================================================*/
/*===========================================================================*/

void crear_tareas(void) {

		xQueue_ADC0_lli = xQueueCreate( 1, sizeof( uint8_t ) );

		xQueue_ADC0 = xQueueCreate( MUESTRAS_BUFFER, sizeof( uint16_t ) );

		xSemaphore_ADC0_procesar_muestras = xSemaphoreCreateBinary();


		xTaskCreate(vADC0_Task, "vADC0_Task", configMINIMAL_STACK_SIZE,
				NULL, (tskIDLE_PRIORITY + 3UL), (TaskHandle_t *) NULL);

		xTaskCreate(vProcesarMuestras_Task, "vProcesarMuestras_Task", configMINIMAL_STACK_SIZE,
					NULL, (tskIDLE_PRIORITY + 2UL), (TaskHandle_t *) NULL);

		xTaskCreate(vLED_Task, "vLED_Task", configMINIMAL_STACK_SIZE,
				NULL, (tskIDLE_PRIORITY + 1UL), (TaskHandle_t *) NULL);

}


/*===========================================================================*/
/*===========================================================================*/

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(0, false);
}


/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	main routine for FreeRTOS blinky example
 * @return	Nothing, function should not exit
 */
int main(void)
{
	prvSetupHardware();

	ADC0_setup();

	configurar_DMA_ADC0();

	crear_tareas();

	// Habilitar interrupciones NVIC DMA
	NVIC_EnableIRQ(DMA_IRQn);
	NVIC_ClearPendingIRQ(DMA_IRQn);

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
