/*====== LIBRARIES ======*/
#include	"main.h"			// Header

/*====== FUNCTION DECLARATIONS ======*/

void vLED_Task(void *pvParameters){

	while(1){
		Board_LED_Toggle(5);
		vTaskDelay(1000 / portTICK_RATE_MS);
	}

}

void vDSK_Task(void *pvParameters){

	while(1){
		disk_timerproc();						// Disk timer process
		vTaskDelay(10 / portTICK_RATE_MS);
	}

}

void vADC_Task(void *pvParameters){

	uint16_t jota;	// Actores de reparto
	uint16_t dsp_out[SAMPLES_BUFFER];

	while(1){
		if(xSemaphoreTake(ADC_BUF_0_libre, (TickType_t) 0) == pdPASS){
			dsp_filter(ADC_BUF_0, dsp_out);
			for(jota = 0; jota < SAMPLES_BUFFER; jota++){
				xQueueSendToBack(queue_from_ADC, &dsp_out[jota], (TickType_t) 1);
			}
			xSemaphoreGive(queue_from_ADC_ready);
		}
		else if(xSemaphoreTake(ADC_BUF_1_libre, (TickType_t) 0) == pdPASS){
			dsp_filter(ADC_BUF_1, dsp_out);
			for(jota = 0; jota < SAMPLES_BUFFER; jota++){
				xQueueSendToBack(queue_from_ADC, &dsp_out[jota], (TickType_t) 1);
			}
			xSemaphoreGive(queue_from_ADC_ready);
		}
		vTaskDelay(1 / portTICK_RATE_MS);
	}

}

void vMEM_Task(void *pvParameters){		// EN DESARROLLO!!!!!!!!!!!!!!!

	uint16_t jota, ka;	// Actores de reparto

	while(1){

		if( xSemaphoreTake(queue_from_ADC_ready, (TickType_t) 1) == pdPASS ){
			for(jota = 0; jota < SAMPLES_BUFFER; jota++){
				xQueueReceive(queue_from_ADC, &ka, (TickType_t) 0);			// Successfully received items are removed from the queue.
				xQueueSendToBack(queue_to_DAC, &ka, (TickType_t) 1);
			}
			xSemaphoreGive(queue_to_DAC_ready);
		}
	}

}

void vDAC_Task(void *pvParameters){

	uint16_t jota, ka;	// Actores de reparto

	while(1){
		if(xSemaphoreTake(queue_to_DAC_ready, (TickType_t) 1) == pdPASS){
			if(xSemaphoreTake(DAC_BUF_0_libre, (TickType_t) 0) == pdPASS){
				for(jota = 0; jota < SAMPLES_BUFFER; jota++){
					xQueueReceive(queue_to_DAC, &ka, (TickType_t) 1);
					DAC_BUF_0[jota] = ka;
				}
			}
			else if(xSemaphoreTake(DAC_BUF_1_libre, (TickType_t) 0) == pdPASS){
				for(jota = 0; jota < SAMPLES_BUFFER; jota++){
					xQueueReceive(queue_to_DAC, &ka, (TickType_t) 1);
					DAC_BUF_1[jota] = ka;
				}
			}
		}
	}

}

void vFIN_Task(void *pvParameters){

	uint16_t jota;	// Actores de reparto

	while(1){
		if(xSemaphoreTake(finalizar_ejecucion, (TickType_t) 100) == pdPASS){

			vTaskPrioritySet(xHandle_FIN_Task, (tskIDLE_PRIORITY + 4UL));	// Set maximum priority (No ejecutar nada mas que el apagado)

			NVIC_DisableIRQ(DMA_IRQn);			// Disable interrupts
			NVIC_DisableIRQ(PIN_INT0_IRQn);
			NVIC_DisableIRQ(PIN_INT1_IRQn);
			NVIC_DisableIRQ(PIN_INT2_IRQn);
			NVIC_DisableIRQ(PIN_INT3_IRQn);

			Chip_ADC_DeInit(LPC_ADC0);			// Shutdown ADC
			Chip_DAC_DeInit(LPC_DAC);			// Shutdown DAC

			// (BUG (OPENOCD?): EL GPDMA NO INICIA DE NUEVO EN EL PROXIMO ENCENDIDO, HAY QUE RECONECTAR LA CIAA)
			Chip_GPDMA_DeInit(LPC_GPDMA);		// Shutdown the GPDMA

			f_close(&file);						// File close
			Chip_SSP_DeInit(LPC_SSP1);			// Deinitialise the SSP

			for(jota = 0; jota < 6; jota++){	// LEDs: Apagado
				Board_LED_Set(jota, FALSE);
			}

			vTaskEndScheduler();				// Frenar Scheduler

			printf("Shutdown complete\n");
			while(1){}							// This is the end
		}
	}

}
