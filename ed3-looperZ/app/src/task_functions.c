/*====== LIBRARIES ======*/
#include	"main.h"			// Header

/*====== FUNCTION DECLARATIONS ======*/

void vLED_Status(void *pvParameters) {

	while(1){
		Board_LED_Toggle(5);
		vTaskDelay(1000 / portTICK_RATE_MS);
	}

}

void vADC_Task(void *pvParameters){		// EN DESARROLLO!!!!!!!!!!!!!!!

	uint16_t jota, ka;	// Actores de reparto

	while(1){
		if(xSemaphoreTake(ADC_BUF_0_libre, (TickType_t) 0) == pdPASS){
			for(jota = 0; jota < SAMPLES_BUFFER; jota++){
				//ka = ADC_BUF_0[jota];
				xQueueSendToBack(queue_intermedia1, &ADC_BUF_0[jota], (TickType_t) 1);
			}
			xSemaphoreGive(queue_intermedia1_ready);
		}
		else if(xSemaphoreTake(ADC_BUF_1_libre, (TickType_t) 0) == pdPASS){
			for(jota = 0; jota < SAMPLES_BUFFER; jota++){
				//ka = ADC_BUF_1[jota];
				xQueueSendToBack(queue_intermedia1, &ADC_BUF_1[jota], (TickType_t) 1);
			}
			xSemaphoreGive(queue_intermedia1_ready);
		}
		vTaskDelay(1 / portTICK_RATE_MS);
	}

}

void vMEM_Task(void *pvParameters){		// EN DESARROLLO!!!!!!!!!!!!!!!

	uint16_t jota, ka;	// Actores de reparto

	while(1){

		if( xSemaphoreTake(queue_intermedia1_ready,( TickType_t ) 1) == pdPASS ){			// REVISHAAR TIEMPO DE ESPERA
			for(jota = 0; jota < SAMPLES_BUFFER; jota++){
				xQueueReceive(queue_intermedia1, &ka, (TickType_t) 0);
				xQueueSendToBack(queue_intermedia2, &ka, (TickType_t) 1);
				xSemaphoreGive(queue_intermedia2_ready);			// Lo puse abajo del FOR que copia la queue
			}
			//xSemaphoreGive(queue_intermedia2_ready);
		}
		//vTaskDelay(1 / portTICK_RATE_MS);
	}

}

void vDAC_Task(void *pvParameters){		// EN DESARROLLO!!!!!!!!!!!!!!!

	uint16_t jota, ka;	// Actores de reparto

	while(1){
		if(xSemaphoreTake(queue_intermedia2_ready, (TickType_t) 0) == pdPASS){
			if(xSemaphoreTake(DAC_BUF_0_libre, (TickType_t) 0) == pdPASS){
				for(jota = 0; jota < SAMPLES_BUFFER; jota++){
					xQueueReceive(queue_intermedia2, &ka, (TickType_t) 1); // Successfully received items are removed from the queue.
					DAC_BUF_0[jota] = ka;
				}
			}
			else if(xSemaphoreTake(DAC_BUF_1_libre, (TickType_t) 0) == pdPASS){

				for(jota = 0; jota < SAMPLES_BUFFER; jota++){
					xQueueReceive(queue_intermedia2, &ka, (TickType_t) 1);
					DAC_BUF_1[jota] = ka;
				}
			}
		}
		vTaskDelay(1 / portTICK_RATE_MS);
	}

}
