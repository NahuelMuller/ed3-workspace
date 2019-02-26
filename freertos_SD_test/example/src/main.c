/* Copyright 2014, ChaN
 * Copyright 2016, Matias Marando
 * Copyright 2016-2017, Eric Pernia
 * All rights reserved.
 *
 * This file is part of sAPI Library.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */


/*======================[   ACA ESTA TODO LO DEL MODULO FATFS   ]======================*/
/*==================[   http://elm-chan.org/fsw/ff/00index_e.html   ]==================*/



#include "board.h"

// Includes de FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#include "sd_spi.h"   // <= su propio archivo de cabecera

#include "ff.h"       // <= Biblioteca FAT FS

#include <string.h>   // <= Biblioteca de manejo de Strings, ver:
// https://es.wikipedia.org/wiki/String.h
// http://www.alciro.org/alciro/Programacion-cpp-Builder_12/funciones-cadenas-caracteres-string.h_448.htm

/*==================[definiciones y macros]==================================*/

#define FILENAME  "log0.dat"

/*==================[definiciones de datos internos]=========================*/

static FATFS filesystem;     // <-- FatFs work area needed for each volume
static FIL file;             // <-- File object needed for each open file


/*==================[declaraciones de funciones internas]====================*/


static void stopProgram( void );		// Se termina el programa dejandolo en un loop infinito
static void inicializarSD(void);		// Registra el filesystem object en la SD
static void inicializarArchivo(void);	// Crea un archivo y asigna/prepara/reserva un bloque continuo de memoria



/*==================[declaraciones de funciones externas]====================*/

// Prototipo de funcion de la tarea myTask
void myTask( void* taskParmPtr );

// Prototipo de funcion de la tarea diskTask
void diskTask( void *taskParmPtr );

// Prototipo de funcion de la tarea logTask
void logTask( void *taskParmPtr );

/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main(void)
{

	SystemCoreClockUpdate();	// Update system core clock rate
	Board_Init();	// EDU CIAA: Inicia LEDs y Pulsadores


	/* Set up clock and power for SSP1 module */
	// Configure SSP SSP1 pins
	Chip_SCU_PinMuxSet(0xf, 4, (SCU_MODE_PULLUP | SCU_MODE_FUNC0)); // CLK0
	Chip_SCU_PinMuxSet(0x1, 3, (SCU_MODE_PULLUP | SCU_MODE_INBUFF_EN | SCU_MODE_ZIF_DIS | SCU_MODE_FUNC5)); // MISO1
	Chip_SCU_PinMuxSet(0x1, 4, (SCU_MODE_PULLUP | SCU_MODE_FUNC5)); // MOSI1
	Chip_SCU_PinMuxSet(0x4, 10, (SCU_MODE_PULLUP | SCU_MODE_FUNC4)); // CS1 configured as LCD4
	Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, 5, 14);
	// Initialize SSP Peripheral
	Chip_SSP_Init( LPC_SSP1 );
	Chip_SSP_SetBitRate(LPC_SSP1, 100000);
	Chip_SSP_Enable( LPC_SSP1 );


   // Led para dar señal de vida
   Board_LED_Set(5, TRUE);

   // Crear tarea myTask en freeRTOS
	xTaskCreate(myTask, "myTask", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+1, (TaskHandle_t *) NULL);

   // Crear tarea logTask en freeRTOS
	xTaskCreate(logTask, "logTask", configMINIMAL_STACK_SIZE*4, 0, tskIDLE_PRIORITY+2, (TaskHandle_t *) NULL);

   // Crear tarea diskTask en freeRTOS
	xTaskCreate(diskTask, "diskTask", configMINIMAL_STACK_SIZE*2, 0, tskIDLE_PRIORITY+3, (TaskHandle_t *) NULL);

	vTaskStartScheduler();

   // ---------- REPETIR POR SIEMPRE --------------------------
   while( TRUE )
   {
      //sleepUntilNextInterrupt();
   }

   return 0;
}


/*==================[definiciones de funciones internas]=====================*/

// Se termina el programa dejandolo en un loop infinito
static void stopProgram( void ){
   printf( "Fin del programa.\n" );
   while(TRUE){
      //sleepUntilNextInterrupt();
   }
}


static void inicializarSD(void){		// Registra el filesystem object en la SD

	FRESULT test;

	test = f_mount(&filesystem, "", 1);

	if(test == FR_OK){
		printf("Tarjeta SD montada y lista para trabajar\n");
	} else {
		printf("Error al montar la tarjeta SD\n");
		printf("%d\n", (uint32_t)test);
		stopProgram();
	}

}

static void inicializarArchivo(void){		// Crea un archivo y asigna/prepara/reserva un bloque continuo de memoria

	uint32_t t_0 = 0, t_o = 0, t_e = 0;
	t_0 = xTaskGetTickCount();
	//if(f_open(&file, FILENAME, FA_CREATE_ALWAYS | FA_WRITE | FA_READ) == FR_OK){
	if(f_open(&file, FILENAME, FA_OPEN_ALWAYS | FA_WRITE | FA_READ) == FR_OK){
		t_o = xTaskGetTickCount() - t_0;
		printf("Archivo creado (en %d mS)\n", t_o);
	} else {
		printf("Error al crear el archivo\n");
		stopProgram();
	}

	/*uint32_t size = 510;
	t_0 = xTaskGetTickCount();
	if(f_expand(&file, size, 1) == FR_OK){		// size = bytes
		t_e = xTaskGetTickCount() - t_0;
		printf("Area (%d bytes) contigua asignada (en %d mS)\n", size, t_e);
	} else {
		printf("Error al asignar area contigua\n");
	}*/

}

/*==================[definiciones de funciones externas]=====================*/

// Implementacion de funcion de la tarea myTask
void myTask( void* taskParmPtr )
{
	while(TRUE)
   {
		Board_LED_Toggle(5);
		vTaskDelay( 500 / portTICK_RATE_MS );
	}
}


void diskTask( void *taskParmPtr )
{
	while(TRUE)
   {
		disk_timerproc();   // Disk timer process
		vTaskDelay( 10 / portTICK_RATE_MS );
	}
}


// Implementacion de funcion de la tarea logTask
void logTask( void *taskParmPtr )
{

	uint16_t SAMPLES_BUFFER = 255;
	uint16_t test_out[SAMPLES_BUFFER];
	uint16_t test_in[SAMPLES_BUFFER];
	uint32_t ka, nbytes = 0;

	uint32_t t_0 = 0, t_w = 0, t_s = 0, t_r = 0, t_d = 0;

	for(ka = 0; ka < SAMPLES_BUFFER; ka++){
		test_out[ka] = ka*ka;
	}

	inicializarSD();			// Arranca la SD
	inicializarArchivo();		// Arranca (y overwrite) el archivo

	t_0 = xTaskGetTickCount();

	f_write(&file, test_out, SAMPLES_BUFFER * sizeof(uint16_t), &nbytes);
	/*if(nbytes == SAMPLES_BUFFER * sizeof(uint32_t)){
		printf("Escritura joia\n");
	} else {
		printf("Escritura NO joia\n");
	}*/
	t_w = xTaskGetTickCount() - t_0;

	t_0 = xTaskGetTickCount();
	vTaskDelay( 2000 / portTICK_RATE_MS );		// Esperemos 2 segundos
	t_d = xTaskGetTickCount() - t_0;

	t_0 = xTaskGetTickCount();
	f_lseek(&file, f_tell(&file)-nbytes);
	t_s = xTaskGetTickCount() - t_0;

	t_0 = xTaskGetTickCount();
	f_read(&file, test_in, SAMPLES_BUFFER * sizeof(uint16_t), &nbytes);
	/*if(nbytes == SAMPLES_BUFFER * sizeof(uint32_t)){
		printf("Lectura joia\n");
	} else {
		printf("Lectura NO joia\n");
	}*/
	t_r = xTaskGetTickCount() - t_0;

	f_close(&file);
	printf("Bytes: %d\n", nbytes);
	printf("Write: %d\n", t_w);
	printf("Delay_2s: %d\n", t_d);
	printf("Seek: %d\n", t_s);
	printf("Read: %d\n", t_r);

	vTaskDelay( 2000 / portTICK_RATE_MS );		// Esperemos otros 2 segundos

	for(ka = 0; ka < 6; ka++){
		printf("%d\n", test_in[ka]);
	}

	stopProgram();

}

/*==================[fin del archivo]========================================*/
