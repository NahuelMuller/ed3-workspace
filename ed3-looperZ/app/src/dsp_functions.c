/*====== LIBRARIES ======*/
#include	"main.h"			// Header

/*====== FUNCTION DECLARATIONS ======*/

void dsp_filter(uint16_t *input, uint16_t *output){

	uint16_t jota;	// Actores de reparto

	for(jota = 0; jota < SAMPLES_BUFFER; jota++){
		output[jota] = input[jota];
	}

}
