/*====== LIBRARIES ======*/
#include	"main.h"			// Header

/*
FIR filter designed with
http://t-filter.appspot.com
sampling frequency: 7994 Hz
fixed point precision: 15 bits
* 0 Hz - 400 Hz
  gain = 1
  desired ripple = 5 dB
  actual ripple = n/a
* 800 Hz - 3997 Hz
  gain = 0
  desired attenuation = -40 dB
  actual attenuation = n/a
*/

#define FILTER_TAP_NUM 28

uint16_t filter_taps[FILTER_TAP_NUM] = {
  -127,  -130,  -157,  -145,  -73,  75,  306,  615, 981,  1369,  1738,
  2042,  2242,  2312,  2242,  2042,  1738,  1369,  981,  615,  306,
  75,  -73,  -145,  -157,  -130,  -127, 0
};

arm_fir_instance_q15 fir_instance;						// Instance structure for the Q15 FIR filter.
uint16_t fir_state[SAMPLES_BUFFER + FILTER_TAP_NUM];		// State buffer

/*====== FUNCTION DECLARATIONS ======*/

void dsp_filter_init(void){

	// The function returns ARM_MATH_SUCCESS if initialization is successful or ARM_MATH_ARGUMENT_ERROR if numTaps is not greater than or equal to 4 and even.
	arm_fir_init_q15(&fir_instance, FILTER_TAP_NUM, &filter_taps[0], &fir_state[0], SAMPLES_BUFFER);

}

void dsp_filter(uint16_t *input, uint16_t *output){

	arm_fir_q15(&fir_instance, &input[0], &output[0], SAMPLES_BUFFER);

}
