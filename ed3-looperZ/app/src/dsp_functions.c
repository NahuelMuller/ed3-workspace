/*====== LIBRARIES ======*/
#include	"main.h"			// Header

#define FILTER_TAP_NUM 29

/*
//LOW PASS fc = 1KHz
const float32_t filter_taps[FILTER_TAP_NUM] = {		// MATLAB: fir1(28, 1/4) = fir1(orden_filtro, f_corte/(0.5 * f_sampling))
		-0.001822523039f, -0.001587929377f, +0.000000000000f, +0.003697750828f,
		+0.008075430263f, +0.008530221683f, +0.000000000000f, -0.017397698394f,
		-0.034145860704f, -0.033359156473f, +0.000000000000f, +0.067630839471f,
		+0.152206183469f, +0.222924695624f, +0.250496093294f, +0.222924695624f,
		+0.152206183469f, +0.067630839471f, +0.000000000000f, -0.033359156473f,
		-0.034145860704f, -0.017397698394f, +0.000000000000f, 0.008530221683f,
		+0.008075430263f, +0.003697750827f, +0.000000000000f, -0.001587929377f,
		-0.001822523039f
};
*/

//HIGH PASS fc = 100Hz
const float32_t filter_taps[FILTER_TAP_NUM] = {		// MATLAB: fir1(28, 100/(0.5*7993.73), 'high') = fir1(orden_filtro, f_corte/(0.5 * f_sampling), tipo)
		-0.001623970158f, -0.001914954240f, -0.002700136391f, -0.003977249409f,
		-0.005711517871f, -0.007836731895f, -0.010258528748f, -0.012859700415f,
		-0.015507194114f, -0.018060344058f, -0.020379776663f, -0.022336375435f,
		-0.023819680468f, -0.024745132218f, +0.976540633486f, -0.024745132218f,
		-0.023819680468f, -0.022336375435f, -0.020379776663f, -0.018060344058f,
		-0.015507194114f, -0.012859700415f, -0.010258528748f, -0.007836731895f,
		-0.005711517871f, -0.003977249409f, -0.002700136391f, -0.001914954240f,
		-0.001623970158f
};

arm_fir_instance_f32 fir_instance;								// Instance structure for the f32 FIR filter.
static float32_t fir_state[SAMPLES_BUFFER + FILTER_TAP_NUM];	// State buffer
static float32_t buffer1[SAMPLES_BUFFER], buffer2[SAMPLES_BUFFER];

/*====== FUNCTION DECLARATIONS ======*/

void dsp_filter_init(void){

	arm_fir_init_f32(&fir_instance, FILTER_TAP_NUM, (float32_t *) &filter_taps[0], &fir_state[0], SAMPLES_BUFFER);

}

void dsp_filter(uint16_t *input, uint16_t *output){

	arm_q15_to_float((q15_t *) &input[0], buffer1, SAMPLES_BUFFER);
	arm_fir_f32(&fir_instance, buffer1, buffer2, SAMPLES_BUFFER);
	arm_float_to_q15(buffer2, (q15_t *) &output[0], SAMPLES_BUFFER);

}
