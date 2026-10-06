/*
 * filter.c
 *
 *  Created on: 4 May 2026
 *      Author: senha
 */

#include "filter.h"
void Biquad_Init(BiquadFilter *f)
{
	// 3.2 kHz Sampling, 80 Hz Cutoff, Q=0.707 (Butterworth)
	f->b0 = 0.0055427f;
	f->b1 = 0.0110854f;
	f->b2 = 0.0055427f;
	f->a1 = -1.7786317f;
	f->a2 = 0.8008026f;

	// reset past
	f->x1_x = f->x2_x = f->y1_x = f->y2_x = 0;
	f->x1_y = f->x2_y = f->y1_y = f->y2_y = 0;
	f->x1_z = f->x2_z = f->y1_z = f->y2_z = 0;
}
float Biquad_Update(float input, float *x1, float *x2, float *y1, float *y2, BiquadFilter *coeffs)
{
	// Biquad formule: y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
	float output = coeffs->b0 * input + coeffs->b1 * (*x1) + coeffs->b2 * (*x2) - coeffs->a1 * (*y1) - coeffs->a2 * (*y2);


	//shift past
	*x2 = *x1;
	*x1 = input;
	*y2 = *y1;
	*y1 = output;

	return output;

}
