/*
 * filter.h
 *
 *  Created on: 4 May 2026
 *      Author: senha
 */

#ifndef INC_FILTER_H_
#define INC_FILTER_H_

typedef struct
{

	float b0,b1,b2,a1,a2;

	float x1_x, x2_x, y1_x, y2_x; // past x
	float x1_y, x2_y, y1_y, y2_y; // past y
	float x1_z, x2_z, y1_z, y2_z; // past z

}BiquadFilter;

void Biquad_Init(BiquadFilter *f);
float Biquad_Update(float input, float *x1, float *x2, float *y1, float *y2, BiquadFilter *coeffs);


#endif /* INC_FILTER_H_ */
