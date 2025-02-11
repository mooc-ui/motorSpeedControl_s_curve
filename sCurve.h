#ifndef __SCURVE_H__
#define __SCURVE_H__


#include "main.h"
#include "arm_math.h"
#include "arm_const_structs.h"


struct S_CURVE_TYPE
{
	float32_t speedTar;
	float32_t lastspeedTar;
	
	float32_t accSet, a_accSet;
	
	float32_t accDir, accTar;
	
	float32_t a_accTemp;
	float32_t a_accStepTemp, d_accStepTemp;

	float32_t acc, j;
	float32_t accT;
	float32_t a_accStep, accStep;		
	
	float32_t out;
	
	uint16_t accCnt, accMaxCnt;
	
	uint8_t sta, acc_dec;

	
	float32_t gain, offset;
	uint8_t useOut;
};

extern struct S_CURVE_TYPE sCurve0;

void SCurveProcess(struct S_CURVE_TYPE *sCurve);

#endif
