#ifndef LOWPASS_H
#define LOWPASS_H

#include "main.h"

typedef struct 
{
	float Tf; 
	float y_prev; 
	unsigned long timestamp_prev;  
} LowPassFilter;

extern LowPassFilter  LPF_current,LPF_velocity, LPF_rc;

void LOWPass_Init();
float LowPassFilter_operator(LowPassFilter *Lfi,float x);


#endif
