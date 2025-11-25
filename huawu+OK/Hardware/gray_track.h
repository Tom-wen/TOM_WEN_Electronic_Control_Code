#ifndef __gray_track_h_
#define __gray_track_h_
#include "headfile.h"

#define D1 digtal(1)
#define D2 digtal(2)
#define D3 digtal(3)
#define D4 digtal(4)
#define D5 digtal(5)
#define D6 digtal(6)
#define D7 digtal(7)
#define D8 digtal(8)

void gray_init(void);
void GRAY_XUNJI(void);
unsigned char digtal(unsigned char channel);
extern float g_cThisState,g_cLastState;
extern float gray_value;

#endif
