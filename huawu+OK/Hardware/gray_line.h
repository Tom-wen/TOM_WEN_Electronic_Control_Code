#ifndef __GRAY_LINE_H_
#define __GRAY_LINE_H_

#define D1 digtal(1)
#define D2 digtal(2)
#define D3 digtal(3)
#define D4 digtal(4)
#define D5 digtal(5)
#define D6 digtal(6)
#define D7 digtal(7)
#define D8 digtal(8)
extern int ERR;

void gray_init(void);
unsigned char digtal(unsigned char channel);
int read_sensor(void);
void Limit(int *motoA);
float get_line_position(void);


#endif