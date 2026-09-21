#ifndef __CIRCULAR_BUFFER_H__
#define __CIRCULAR_BUFFER_H__

#include "main.h"
#include <string.h>

void Command_AddReadIndex(uint16_t length);
uint8_t Command_Read(uint16_t i);
uint16_t Command_GetLength();
uint16_t Command_GetRemain();
uint8_t Command_Write(uint8_t *data, uint16_t length);
uint16_t Referee_GetFrame(uint8_t *frame);
uint8_t Angle_GetCommand(uint8_t *command);


#endif
