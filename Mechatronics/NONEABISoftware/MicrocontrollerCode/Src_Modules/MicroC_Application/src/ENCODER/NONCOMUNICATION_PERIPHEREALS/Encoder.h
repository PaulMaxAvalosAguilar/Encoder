#ifndef ENCODER_H
#define ENCODER_H

#include <inttypes.h>

extern volatile int16_t encoderCounter;

void initEncoder(void);
void ext_interruptConfig(void);
void timer1Config(void);
int8_t getPositionValues(int *target);

#endif
