#ifndef __SEN_H
#define __SEN_H

#include "main.h"

void CurCloseloop(float IqRef, float Theta);
void SpeedCloseloop(int16_t SpeedRef, uint8_t UseRamp);
void PosCloseloop(void);
void PositionToZero(void);
void PostionToZeroDouble(void);

#endif
