#ifndef __FAULT_H
#define __FAULT_H

#include "main.h"

typedef union
{
    struct
    {
        unsigned OverVbusFlag : 1;  // 过压
        unsigned UnderVbusFlag : 1; // 欠压
        unsigned OverTempFlag : 1;  // 过温
        unsigned OverIBUSFlag : 1;  // 过流
        unsigned LockedFlag : 1;    // 堵转
        unsigned OverSpeedFlag : 1; // 超速
        unsigned OffsetFlag : 1;    // 校准异常
    } bit;
    uint16_t Byte;
} DiagFlag;
extern DiagFlag Fault;

void FaultCheck(void);

#endif
