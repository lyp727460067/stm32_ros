#ifndef _HW
#define _HW
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f0xx.h"

void HwInit(void);

void McuSw5VEnable(uint8_t d);
void McuSw12VEnable(uint8_t d);
#ifdef __cplusplus
}
#endif
#endif