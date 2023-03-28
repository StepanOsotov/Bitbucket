#ifndef __DELAY_US_H__
#define __DELAY_US_H__

#include "main.h"
#include "core_cm3.h"

#define TIMEOUT_STILL					120000
#define TIMEOUT_LONG_STILL		300000

#define WORK_WITH_ZEROING_CYCCNT

extern volatile uint32_t timeoutTouchDeviceBegin;
extern volatile uint32_t timeoutTouchDeviceStep;
extern volatile uint8_t flagDisplayOFF;
extern volatile uint8_t flagTouchButton;

void DWT_Init(void);
void delay_us(uint32_t us); // Delay MicroSecond
void delay_ms(uint32_t ms); // Delay MiliSecond

void clear_cyccnt(void);
uint32_t get_cyccnt(void);

void timeOutHandlerCalmState(void);

#endif	//__DELAY_US_H__
