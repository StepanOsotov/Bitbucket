#ifndef __ADC_INJECTS_CHANNELS_H__
#define __ADC_INJECTS_CHANNELS_H__

//--------------------------------------------------------------------------------

#include "main.h"
#include "delay_us.h"
#include "user_tasks.h"

//--------------------------------------------------------------------------------

#define EVALUATE_AVERAGE_AMOUNT	10

typedef struct
{
#if defined(ADC_INJECT_CHANNELS)
	uint32_t jadcData[ADC_INJECT_CHANNELS];
	float instantaneousValue;
	float avgValue[EVALUATE_AVERAGE_AMOUNT];
	uint8_t avgCount;
	float rmsValue;
	float convertToUSART;
#endif	//ADC_INJECT_CHANNELS
	uint16_t calibrVal;
	volatile uint8_t isStart;
	volatile uint8_t isEnd;
	uint8_t mode;
	uint8_t irq;
	uint8_t watchdog;
	uint8_t user;
	
} adcInject_t;

extern adcInject_t adcInject;

#define MODE_J_SINGLE				1
#define MODE_J_CONTINUOUS		0

//--------------------------------------------------------------------------------

void ADC1_Inject_Config_All_Channel(void);
void ADC1_Inject_Single_Get_Data(void);

//--------------------------------------------------------------------------------

#endif // __ADC_INJECTS_CHANNELS_H__
