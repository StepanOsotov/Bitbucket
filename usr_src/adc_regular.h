#ifndef __ADC_REGULAR_CHANNELS_H__
#define __ADC_REGULAR_CHANNELS_H__

//--------------------------------------------------------------------------------

#include "main.h"
#include "delay_us.h"

//--------------------------------------------------------------------------------

typedef struct
{
#if defined(ADC_REGULAR_CHANNELS)
	uint32_t radcData[ADC_REGULAR_CHANNELS];
#endif	//ADC_REGULAR_CHANNELS
	
	uint32_t *ptrArrStart;
	uint32_t *ptrArrEnd;
	
	uint16_t calibrVal;
	uint16_t readCurrent;
	uint8_t irq;
	uint8_t irqCountRd;
	volatile uint8_t irqIsStart;
	volatile uint8_t irqIsEnd;
	volatile uint8_t irqIsReady;
	uint8_t mode;
	uint8_t trig;
	uint8_t dma;
	uint8_t dmaUser;
	uint8_t dmaEnable;
	volatile uint8_t dmaIsEnd;
	uint8_t watchdog;
	
} adcRegular_t;

extern adcRegular_t adcRegular;

#define MODE_SINGLE				1
#define MODE_CONTINUOUS		0

#define TRIG_CONVERT_SOFT				1

//--------------------------------------------------------------------------------

void ADC1_Regular_Config_All_Channel(void);

void ADC1_Regular_Single_Get_Data(void);

void ADC1_Regular_DMA1_Init(void);
void ADC1_Regular_DMA1_Enable(uint32_t *buffer, uint32_t len);

//--------------------------------------------------------------------------------

#endif // __ADC_REGULAR_CHANNELS_H__
