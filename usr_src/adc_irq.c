//--------------------------------------------------------------------------------

#include "main.h"
#include "adc_regular.h"
#include "adc_inject.h"

//-----------------------------------------------------------------------------

void ADC1_2_IRQHandler(void)
{
//======================================================_START_EOC_REGULAR_
#if defined(ADC_REGULAR_CHANNELS)
	//Bit 4 STRT: Regular channel Start flag
	if(ADC1->SR & ADC_SR_STRT)
	{
		if(!adcRegular.dma)
		{
			adcRegular.irqCountRd = 0;
		}
		ADC1->SR &= ~ADC_SR_STRT;
	}
	//Bit 1 EOC: End of conversion
	if(ADC1->SR & ADC_SR_EOC)
	{
		if(!adcRegular.dma)
		{
			adcRegular.irqIsReady = 1;		
		}
		ADC1->SR &= ~ADC_SR_EOC;	
	}
	
	if(adcRegular.dma)
	{
		//Bit 22 SWSTART: Start conversion of regular channels
		ADC1->CR2 |= ADC_CR2_SWSTART;
	}	
	
#endif	//ADC_REGULAR_CHANNELS

//======================================================_START_EOC_INJECT_

#if defined(ADC_INJECT_CHANNELS)
	//Bit 3 JSTRT: Injected channel Start flag
	if(ADC1->SR & ADC_SR_JSTRT)
	{
		ADC1->SR &= ~ADC_SR_JSTRT;
	}
	//Bit 2 JEOC: Injected channel end of conversion
	if(ADC1->SR & ADC_SR_JEOC)
	{
		adcInject.isEnd = 1;

		ADC1->SR &= ~ADC_SR_JEOC;
	}
#endif	//ADC_INJECT_CHANNELS

//======================================================_WATCHDOG_BOTH_
	
	//Bit 0 AWD: Analog watchdog flag
	if(ADC1->SR & ADC_SR_AWD)
	{

#if defined(ADC_REGULAR_CHANNELS)
		adcRegular.watchdog = 1;
#endif	//ADC_REGULAR_CHANNELS

#if defined(ADC_INJECT_CHANNELS)
		
		adcInject.watchdog = 1;
		
#endif	//ADC_INJECT_CHANNELS
		
		ADC1->SR &= ~ADC_SR_AWD;
	}
	
//======================================================
	
}

//-----------------------------------------------------------------------------

#if defined(ADC_REGULAR_CHANNELS)

void DMA1_Channel1_IRQHandler(void)
{
	uint32_t status = DMA1->ISR;
	
	if(status & DMA_ISR_TCIF1)
	{
		//CTCIFx: Channel x transfer complete clear
		//1: Clears the corresponding TCIF flag in the DMA_ISR register
		DMA1->IFCR |= DMA_IFCR_CTCIF1;

#if defined(ADC_REGULAR_CHANNELS)
		adcRegular.dmaIsEnd = 1;
#endif	//ADC_REGULAR_CHANNELS
	}
	
	if(status & DMA_ISR_HTIF1)
	{
		//CHTIFx: Channel x half transfer clear
		//1: Clears the corresponding HTIF flag in the DMA_ISR register
		DMA1->IFCR |= DMA_IFCR_CHTIF1;
	}
	
	if(status & DMA_ISR_GIF1)
	{
		//CGIFx: Channel x global interrupt clear
		//1: Clears the GIF, TEIF, HTIF and TCIF flags in the DMA_ISR register
		DMA1->IFCR |= DMA_IFCR_CGIF1;
		
	}
	
	if(status & DMA_ISR_TEIF1)
	{
		//CTEIFx: Channel x transfer error clear
		//1: Clears the corresponding TEIF flag in the DMA_ISR register
		DMA1->IFCR |= DMA_IFCR_CTEIF1;
		
	}
	
	DMA1_Channel1->CCR &= ~DMA_CCR1_EN;
	
}

#endif	//ADC_REGULAR_CHANNELS

//-----------------------------------------------------------------------------
