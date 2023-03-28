#include "adc_inject.h"

//-------------------------------------------------------------------------------------------------
//------------------------------------------------DATA-ADC-1---------------------------------------
//-------------------------------------------------------------------------------------------------

adcInject_t adcInject;

//-------------------------------------------------------------------------------------------------
//------------------------------------------------INJECT-ADC-1-------------------------------------
//-------------------------------------------------------------------------------------------------
/*
	ADC1
	 - inject use
		PA6 - ADC_IN6
		PA7 - ADC_IN7
		PB0 - ADC_IN8
		PB1 - ADC_IN9
*/
void ADC1_Inject_Config_All_Channel(void)
{
	
	//=========================================================
	//
	if(!(RCC->APB2ENR & RCC_APB2ENR_AFIOEN))
	{
		// AFIO Clock
		RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	}
	//The ADCCLK clock provided by the Clock Controller
	//is synchronous with the PCLK2 (APB2clock).
	//	ADC Clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	//	PORTA Clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	//	PORTB Clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	//
	GPIO_InitTypeDef PORTS_ADC;
	PORTS_ADC.GPIO_Mode = GPIO_Mode_AIN;
	PORTS_ADC.GPIO_Pin = GPIO_Pin_6 |	//ADC_IN6 (0x06, 0b0110)
											 GPIO_Pin_7;	//ADC_IN7 (0x07, 0b0111)
	PORTS_ADC.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&PORTS_ADC);
	//
	PORTS_ADC.GPIO_Mode = GPIO_Mode_AIN;
	PORTS_ADC.GPIO_Pin = GPIO_Pin_0 |	//ADC_IN8 (0x08, 0b1000)
											 GPIO_Pin_1;	//ADC_IN9 (0x09, 0b1001)
	PORTS_ADC.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&PORTS_ADC);
	
	//=========================================================
	
	//ADC on-off control
	//The ADC can be powered-on by setting the ADON bit
	ADC1->CR2 &= ~ADC_CR2_ADON;
	
	//=========================================================
	//The injected channels and their order in the conversion sequence
	//must be selected in the ADC_JSQR register.
	//Bits 21:20 JL[1:0]: Injected sequence length
	//00: 1 conversion
	//01: 2 conversions
	//10: 3 conversions
	//11: 4 conversions
	
#if defined(ADC_INJECT_CHANNELS)
	ADC1->JSQR |= (ADC_INJECT_CHANNELS-1) << 20;
#endif	//ADC_INJECT_CHANNELS
	
	uint32_t sw_ch = 0;
	
	sw_ch |= (0x6 << 0);
	sw_ch |= (0x7 << 5);
	sw_ch |= (0x8 << 10);
	sw_ch |= (0x9 << 15);
	
	ADC1->JSQR |= sw_ch;
	
	//=========================================================
	
	if(MODE_J_SINGLE == adcInject.mode)
	{
		//Single conversion mode
		ADC1->CR2 &= ~ADC_CR2_CONT;
	}
	else if(MODE_J_CONTINUOUS == adcInject.mode)
	{
		//Continuous conversion mode
		ADC1->CR2 |= ADC_CR2_CONT;
	}
	
	//=========================================================
	
	//Scan mode can be selected by setting the SCAN bit
	ADC1->CR1 |= ADC_CR1_SCAN;
	
	//=========================================================
	
	//Triggered injection
	ADC1->CR1 &= ~ADC_CR1_JAUTO;
	//If the JAUTO bit is set, then the injected group channels
	//are automatically converted after the regular group channels.
	
	//=========================================================
	
	uint32_t sample_time = 0;
	uint8_t cycles = 0x4;
	//programmable sample time
	//000: 1.5 cycles
	//001: 7.5 cycles
	//010: 13.5 cycles
	//011: 28.5 cycles
	//100: 41.5 cycles
	//101: 55.5 cycles
	//110: 71.5 cycles
	//111: 239.5 cycles
	
	sample_time |= (cycles << 0);
	sample_time |= (cycles << 3);
	sample_time |= (cycles << 6);
	sample_time |= (cycles << 9);
	
	ADC1->SMPR2 = sample_time;
	
	//=========================================================
	
	//Bits 14:12 JEXTSEL[2:0]: External event select for injected group
	//These bits select the external event used to trigger
	//the start of conversion of an injected group:
	//000: Timer 1 TRGO event
	//001: Timer 1 CC4 event
	//010: Timer 2 TRGO event
	//011: Timer 2 CC1 event
	//100: Timer 3 CC4 event
	//101: Timer 4 TRGO event
	//110: EXTI line15/TIM8_CC4 event
	//111: JSWSTART
	
	ADC1->CR2 |= ADC_CR2_JEXTSEL;
	
	//Bit 15 JEXTTRIG: External trigger conversion mode for injected channels
	ADC1->CR2 |= ADC_CR2_JEXTTRIG;
	
	//=========================================================
	
	//ADC on-off control
	//The ADC can be powered-on by setting the ADON bit
	ADC1->CR2 |= ADC_CR2_ADON;
	
	//=========================================================
	
	//Calibration is started by setting the CAL bit in the ADC_CR2 register.
	ADC1->CR2 |= ADC_CR2_CAL;
	//Once calibration is over, the
	//CAL bit is reset by hardware
	while(ADC1->CR2 & ADC_CR2_CAL);
	//
	adcInject.calibrVal = ADC1->DR;
	
	//=========================================================
	
	if(adcInject.irq)
	{
		//Bit 7 JEOCIE: Interrupt enable for injected channels
		ADC1->CR1 |= ADC_CR1_JEOCIE;	
		
		NVIC_EnableIRQ(ADC1_2_IRQn);
	}
	
	//=========================================================
	
}

//-----------------------------------------------------------------------------

void ADC1_Inject_Single_Start(void)
{
	if(!(ADC1->CR2 & ADC_CR2_ADON))
	{
		ADC1->CR2 |= ADC_CR2_ADON;
	}
	//Bit 21 JSWSTART: Start conversion of injected channels
	ADC1->CR2 |= ADC_CR2_JSWSTART;
	//Bit 3 JSTRT: Injected channel Start flag
	//This bit is set by hardware when injected channel group conversion starts.
	while(!(ADC1->SR & ADC_SR_JSTRT));
	//It is cleared by software.
	ADC1->SR &= ~ADC_SR_JSTRT;
	//Bit 2 JEOC: Injected channel end of conversion
	//This bit is set by hardware at the end of all injected group channel conversion.
	while(!(ADC1->SR & ADC_SR_JEOC));
	//It is cleared by software.
	ADC1->SR &= ~ADC_SR_JEOC;
	
}

//-----------------------------------------------------------------------------

static const uint16_t DELAY_POOL_MS = 500;

void ADC1_Inject_Single_Get_Data(void)
{
	
//-------------------------------------------------------
//-----------------------IRQ-----------------------------
//-------------------------------------------------------
	
	if(adcInject.irq)
	{
		if((!adcInject.isStart))
		{
			adcInject.isStart = 1;
			adcInject.isEnd = 0;
			
			//Bit 21 JSWSTART: Start conversion of injected channels
			ADC1->CR2 |= ADC_CR2_JSWSTART;	
		}
		if(adcInject.isEnd)
		{
			
#if defined(ADC_INJECT_CHANNELS)
			
			//The converted data is stored in the 16-bit ADC_DRJ1 register
			
			static volatile uint32_t *pJDRx = &(ADC1->JDR1);
			
			for(uint8_t i = 0; i < ADC_INJECT_CHANNELS; i++)
			{
				*(adcInject.jadcData + i) = *(pJDRx + i);
			}
			
			/*
			adcInject.jadcData[0] = ADC1->JDR1;
			adcInject.jadcData[1] = ADC1->JDR2;
			adcInject.jadcData[2] = ADC1->JDR3;
			adcInject.jadcData[3] = ADC1->JDR4;
			*/
	
#endif	//ADC_INJECT_CHANNELS	
			
			adcInject.isStart = 0;
			adcInject.isEnd = 0;
		}
		if((!adcInject.isStart) && (adcInject.isEnd))
		{
			delay_ms(DELAY_POOL_MS);
		}
	}
	
	//-------------------------------------------------------
	//----------------------POLL-----------------------------
	//-------------------------------------------------------
	
	else
	{
		//
		ADC1_Inject_Single_Start();
		
#if defined(ADC_INJECT_CHANNELS)
		
		//The converted data is stored in the 16-bit ADC_DRJ1 register
		adcInject.jadcData[0] = ADC1->JDR1;
		adcInject.jadcData[1] = ADC1->JDR2;
		adcInject.jadcData[2] = ADC1->JDR3;
		adcInject.jadcData[3] = ADC1->JDR4;
	
#endif	//ADC_INJECT_CHANNELS
		
		delay_ms(DELAY_POOL_MS);
	}
}

//-----------------------------------------------------------------------------
