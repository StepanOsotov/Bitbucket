#include "adc_regular.h"

//-------------------------------------------------------------------------------------------------
//------------------------------------------------DATA-ADC-1---------------------------------------
//-------------------------------------------------------------------------------------------------

adcRegular_t adcRegular;

//-------------------------------------------------------------------------------------------------
//------------------------------------------------REGULAR-ADC-1-------------------------------------
//-------------------------------------------------------------------------------------------------
/*
	ADC1
	 - regular use
		PA6 - ADC_IN6
		PA7 - ADC_IN7
		PB0 - ADC_IN8
		PB1 - ADC_IN9
*/
void ADC1_Regular_Config_All_Channel(void)
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
	PORTS_ADC.GPIO_Pin = GPIO_Pin_4 |	//ADC_IN4 (0x04, 0b0100)
											 GPIO_Pin_5 |	//ADC_IN5 (0x05, 0b0101)
											 GPIO_Pin_6 |	//ADC_IN6 (0x06, 0b0110)	
											 GPIO_Pin_7; 	//ADC_IN7 (0x07, 0b0111)
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
	//Bits 23:20 L[3:0]: Regular channel sequence length
	//
#if defined(ADC_REGULAR_CHANNELS)
	ADC1->SQR1 |= (ADC_REGULAR_CHANNELS-1) << 20;
#endif	//ADC_REGULAR_CHANNELS
	
	uint32_t sw_ch = 0;
	
	sw_ch |= (0x4 << 0);
	sw_ch |= (0x5 << 5);
	sw_ch |= (0x6 << 10);
	sw_ch |= (0x7 << 15);
	sw_ch |= (0x8 << 20);
	sw_ch |= (0x9 << 25);
	
	ADC1->SQR3 = sw_ch;
	
	//=========================================================
	
	//Discontinuous mode
#if defined(ADC_REGULAR_CHANNELS)

	//Bits 15:13 DISCNUM[2:0]: Discontinuous mode channel count
	ADC1->CR1 |= 0x00 << 13;
	
#endif	//ADC_REGULAR_CHANNELS
	
	//=========================================================
	
	//Scan mode can be selected by setting the SCAN bit
	ADC1->CR1 &= ~ADC_CR1_SCAN;
	

	//Bit 11 DISCEN: Discontinuous mode on regular channels
	ADC1->CR1 |= ADC_CR1_DISCEN;
	
	//=========================================================
	
	if(MODE_SINGLE == adcRegular.mode)
	{
		//Single conversion mode
		ADC1->CR2 &= ~ADC_CR2_CONT;
	}
	if(MODE_CONTINUOUS == adcRegular.mode)
	{
		//Continuous conversion mode
		ADC1->CR2 |= ADC_CR2_CONT;
	}
	
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
	sample_time |= (cycles << 12);
	sample_time |= (cycles << 15);
	
	ADC1->SMPR2 = sample_time;
	
	//=========================================================
	
	//Bits 19:17 EXTSEL[2:0]: External event select for regular group
	//These bits select the external event used to trigger the start of conversion of a regular group:
	//For ADC1 and ADC2, the assigned triggers are:
	//000: Timer 1 CC1 event
	//001: Timer 1 CC2 event
	//010: Timer 1 CC3 event
	//011: Timer 2 CC2 event
	//100: Timer 3 TRGO event
	//101: Timer 4 CC4 event
	//110: EXTI line 11/TIM8_TRGO event
	//111: SWSTART
	
	if(TRIG_CONVERT_SOFT == adcRegular.trig)
	{
		ADC1->CR2 |= ADC_CR2_EXTSEL;
	}
	
	//Bit 20 EXTTRIG: External trigger conversion mode for regular channels
	ADC1->CR2 |= ADC_CR2_EXTTRIG;
	
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
	adcRegular.calibrVal = ADC1->DR;
	
	//=========================================================
	
	if(adcRegular.irq)
	{
		
		//Bit 5 EOCIE: Interrupt enable for EOC
		ADC1->CR1 |= ADC_CR1_EOCIE;	
		
		NVIC_EnableIRQ(ADC1_2_IRQn);
	}
	
	//=========================================================
	
	if(adcRegular.dma)
	{
		//Bit 8 DMA: Direct memory access mode
		//Only ADC1 and ADC3 can generate a DMA request.
		ADC1->CR2 |= ADC_CR2_DMA;
		
		ADC1_Regular_DMA1_Init();
	}
	
	//=========================================================
	
	
}

//-----------------------------------------------------------------------------

static const uint16_t DELAY_POOL_MS = 2000;

void ADC1_Regular_Single_Get_Data(void)
{
	
	//-------------------------------------------------------
	//-----------------------IRQ+DMA-------------------------
	//-------------------------------------------------------
	

	//
	if(!(ADC1->CR2 & ADC_CR2_ADON))
	{
		ADC1->CR2 |= ADC_CR2_ADON;
	}
	
	//-------------------------------------------------------
	//----------------------PURE-IRQ-------------------------
	//-------------------------------------------------------
	
	if((adcRegular.irq) && (!adcRegular.dma))
	{
		if((!adcRegular.irqIsStart) && (!adcRegular.irqIsEnd))
		{
			adcRegular.irqIsStart = 1;
			adcRegular.irqIsReady = 0;

#if defined(ADC_REGULAR_CHANNELS)
			
			adcRegular.ptrArrStart = &adcRegular.radcData[0];
			adcRegular.ptrArrEnd = &adcRegular.radcData[ADC_REGULAR_CHANNELS];
			
#endif	//ADC_REGULAR_CHANNELS
			
			//Bit 22 SWSTART: Start conversion of regular channels
			ADC1->CR2 |= ADC_CR2_SWSTART;	
		}
		if(adcRegular.irqIsReady)
		{
			adcRegular.irqIsReady = 0;

			*(adcRegular.ptrArrStart++) = ADC1->DR;
			
			if(adcRegular.ptrArrStart >= adcRegular.ptrArrEnd)
			{
				adcRegular.irqIsEnd = 1;
				adcRegular.irqIsStart = 0;
			}
			else
			{
				//Bit 22 SWSTART: Start conversion of regular channels
				ADC1->CR2 |= ADC_CR2_SWSTART;				
			}
		}
		if((!adcRegular.irqIsStart) && (adcRegular.irqIsEnd))
		{
			delay_ms(DELAY_POOL_MS);
			adcRegular.irqIsEnd = 0;
		}
	}
	
	//-------------------------------------------------------
	//----------------------IRQ+DMA--------------------------
	//-------------------------------------------------------
	
	else if((adcRegular.irq) && (adcRegular.dma))
	{
		if(adcRegular.dmaUser)
		{
			if(!adcRegular.dmaEnable)
			{
#if defined(ADC_REGULAR_CHANNELS)
				
				adcRegular.dmaEnable = 1;
				ADC1_Regular_DMA1_Enable(&adcRegular.radcData[0], ADC_REGULAR_CHANNELS);
			
				//Bit 22 SWSTART: Start conversion of regular channels
				ADC1->CR2 |= ADC_CR2_SWSTART;
				
#endif	//ADC_INJECT_CHANNELS				
				
			}
			
		}
		
		if(!adcRegular.dmaUser)
		{
			if(adcRegular.dmaEnable)
			{
				adcRegular.dmaEnable = 0;
				adcRegular.dmaIsEnd = 0;
			}
		}
		
		if(adcRegular.dmaIsEnd)
		{
			adcRegular.dmaIsEnd = 0;
			adcRegular.dmaEnable = 0;
			delay_ms(DELAY_POOL_MS);
		}
		
	}
	
	//-------------------------------------------------------
	//---------------------PURE-POLL-------------------------
	//-------------------------------------------------------
	
	else
	{
		adcRegular.irqCountRd = 0;
		
#if defined(ADC_REGULAR_CHANNELS)
		
		adcRegular.ptrArrStart = &adcRegular.radcData[0];
		adcRegular.ptrArrEnd = &adcRegular.radcData[ADC_REGULAR_CHANNELS];
		
		//while((adcRegular.irqCountRd++) < ADC_REGULAR_CHANNELS)
		while(adcRegular.ptrArrStart < adcRegular.ptrArrEnd)
		{
			//Bit 22 SWSTART: Start conversion of regular channels
			ADC1->CR2 |= ADC_CR2_SWSTART;
			//
			while(!(ADC1->SR & ADC_SR_EOC));
			//It is cleared by software.
			//ADC1->SR &= ~ADC_SR_EOC;
			//adcRegular.radc[adcRegular.countRd++] = ADC1->DR;
			*(adcRegular.ptrArrStart++) = ADC1->DR;
			
			//delay_ms(100);
		}
#endif	//ADC_REGULAR_CHANNELS
		
		delay_ms(DELAY_POOL_MS);
	}
	
}

//-------------------------------------------------------------------------------------------------
//------------------------------------------------DMA-1-------------------------------
//-------------------------------------------------------------------------------------------------

void ADC1_Regular_DMA1_Init(void)
{
	if(!(RCC->AHBENR & RCC_AHBENR_DMA1EN))
	{
		RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	}
	
	//1. Set the peripheral register address in the DMA_CPARx register. The data will be
	//moved from/ to this address to/ from the memory after the peripheral event.
	//DMA channel x peripheral address register (DMA_CPARx) (x = 1..7)
	//--------------------------------------------------------------------------
	//--------------------------PERIPHERAL-ADDRESS------------------------------
	//--------------------------------------------------------------------------
	
	DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);

	//2. Set the memory address in the DMA_CMARx register. The data will be written to or
	//read from this memory after the peripheral event.
	//DMA channel x memory address register (DMA_CMARx) (x = 1..7)
	//--------------------------------------------------------------------------
	//----------------------MEMORY-ADDRESS--------------------------------------
	//--------------------------------------------------------------------------
	
#if defined(ADC_REGULAR_CHANNELS)
	
	DMA1_Channel1->CMAR = (uint32_t)&(adcRegular.radcData);
	
#endif	//ADC_INJECT_CHANNELS

	//3. Configure the total number of data to be transferred in the DMA_CNDTRx register.
	//After each peripheral event, this value will be decremented.
	//DMA channel x number of data register (DMA_CNDTRx) (x = 1..7)
	//--------------------------------------------------------------------------
	//----------------------NUMBER-OF-DATA--------------------------------------
	//--------------------------------------------------------------------------
	
	DMA1_Channel1->CNDTR = 0;

	//--------------------------------------------------------------------------
	//----------------------CHANNEL-CONFIGURATION-REGISTER----------------------
	//--------------------------------------------------------------------------
	
	//Zeroing Register
	DMA1_Channel1->CCR = 0;

	//4. Configure the channel priority using the PL[1:0] bits in the DMA_CCRx register
	//DMA channel x configuration register (DMA_CCRx) (x = 1..7)
	//Bits 13:12 PL[1:0]: Channel priority level
	//00: Low
	//01: Medium
	//10: High
	//11: Very high
	DMA1_Channel1->CCR &= ~DMA_CCR1_PL;

	//5. Configure data transfer direction, circular mode, peripheral & memory incremented
	//mode, peripheral & memory data size, and interrupt after half and/or full transfer in the
	//DMA_CCRx register
	//Bit 14 MEM2MEM: Memory to memory mode
	//0: Memory to memory mode disabled
	//1: Memory to memory mode enabled
	DMA1_Channel1->CCR &= ~DMA_CCR1_MEM2MEM;
	//Bits 11:10 MSIZE[1:0]: Memory size
	//00: 8-bits
	//01: 16-bits
	//10: 32-bits
	//11: Reserved
	DMA1_Channel1->CCR &= ~DMA_CCR1_MSIZE;
	DMA1_Channel1->CCR |= DMA_CCR1_MSIZE_1;
	//Bits 9:8 PSIZE[1:0]: Peripheral size
	//00: 8-bits
	//01: 16-bits
	//10: 32-bits
	//11: Reserved
	DMA1_Channel1->CCR &= ~DMA_CCR1_PSIZE;
	DMA1_Channel1->CCR |= DMA_CCR1_PSIZE_1;
	//Bit 7 MINC: Memory increment mode
	DMA1_Channel1->CCR |= DMA_CCR1_MINC;
	//Bit 6 PINC: Peripheral increment mode
	DMA1_Channel1->CCR &= ~DMA_CCR1_PINC;
	//Bit 5 CIRC: Circular mode
	//In circular mode, after the last transfer, the DMA_CNDTRx register is automatically reloaded
	//with the initially programmed value. The current internal address registers are reloaded with
	//the base address values from the DMA_CPARx/DMA_CMARx registers.
	DMA1_Channel1->CCR &= ~DMA_CCR1_CIRC;
	//Bit 4 DIR: Data transfer direction
	//0: Read from peripheral
	//1: Read from memory
	DMA1_Channel1->CCR &= ~DMA_CCR1_DIR;
	//Half transfer interrupt enable
	//0: HT interrupt disabled
	//1: HT interrupt enabled
	DMA1_Channel1->CCR &= ~DMA_CCR1_HTIE;
	//TCIE: Transfer complete interrupt enable
	//0: TC interrupt disabled
	//1: TC interrupt enabled
	DMA1_Channel1->CCR &= ~DMA_CCR1_TCIE;
	//NVIC
	//NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	NVIC->ISER[0] |= 1<<DMA1_Channel1_IRQn;
	//6. Activate the channel by setting the ENABLE bit in the DMA_CCRx register.
	DMA1_Channel1->CCR &= ~DMA_CCR1_EN;
	
}

//-----------------------------------------------------------------------------

void ADC1_Regular_DMA1_Enable(uint32_t *buffer, uint32_t len)
{
	//Write the USART_DR register address in the DMA control
	//register to configure it as the destination of the transfer.
	//DMA1_Channel7->CPAR = (uint32_t)&(USART2->DR);

	//Write the memory address in the DMA control register
	//to configure it as the source of the transfer.
	DMA1_Channel1->CMAR = (uint32_t)buffer;

	//Configure the total number of bytes to be
	//transferred to the DMA control register.
	DMA1_Channel1->CNDTR = len;

	//Configure the channel priority in the DMA register
	//DMA1_Channel7->CCR &= ~DMA_CCR7_PL;

	//Configure DMA interrupt generation after half/ full
	//transfer as required by the application.
	//Half transfer interrupt enable
	//0: HT interrupt disabled
	//1: HT interrupt enabled
	//DMA1_Channel1->CCR |= DMA_CCR1_HTIE;
	//TCIE: Transfer complete interrupt enable
	//0: TC interrupt disabled
	//1: TC interrupt enabled
	DMA1_Channel1->CCR |= DMA_CCR1_TCIE;
	
	//Activate the channel in the DMA register
	DMA1_Channel1->CCR |= DMA_CCR1_EN;

}

//-----------------------------------------------------------------------------
