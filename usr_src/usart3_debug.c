#include "usart3_debug.h"

//-------------------------------------------------------------------------------------------------
//--------------------------------------------DATA-USART-3-----------------------------------------
//-------------------------------------------------------------------------------------------------

uint8_t isUsart3DmaENtx = 0;
uint8_t isUsart3DmaENrx = 0;
uint8_t isUsart3IrqRxEN = 0;
volatile uint8_t dma_wait_tx_usart3 = 0;
volatile uint8_t dma_wait_rx_usart3 = 0;

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------USART-3-----------------------------------------
//-------------------------------------------------------------------------------------------------
/*
	USART3
		TxD	-	PB10
		RxD -	PB11
*/
void USART3_Debug_Config(uint8_t isDMAtx, uint8_t isDMArx, uint8_t isIrqRx)
{
	if(!(RCC->APB2ENR & RCC_APB2ENR_AFIOEN))
	{
		// AFIO Clock
		RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
	}
	//	PORTB Clock
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	//	USART3 Clock
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	//
	GPIO_InitTypeDef PORT_UART;
	PORT_UART.GPIO_Mode = GPIO_Mode_AF_OD;
	PORT_UART.GPIO_Pin = GPIO_Pin_10;	//UART3 TX
	PORT_UART.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&PORT_UART);

	//PORT_UART.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	PORT_UART.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	PORT_UART.GPIO_Pin = GPIO_Pin_11;	//UART3 RX
	PORT_UART.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&PORT_UART);
	
	//Procedure:
//-----------------------------------------------------------------------------
	//1.	Enable the USART by writing the UE bit in USART_CR1 register to 1.
	USART3->CR1 |= USART_CR1_UE;
//-----------------------------------------------------------------------------
	//2.	Program the M bit in USART_CR1 to define the word length.
	//USART_CR1_M = Clear(12 BIT) : 0 - 8 bit, 1 - 9 bit
	USART3->CR1 &= ~USART_CR1_M;
//-----------------------------------------------------------------------------
	/*3.		Program the number of stop bits in USART_CR2.
	 * USART_CR2_STOP
	(1)13:12(0)
	 * 00 - 1 Stop bit
	 * 01 - 0.5 Stop bit
	 * 10 - 2 Stop bit
	 * 11 - 1.5 Stop bit
	 */
	USART3->CR2 &= ~USART_CR2_STOP;
//-----------------------------------------------------------------------------
	//DMA off
	//4.	Select DMA enable (DMAT) in USART_CR3 if Multi buffer Communication is to take place. 
	//		Configure the DMA register as explained in multibuffer communication.
	if((!isDMAtx) && (!isDMArx))
	{
		USART3->CR3 &= ~USART_CR3_DMAT;
		USART3->CR3 &= ~USART_CR3_DMAR;
	}
	else
	{
		if(1 == isDMAtx)
		{
			//Bit 7 DMAT: DMA enable transmitter
			USART3->CR3 |= USART_CR3_DMAT;
			
			DMA1_Usart3_InitTX();
			
		}
		if(1 == isDMArx)
		{
			
			//Bit 6 DMAR: DMA enable receiver
			USART3->CR3 |= USART_CR3_DMAR;
			
			DMA1_Usart3_InitRX();
		}		
	}

//-----------------------------------------------------------------------------
	//5.	Select the desired baud rate using the USART_BRR register.
	//----
	/* USART3->BRR =
	 *   36MHz
	 * ---------- = 19,53125 = 0x13
	 * 16 * 115200
	 *
	 * 0.53125 * 16 = 8.5 = 0x8
	 * USART3->BRR |= 0x27 << 0;
	 */
	USART3->BRR |= 0x138 << 0;
//-----------------------------------------------------------------------------
	//6.	Set the TE bit in USART_CR1 to send an idle frame as first transmission.
	USART3->CR1 |= USART_CR1_TE;
	//
	while(  (!(USART3->SR & USART_SR_TXE))  );
//-----------------------------------------------------------------------------
	//Set the RE bit USART_CR1. This enables the receiver which
	//begins searching for a start bit.
	USART3->CR1 |= USART_CR1_RE;
	//
	if(/*(!isDMAtx) && (!isDMArx) && */isIrqRx)
	{
		//Bit 5 RXNEIE: RXNE interrupt enable
		USART3->CR1 |= USART_CR1_RXNEIE;	
		
		NVIC_EnableIRQ(USART3_IRQn);
	}
//-----------------------------------------------------------------------------
}

//-----------------------------------------------------------------------------

void USART3_SendByte(const uint8_t data)
{
/*
Single byte communication
Clearing the TXE bit is always performed by a write to the data register.
*/
	USART3->DR = data;
	//
	while(  !(USART3->SR & USART_SR_TXE)  );
/*
The TXE bit is set by hardware and it indicates:
• The data has been moved from DR to the shift register
	and the data transmission has	started.
• The DR register is empty.
• The next data can be written in the USART_DR register
	without overwriting the previous data.
*/
}

//-----------------------------------------------------------------------------

void USART3_SendText(const uint8_t *data,uint16_t size)
{
	if(!isUsart3DmaENtx)
	{
		while(size--)
		{
			USART3_SendByte(*(data++));
		}
	}
	else
	{
		dma_wait_tx_usart3 = 0;
		tx_USART3_with_DMA(data, size);
		while(!dma_wait_tx_usart3);
	}
}

//-----------------------------------------------------------------------------

uint8_t USART3_Rx_Byte(uint8_t *data)
{
	//When a character is received
	//The RXNE bit is set.
	//This bit is set by hardware when the content of the DR shift register
  //has been transferred to the UART_DR register.
	if(USART3->SR & USART_SR_RXNE)
	{
    //0: Data is not received
		//1: Received data is ready to be read.
		*data = USART3->DR;
		return SUCCESS_BYTE_3;
	}
	//
	return NO_DATA_3;
}


//-----------------------------------------------------------------------------
// 1 byte => this byte length all wait rx array byte

int8_t USART3_ReceiveText(uint8_t *data, uint8_t size)
{
	
  static uint8_t countSerial = 0;
  static uint32_t timeNewSerial = 0;
  static uint32_t timeOldSerial = 0;
  
	uint8_t llRxData = 0;
  uint8_t length = 0;
  
	if(size == 0)
  {
    countSerial = 0;
		return NO_DATA_3;
  }
  
  if(countSerial > 0)
  {
    timeNewSerial = get_cyccnt();
    if((timeNewSerial - timeOldSerial) >= THRESHOLD_TIMEOUT_3)
    {
      timeNewSerial = timeOldSerial = get_cyccnt();;
      countSerial = 0;
      return NO_RESPONSE_3;
    }
  }
	
	if(!USART3_Rx_Byte(&llRxData))
	{
		return NO_DATA_3;
	}
  timeNewSerial = timeOldSerial = get_cyccnt();
  
  *(data + countSerial) = llRxData;
	countSerial++;
  length = countSerial;
	if(countSerial >= size)
	{
		//very big data
		countSerial = 0;
	}
  return length;

}

//-----------------------------------------------------------------------------

void USART3_IRQHandler(void)
{
	if(USART3->SR & USART_SR_RXNE)
	{
		if(rxUsartCntByte >= SIZE_DEBUG_STRIG)
		{
			rxUsartCntByte = 0;
		}
		usart3Rx[rxUsartCntByte++] = USART3->DR;
	}
}

//-------------------------------------------------------------------------------------------------
//---------------------------------------------DMA-USART-3-----------------------------------------
//-------------------------------------------------------------------------------------------------

void DMA1_Usart3_InitTX(void)
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
	DMA1_Channel2->CPAR = (uint32_t)&(USART3->DR);

	//2. Set the memory address in the DMA_CMARx register. The data will be written to or
	//read from this memory after the peripheral event.
	//DMA channel x memory address register (DMA_CMARx) (x = 1..7)
	//--------------------------------------------------------------------------
	//----------------------MEMORY-ADDRESS--------------------------------------
	//--------------------------------------------------------------------------
	DMA1_Channel2->CMAR = 0;

	//3. Configure the total number of data to be transferred in the DMA_CNDTRx register.
	//After each peripheral event, this value will be decremented.
	//DMA channel x number of data register (DMA_CNDTRx) (x = 1..7)
	//--------------------------------------------------------------------------
	//----------------------NUMBER-OF-DATA--------------------------------------
	//--------------------------------------------------------------------------
	DMA1_Channel2->CNDTR = 0;

	//
	DMA1_Channel2->CCR = 0;

	//4. Configure the channel priority using the PL[1:0] bits in the DMA_CCRx register
	//DMA channel x configuration register (DMA_CCRx) (x = 1..7)
	//00: Low
	//01: Medium
	//10: High
	//11: Very high
	DMA1_Channel2->CCR &= ~DMA_CCR2_PL;

	//5. Configure data transfer direction, circular mode, peripheral & memory incremented
	//mode, peripheral & memory data size, and interrupt after half and/or full transfer in the
	//DMA_CCRx register
	//Memory to memory mode
	//0: Memory to memory mode disabled
	//1: Memory to memory mode enabled
	DMA1_Channel2->CCR &= ~DMA_CCR2_MEM2MEM;
	//Memory size
	//00: 8-bits
	//01: 16-bits
	//10: 32-bits
	//11: Reserved
	DMA1_Channel2->CCR &= ~DMA_CCR2_MSIZE;
	//Peripheral size
	//00: 8-bits
	//01: 16-bits
	//10: 32-bits
	//11: Reserved
	DMA1_Channel2->CCR &= ~(DMA_CCR2_PSIZE_0 | DMA_CCR2_PSIZE_1);
	//Memory increment mode
	DMA1_Channel2->CCR |= DMA_CCR2_MINC;
	//Peripheral increment mode
	DMA1_Channel2->CCR &= ~DMA_CCR2_PINC;

	//Circular mode
	DMA1_Channel2->CCR &= ~DMA_CCR2_CIRC;

	//Data transfer direction
	//0: Read from peripheral
	//1: Read from memory
	DMA1_Channel2->CCR |= DMA_CCR2_DIR;
	//Half transfer interrupt enable
	//0: HT interrupt disabled
	//1: HT interrupt enabled
	DMA1_Channel2->CCR &= ~DMA_CCR2_HTIE;
	//TCIE: Transfer complete interrupt enable
	//0: TC interrupt disabled
	//1: TC interrupt enabled
	DMA1_Channel2->CCR |= DMA_CCR2_TCIE;
	//NVIC
	//NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	NVIC->ISER[0] |= 1<<DMA1_Channel2_IRQn;
	//6. Activate the channel by setting the ENABLE bit in the DMA_CCRx register.
	DMA1_Channel2->CCR &= ~DMA_CCR2_EN;

}

//-----------------------------------------------------------------------------

void tx_USART3_with_DMA(const uint8_t *buffer, uint32_t len)
{
	//Write the USART_DR register address in the DMA control
	//register to configure it as the destination of the transfer.
	//DMA1_Channel2->CPAR = (uint32_t)&(USART3->DR);

	//Write the memory address in the DMA control register
	//to configure it as the source of the transfer.
	DMA1_Channel2->CMAR = (uint32_t)buffer;

	//Configure the total number of bytes to be
	//transferred to the DMA control register.
	DMA1_Channel2->CNDTR = len;

	//Configure the channel priority in the DMA register
	//DMA1_Channel7->CCR &= ~DMA_CCR7_PL;

	//Configure DMA interrupt generation after half/ full
	//transfer as required by the application.
	//Half transfer interrupt enable
	//0: HT interrupt disabled
	//1: HT interrupt enabled
	//DMA1_Channel7->CCR &= ~DMA_CCR7_HTIE;
	//TCIE: Transfer complete interrupt enable
	//0: TC interrupt disabled
	//1: TC interrupt enabled
	//DMA1_Channel2->CCR |= DMA_CCR2_TCIE;

	//Clear the TC bit in the SR register by writing 0 to it.
	//USART3->SR &= ~USART_SR_TC;

	//Activate the channel in the DMA register
	DMA1_Channel2->CCR |= DMA_CCR2_EN;

}

//-----------------------------------------------------------------------------

void DMA1_Channel2_IRQHandler(void)
{
	//
	if((DMA1->ISR & DMA_ISR_TCIF2) == DMA_ISR_TCIF2)
	{
		//1: Clears the GIF, TEIF, HTIF and TCIF flags in the DMA_ISR register
		DMA1->IFCR |= DMA_IFCR_CGIF2;

		DMA1_Channel2->CCR &= ~DMA_CCR2_EN;
		
		dma_wait_tx_usart3 = 1;
	}
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

void DMA1_Usart3_InitRX(void)
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
	DMA1_Channel3->CPAR = (uint32_t)&(USART3->DR);

	//2. Set the memory address in the DMA_CMARx register. The data will be written to or
	//read from this memory after the peripheral event.
	//DMA channel x memory address register (DMA_CMARx) (x = 1..7)
	//--------------------------------------------------------------------------
	//----------------------MEMORY-ADDRESS--------------------------------------
	//--------------------------------------------------------------------------
	DMA1_Channel3->CMAR = 0;

	//3. Configure the total number of data to be transferred in the DMA_CNDTRx register.
	//After each peripheral event, this value will be decremented.
	//DMA channel x number of data register (DMA_CNDTRx) (x = 1..7)
	//--------------------------------------------------------------------------
	//----------------------NUMBER-OF-DATA--------------------------------------
	//--------------------------------------------------------------------------
	DMA1_Channel3->CNDTR = 0;

	DMA1_Channel3->CCR = 0;
	//DMA1->IFCR |= DMA_IFCR_CTCIF3;

	//4. Configure the channel priority using the PL[1:0] bits in the DMA_CCRx register
	//DMA channel x configuration register (DMA_CCRx) (x = 1..7)
	//00: Low
	//01: Medium
	//10: High
	//11: Very high
	DMA1_Channel3->CCR &= ~DMA_CCR3_PL;

	//5. Configure data transfer direction, circular mode, peripheral & memory incremented
	//mode, peripheral & memory data size, and interrupt after half and/or full transfer in the
	//DMA_CCRx register
	//Memory to memory mode
	//0: Memory to memory mode disabled
	//1: Memory to memory mode enabled
	DMA1_Channel3->CCR &= ~DMA_CCR3_MEM2MEM;
	//Memory size
	//00: 8-bits
	//01: 16-bits
	//10: 32-bits
	//11: Reserved
	DMA1_Channel3->CCR &= ~DMA_CCR3_MSIZE;
	//Peripheral size
	//00: 8-bits
	//01: 16-bits
	//10: 32-bits
	//11: Reserved
	DMA1_Channel3->CCR &= ~(DMA_CCR3_PSIZE_0 | DMA_CCR3_PSIZE_1);
	//Memory increment mode
	DMA1_Channel3->CCR |= DMA_CCR3_MINC;
	//Peripheral increment mode
	DMA1_Channel3->CCR &= ~DMA_CCR3_PINC;

	//Circular mode
	DMA1_Channel3->CCR &= ~DMA_CCR3_CIRC;

	//Data transfer direction
	//0: Read from peripheral
	//1: Read from memory
	DMA1_Channel3->CCR &= ~DMA_CCR3_DIR;
	//Half transfer interrupt enable
	//0: HT interrupt disabled
	//1: HT interrupt enabled
	DMA1_Channel3->CCR &= ~DMA_CCR3_HTIE;
	//TCIE: Transfer complete interrupt enable
	//0: TC interrupt disabled
	//1: TC interrupt enabled
	DMA1_Channel3->CCR |= DMA_CCR3_TCIE;
	//NVIC
	//NVIC_EnableIRQ(DMA1_Channel3_IRQn);
	NVIC->ISER[0] |= 1<<DMA1_Channel3_IRQn;
	//6. Activate the channel by setting the ENABLE bit in the DMA_CCRx register.
	DMA1_Channel3->CCR &= ~DMA_CCR3_EN;

}

//-----------------------------------------------------------------------------

void rx_USART3_with_DMA(uint8_t *const buffer, uint32_t len)
{
	//Write the memory address in the DMA control register
	//to configure it as the source of the transfer.
	DMA1_Channel3->CMAR = (uint32_t)buffer;

	//Configure the total number of bytes to be
	//transferred to the DMA control register.
	DMA1_Channel3->CNDTR = len;

	//Configure DMA interrupt generation after half/ full
	//transfer as required by the application.
	//DMA1_Channel3->CCR |= DMA_CCR3_TCIE;
	//DMA1_Channel3->CCR |= DMA_CCR3_HTIE;

	//Clear the TC bit in the SR register by writing 0 to it.
	//USART3->SR &= ~USART_SR_TC;

	//Activate the channel in the DMA register
	DMA1_Channel3->CCR |= DMA_CCR3_EN;

}

//-----------------------------------------------------------------------------

void DMA1_Channel3_IRQHandler(void)
{

	//static uint8_t countRxDMA = 0;

	if((DMA1->ISR & DMA_ISR_TCIF3) == DMA_ISR_TCIF3)
	{
		//1: Clears the GIF, TEIF, HTIF and TCIF flags in the DMA_ISR register
		DMA1->IFCR |= DMA_IFCR_CGIF3;

		DMA1_Channel3->CCR &= ~DMA_CCR3_EN;
		
		dma_wait_rx_usart3 = 1;
	}
}

//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------
