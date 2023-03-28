#ifndef __USART_3_DEBUG_H__
#define __USART_3_DEBUG_H__

#include "main.h"
#include "delay_us.h"
#include "user_tasks.h"

#define NO_RESPONSE_3 (-1)

#define THRESHOLD_TIMEOUT_3  6000

#define NO_DATA_3				0
#define SUCCESS_BYTE_3	1

extern volatile uint8_t dma_wait_tx_usart3;
extern volatile uint8_t dma_wait_rx_usart3;

extern uint8_t isUsart3DmaENtx;
extern uint8_t isUsart3DmaENrx;
extern uint8_t isUsart3IrqRxEN;

void USART3_Debug_Config(uint8_t isDMAtx, uint8_t isDMArx, uint8_t isIrqRx);
void USART3_SendByte(const uint8_t data);
void USART3_SendText(const uint8_t *data,uint16_t size);
uint8_t USART3_Rx_Byte(uint8_t *data);
int8_t USART3_ReceiveText(uint8_t *data, uint8_t size);

void DMA1_Usart3_InitTX(void);
void DMA1_Usart3_InitRX(void);

void tx_USART3_with_DMA(const uint8_t *buffer, uint32_t len);
void rx_USART3_with_DMA(uint8_t *const buffer, uint32_t len);

#endif //__USART_3_DEBUG_H__
