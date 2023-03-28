/**
  ******************************************************************************
  * @file    GPIO/IOToggle/main.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "init_periph.h"
#include "user_tasks.h"
#include "delay_us.h"
//#include "timers2and3.h"

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup GPIO_IOToggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define USE_FULL_ASSERT

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

#if defined(ADC_INJECT_CHANNELS)
#include "adc_inject.h"
#endif	//ADC_INJECT_CHANNELS

#if defined(ADC_REGULAR_CHANNELS)
#include "adc_regular.h"
#endif	//ADC_REGULAR_CHANNELS

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
	/*
	USART3
		PB10	-	TxD
		PB11	-	RxD
	
	ADC1
	 - inject use
		PA6	- ADC_IN6
		PA7 - ADC_IN7
		PB0 - ADC_IN8
		PB1 - ADC_IN9
		EXTSEL - Timer 4 TRGO event
	
	ADC1
	 - regular use
		PA4	- ADC_IN4
		PA5 - ADC_IN5
		PA6	- ADC_IN6
		PA7 - ADC_IN7
		PB0 - ADC_IN8
		PB1 - ADC_IN9
		
	*/
	
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f10x_xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f10x.c file
     */     
	
  /* To achieve GPIO toggling maximum frequency, the following  sequence is mandatory. 
     You can monitor PD0 or PD2 on the scope to measure the output signal. 
     If you need to fine tune this frequency, you can add more GPIO set/reset 
     cycles to minimize more the infinite loop timing.
     This code needs to be compiled with high speed optimization option.
	*/
	
#if defined(DWT_WORK)
	
	DWT_Init();
	const char *simpleString = "Hello world !!!\n";
	uint16_t length = strlen(simpleString);
	
#endif	//DWT_WORK
	
	initClock();
	initDebugLed();
	
	isUsart3DmaENtx = 1;
	isUsart3DmaENrx = 0;
	isUsart3IrqRxEN = 1;
	
	USART3_Debug_Config(
											isUsart3DmaENtx,
											isUsart3DmaENrx,
											isUsart3IrqRxEN);
	if(isUsart3DmaENrx)
	{
		rx_USART3_with_DMA((uint8_t *)usart3Rx, DMA_USART3_RX_COUNT);
	}
	
	//TIM2_Init();
	
	
#if defined(ADC_INJECT_CHANNELS)

	adcInject.mode = MODE_J_SINGLE;
	adcInject.irq = 1;
	adcInject.avgCount = 0;
	ADC1_Inject_Config_All_Channel();
	
#endif	//ADC_INJECT_CHANNELS

#if defined(ADC_REGULAR_CHANNELS)

	adcRegular.mode = MODE_SINGLE;
	adcRegular.trig = TRIG_CONVERT_SOFT;
	adcRegular.irq = 1;
	adcRegular.irqCountRd = 0;
	adcRegular.irqIsStart = 0;
	adcRegular.irqIsEnd = 0;
	adcRegular.dma = 1;
	if(adcRegular.dma)
	{
		adcRegular.irq = 1;
	}
	adcRegular.dmaUser = 1;
	ADC1_Regular_Config_All_Channel();

#endif	//ADC_REGULAR_CHANNELS

#if !defined(DWT_WORK)
		
	xTaskCreate(vTaskLedToggle, (const char *)"taskToggle",
							configMINIMAL_STACK_SIZE, NULL,
							tskIDLE_PRIORITY + 1, NULL);
	
	xTaskCreate(vTaskRxUsartDebug3, (const char *)"RxUsart3",
							configMINIMAL_STACK_SIZE, NULL,
							tskIDLE_PRIORITY + 1, NULL);
	
	USART3_SendText((const uint8_t *)"vTaskStartScheduler\n",
										strlen("vTaskStartScheduler\n"));

  vTaskStartScheduler();  //start FreeRTOS
		
#endif	//DWT_WORK									
	
  while(1)
	{
#if defined(DWT_WORK)
		
		//check work DWT
		//delay_ms(1000);
		//delay_us(1000000);
		//GPIOC->ODR ^= (1<<13);
		
		//USART3_SendText((const uint8_t *)simpleString, length);
		
#if defined(ADC_INJECT_CHANNELS)
		
		ADC1_Inject_Single_Get_Data();
		
#endif	//ADC_INJECT_CHANNELS

#if defined(ADC_REGULAR_CHANNELS)
		
		if(TRIG_CONVERT_SOFT == adcRegular.trig)
		{
			ADC1_Regular_Single_Get_Data();
		}
		
#endif	//ADC_REGULAR_CHANNELS
		
#endif	//DWT_WORK
		
	}
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
