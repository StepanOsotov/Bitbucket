#include "timers2and3.h"

//-----------------------------------------------------------------------------

void TIM2_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	TIM2->PSC = 1000;
	TIM2->ARR = 2400;		//100 ms

	TIM2->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM2_IRQn);
	
	TIM2->CR1 |= TIM_CR1_CEN;
}


//----------------------------------------------------------------------------

void TIM3_Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	//HCLK = 24 000 000
	//APB1 Timer Clock = 24 000 000
	TIM3->PSC = 1000;
	//24 000 000 / TIMx->PSC = 24 000 = 1 sec
	//2400 = 100 ms
	TIM3->ARR = 2400 * 2;	//4800 = 200 ms
	//timer count TIMx->CNT [0 .. TIMx->ARR-1]
	//
	//if ((TIM_DIER_UIE == 1) && EnableIRQ(TIMx_IRQn))
	//
	//		if (TIMx->CNT == TIMx->ARR-1)
	//
	//				set TIM_SR_UIF ==>> TIM7_IRQHandler()
	

	TIM3->DIER |= TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM3_IRQn);
	
	TIM3->CR1 |= TIM_CR1_CEN;
}

//-----------------------------------------------------------------------------


void TIM2_IRQHandler(void)
{
	if(TIM2->SR & TIM_SR_UIF)
	{
		TIM2->SR &= ~TIM_SR_UIF;
	}
}

void TIM3_IRQHandler(void)
{
	if(TIM3->SR & TIM_SR_UIF)
	{
		TIM3->SR &= ~TIM_SR_UIF;
	}
}


//----------------------------------------------------------------------------
