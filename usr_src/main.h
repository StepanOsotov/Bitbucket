#ifndef __MAIN_H__
#define __MAIN_H__
//--------------------------------------------------------------------------------
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//--------------------------------------------------------------------------------

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "portmacro.h"
#include "semphr.h"
#include "projdefs.h"
#include "croutine.h"
#include "event_groups.h"
#include "message_buffer.h"

//--------------------------------------------------------------------------------
#include "stm32f10x.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_usart.h"
#include "misc.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//--------------------------------------------------------------------------------


//#define DWT_WORK

#define ADC_INJECT_CHANNELS	4
//#define ADC_REGULAR_CHANNELS	6

//--------------------------------------------------------------------------------

#endif // __MAIN_H__
