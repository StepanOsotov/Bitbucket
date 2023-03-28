#include "delay_us.h"
#include "core_cm3.h"

volatile uint32_t timeoutTouchDeviceBegin = 0;
volatile uint32_t timeoutTouchDeviceStep = 0;
volatile uint8_t flagDisplayOFF = 0;
volatile uint8_t flagTouchButton = 0;

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

typedef struct
{
  __IO uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  Control Register */
  __IO uint32_t CYCCNT;                 /*!< Offset: 0x004 (R/W)  Cycle Count Register */
  __IO uint32_t CPICNT;                 /*!< Offset: 0x008 (R/W)  CPI Count Register */
  __IO uint32_t EXCCNT;                 /*!< Offset: 0x00C (R/W)  Exception Overhead Count Register */
  __IO uint32_t SLEEPCNT;               /*!< Offset: 0x010 (R/W)  Sleep Count Register */
  __IO uint32_t LSUCNT;                 /*!< Offset: 0x014 (R/W)  LSU Count Register */
  __IO uint32_t FOLDCNT;                /*!< Offset: 0x018 (R/W)  Folded-instruction Count Register */
  __I  uint32_t PCSR;                   /*!< Offset: 0x01C (R/ )  Program Counter Sample Register */
  __IO uint32_t COMP0;                  /*!< Offset: 0x020 (R/W)  Comparator Register 0 */
  __IO uint32_t MASK0;                  /*!< Offset: 0x024 (R/W)  Mask Register 0 */
  __IO uint32_t FUNCTION0;              /*!< Offset: 0x028 (R/W)  Function Register 0 */
        uint32_t RESERVED0[1U];
  __IO uint32_t COMP1;                  /*!< Offset: 0x030 (R/W)  Comparator Register 1 */
  __IO uint32_t MASK1;                  /*!< Offset: 0x034 (R/W)  Mask Register 1 */
  __IO uint32_t FUNCTION1;              /*!< Offset: 0x038 (R/W)  Function Register 1 */
        uint32_t RESERVED1[1U];
  __IO uint32_t COMP2;                  /*!< Offset: 0x040 (R/W)  Comparator Register 2 */
  __IO uint32_t MASK2;                  /*!< Offset: 0x044 (R/W)  Mask Register 2 */
  __IO uint32_t FUNCTION2;              /*!< Offset: 0x048 (R/W)  Function Register 2 */
        uint32_t RESERVED2[1U];
  __IO uint32_t COMP3;                  /*!< Offset: 0x050 (R/W)  Comparator Register 3 */
  __IO uint32_t MASK3;                  /*!< Offset: 0x054 (R/W)  Mask Register 3 */
  __IO uint32_t FUNCTION3;              /*!< Offset: 0x058 (R/W)  Function Register 3 */
} DWT_Type;


#define DWT_BASE            (0xE0001000UL)                            /*!< DWT Base Address */

#define DWT                 ((DWT_Type       *)     DWT_BASE      )   /*!< DWT configuration struct */

//#define DWT_CONTROL *(volatile uint32_t*)0xE0001000
//#define SCB_DEMCR   *(volatile uint32_t*)0xE000EDFC

void DWT_Init(void)
{
//	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // permit use counter
//	DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;   // run counter
	
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;		// permit use counter
	DWT->CTRL |= 0x1UL;													   // run counter
}

//----------------------------------------------------------------------------

void delay_us(uint32_t us) // Delay MicroSecond
{
	uint32_t us_count_tic =  us * (SystemCoreClock / 1000000);
	
#if defined(WORK_WITH_ZEROING_CYCCNT)
	
	DWT->CYCCNT = 0U; //
	while(DWT->CYCCNT < us_count_tic);
	
#else
	
	uint32_t us_start = get_cyccnt();
	while((get_cyccnt() - us_start) < us_count_tic);
	
#endif	//WORK_WITH_ZEROING_CYCCNT
}

//----------------------------------------------------------------------------

void delay_ms(uint32_t ms) // Delay MiliSecond
{
	uint32_t ms_count_tic =  ms * (SystemCoreClock / 1000);
	
#if defined(WORK_WITH_ZEROING_CYCCNT)
	
	DWT->CYCCNT = 0U; //
	while(DWT->CYCCNT < ms_count_tic);
	
#else
	
	uint32_t ms_start = get_cyccnt();
	while((get_cyccnt() - ms_start) < ms_count_tic);
	
#endif	//WORK_WITH_ZEROING_CYCCNT
}

//----------------------------------------------------------------------------

void clear_cyccnt(void)
{
	DWT->CYCCNT = 0U; // 
}

//----------------------------------------------------------------------------

uint32_t get_cyccnt(void)
{
	return DWT->CYCCNT;
}

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------


//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
