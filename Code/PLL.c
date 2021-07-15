#include <lpc23xx.h>
#include "gpio.h"
#include "PLL.h"
#include "timer.h"

void PLL_CONFIG(BOOL speed)
{
	static BOOL initDone = FALSE;
	
// FROM DATASHEET:
// 1 DISCONNECT PLL WITH FEED
// 2 DISABLE PLL WITH FEED
// 3 CHANGE CPU CLK DIVIDER (W/O PLL)
// 4 WRITE CLK SOURCE SEL REG
// 5 WRITE PLLCFG AND FEED			
// 6 ENABLE PLL WITH FEED
// 7 CHANGE CPPU CLK DIVIDER (W/PLL)
// 8 WAIT FOR LOCK
// 9 CONNECT PLL WITH FEED
					
//************************************	
	if(initDone == FALSE)
	{
		// step 1 - disconnect
		PLLCON &= ~(1 << PLLC);

		// feed
		PLLFEED = 0xAA;
		PLLFEED = 0x55;
		
		// step 2 - disable
		PLLCON &= ~(1 << PLLE);

		// feed
		PLLFEED = 0xAA;
		PLLFEED = 0x55;
		
		//change cpu clk divider to 4: (N + 1) = 4 --> N = 3
		// OUT OF ORDER? STEP  3
		USBCLKCFG = 4;
		
		// reconfigure clocking
		// select IRC step 4
		CLKSRCSEL = 0x00;
	}

	if(speed == TRUE)
	{
		// write PLLCFG.  (multiplier val) STEP 5
		PLLCFG =  (PLL_MULT << MSEL) | (PLL_DIV_72MHZ << NSEL);			
	}
	else
	{
		// write PLLCFG.  (multiplier val) STEP 5
		PLLCFG =  (PLL_MULT_SLOW << MSEL) | (PLL_DIV_72MHZ_SLOW << NSEL);			
		CCLKCFG = CCLKSEL_72MHZ_SLOW;
	}
		
	// feed
	PLLFEED = 0xAA;
	PLLFEED = 0x55;

	// enable PLL STEP 6
	PLLCON = (1 << PLLE) | (1 << PLLC);

	// feed
	PLLFEED = 0xAA;
	PLLFEED = 0x55;
		
//	// STEP 7
//	// CHANGE CPU CLK DIV FOR OPERATION W/PLL
//	CCLKCFG = CCLKSEL_72MHZ; 
	
	// STEP 8
	// wait for lock
	while(!(PLLSTAT & (1 << PLLSTAT_PLOCK))) {}
	
	// 	STEP 9
	// connect with feed
	PLLFEED = 0xAA;
	PLLFEED = 0x55;

	if(speed == TRUE)
	{
		CCLKCFG = CCLKSEL_72MHZ;
	}
	else
	{
		CCLKCFG = CCLKSEL_72MHZ_SLOW;

	}
	
	// done
	//************************************	
	initDone = TRUE;
}

void PLL_CLOCK_SETUP(void)
{
	PCLKSEL0 = (CCLK_OVER_1 << PCLK_UART0); // DEBUG
	PCLKSEL0 |= (CCLK_OVER_1 << PCLK_UART1); // 485
	PCLKSEL1 = (CCLK_OVER_1 << PCLK_UART2); // 485
	PCLKSEL1 |= (CCLK_OVER_1 << PCLK_UART3); // 485

	/////
	// configure the PLL
	/////
	PLL_CONFIG(FALSE);
}
