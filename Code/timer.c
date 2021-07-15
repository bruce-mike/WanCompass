#include <lpc23xx.h>
#include "gpio.h"
#include "timer.h"
#include "irq.h"

static int nLEDCycles = 0;
static UINT nCurrentMilliseconds = 0;
static BYTE nCurrentEpoch = 0;
static int nSetLEDCycles = 0;

void timerToggleLED(void)
{
#ifdef DEVELOPMENT
	static BOOL bSysLED = FALSE;

	if(bSysLED)
	{
		FIO1CLR |= (1 << GPIO_PIN_uP_AUX_LED);
		bSysLED = FALSE;
	}
	else
	{
		FIO1SET |= (1 << GPIO_PIN_uP_AUX_LED);
		bSysLED = TRUE;
	}
#endif
}

void T0Handler(void)  __irq
{
	// acknowledge interrupt
	T0IR = (1 << T0IR_MR3);
	//	EXTINT = T0INT_CLEAR;

	// TOGGLE UNUSED I/O FOR TESTING
	// that the processor is alive.
	//FIO1PIN ^= (1 << uP_USB_LED);
	// 32b int msecs = 1,192h till rollover ~ 50days
	if(0 == ++nCurrentMilliseconds)
	{
		nCurrentEpoch++;
	}

	if(0 != nSetLEDCycles)
	{
		if(--nLEDCycles <= 0)
		{
			timerToggleLED();
			nLEDCycles = nSetLEDCycles;
		}
	}
	VICVectAddr = 0;		/* Acknowledge Interrupt */
}

// configure int to fire every x millisecs

void timerSpeed(int desired_msecs, BOOL speed)
{
	if(speed == TRUE)
	{		
		// reset counter
		T0TCR = (1 << T0_RESET);
		
		// set match value to int 
		T0MR3 = (BASE_CLK_72MHZ) * (desired_msecs) / (4 * 1000);

		// start counter
		T0TCR = (1 << T0_ENA);
	}
	else
	{	
		// reset counter
		T0TCR = (1 << T0_RESET);
		
		// set match value to int 
		T0MR3 = (BASE_CLK_7680000HZ) * (desired_msecs) / (4 * 1000);

		// start counter
		T0TCR = (1 << T0_ENA);
	}
}

void T0_SETUP_PERIODIC_INT(int desired_msecs)
{
	FIO1SET |= (1 << GPIO_PIN_uP_AUX_LED);

	// turn on periph power
	PCONP |= (1 << PCTIM0);
		
	// set pclock
	PCLKSEL0 |= (CCLK_OVER_4 << PCLK_TIMER0);
	// set to INT and reset on match
		
	T0MCR |= (1 << MR3I) | (1 << MR3R);

	// install handler
	// not sure why void() (void) __irq is necessary.  NXP demo code called for (void(*))?
	install_irq( TIMER0_INT, (void (*)(void)__irq) T0Handler, LOWEST_PRIORITY );

	timerSpeed(desired_msecs, FALSE);
}

void initTimer(TIMERCTL* pTimer)
{
	pTimer->nTimeoutEpoch = 0;
	pTimer->nTimeoutTime = 0;
}

void timerShowLEDHeartbeat(int theSetLEDCycles)
{
	nSetLEDCycles = theSetLEDCycles;
	nLEDCycles = nSetLEDCycles;
}

int getTimerNow()
{
	BYTE nEpoch = nCurrentEpoch;
	UINT nMilliseconds = nCurrentMilliseconds;
	if(nEpoch != nCurrentEpoch)
	{
		/////
		// timer rolled over, so grab it again
		/////
		nEpoch = nCurrentEpoch;
		nMilliseconds = nCurrentMilliseconds;
	}
	return nMilliseconds;
}

void startTimer(TIMERCTL *pTimer, UINT nTimeoutMS)
{
	BYTE nEpoch = nCurrentEpoch;
	UINT nMilliseconds = nCurrentMilliseconds;
	if(nEpoch != nCurrentEpoch)
	{
		/////
		// timer rolled over, so grab it again
		/////
		nEpoch = nCurrentEpoch;
		nMilliseconds = nCurrentMilliseconds;
	}
	pTimer->nTimeoutEpoch = nEpoch;
	pTimer->nTimeoutTime = nMilliseconds;
	if(0 < nTimeoutMS)
	{

		pTimer->nTimeoutTime += nTimeoutMS;
		if(pTimer->nTimeoutTime < nMilliseconds)
		{
			/////
			// our timer rolled over
			// so increment to next epoch
			/////
			pTimer->nTimeoutEpoch++;
		}
	}
}
BOOL isTimerExpired(TIMERCTL* pTimer)
{
	BOOL bRetVal = FALSE;
	BYTE nEpoch = nCurrentEpoch;
	UINT nMilliseconds = nCurrentMilliseconds;

	if(nEpoch != nCurrentEpoch)
	{
		nEpoch = nCurrentEpoch;
		nMilliseconds = nCurrentMilliseconds;
	}
	if(pTimer->nTimeoutEpoch < nEpoch)
	{
		bRetVal = TRUE;
	}
	else if((pTimer->nTimeoutEpoch == nEpoch) && (pTimer->nTimeoutTime < nMilliseconds))
	{
		bRetVal = TRUE;
	}
	return bRetVal;
}

void stopTimer(TIMERCTL* pTimer)
{
	pTimer->nTimeoutEpoch = 0;
	pTimer->nTimeoutTime = 0;
}
void timerInit(void)
{
	nCurrentMilliseconds = 0;
	nCurrentEpoch = 0;
	T0_SETUP_PERIODIC_INT(1);
}
