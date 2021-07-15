#ifndef TIMER_H
#define TIMER_H
/////////////////////////////////
////// Timer 0 Settings
/////////////////////////////////
#define PCTIM0 				1		// PCONP bit loc
#define PCLK_TIMER0	 	2		// pclksel0 bit(s) loc
#define MR3I 					9  	// MCR bit locations
#define MR3R					10  // MCR bit
#define T0INT_CLEAR  	4 	// VIC int clear bit loc
#define T0_ENA				0		// tcr enable bit loc
#define T0_RESET			1		// tcr enable bit loc
#define T0IR_MR3			3
/////////////////////////////////
////// Timer 1 Settings
/////////////////////////////////
#define PCLK_TIMER1	 	2		// pclksel0 bit(s) loc
#define MR3I 					9  	// MCR1 Interrupt bit location
#define MR3R					10  	// MCR1 Reset bit location
#define T1INT_CLEAR  	4 	// VIC int clear bit loc
#define T1_ENA				0		// tcr enable bit loc
#define T1IR_MR3			3

#define Fcclk	60000000
#define Fpclk	(Fcclk / 4)
#define TIME_INTERVAL	(Fpclk/100 - 1)

typedef struct timerctl
{
	UINT nTimeoutTime;
	BYTE nTimeoutEpoch;
}TIMERCTL;

void initTimer(TIMERCTL* pTimer);
void timerShowLEDHeartbeat(int theSetLEDCycles);
int getTimerNow(void);
void startTimer(TIMERCTL *pTimer, UINT nTimeoutMS);
BOOL isTimerExpired(TIMERCTL* pTimer);
void stopTimer(TIMERCTL* pTimer);
void timerInit(void);
void timerToggleLED(void);
void timerSpeed(int desired_msecs, BOOL speed);

#endif		// TIMER_H
