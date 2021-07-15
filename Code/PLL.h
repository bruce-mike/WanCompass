#ifndef PLL_H 
#define PLL_H
//PLL #defines

//PLLCFG
#define MSEL 0
#define NSEL 16


//PLLCON (These define PLL mode (off/active-no connect/active-connected)
#define PLLE 0	
#define PLLC 1

//PLLSTAT
#define PLLSTAT_PLLE 	24
#define PLLSTAT_PLLC 	25
#define PLLSTAT_PLOCK 26

// (PLL_MULT (M) = 35) (PLL_DIV (N) = 3)  gives fcco=(2 x (M+1) x Fin)/(N + 1) = (2*36*4000000)/4 = 72000000 
#define PLL_MULT 35
// PLL div N = 4, reg holds N-1 = 3
#define PLL_DIV_72MHZ  3	 // for 72000000
#define CCLKSEL_72MHZ  0

// (PLL_MULT (M) = 23) (PLL_DIV (N) = 24)  gives fcco=(2 x (M+1) x Fin)/(N + 1) = (2*24*4000000)/25 = 7680000 
#define PLL_MULT_SLOW 414
// PLL div N = 25, reg holds N-1 = 24
#define PLL_DIV_72MHZ_SLOW  15	 // for 7680000
// 6144000 
#define CCLKSEL_72MHZ_SLOW  25

void PLL_CONFIG(BOOL speed);
void PLL_CLOCK_SETUP(void);
#endif /* end PLL_H */
