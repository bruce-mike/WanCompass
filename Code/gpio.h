#ifndef GPIO_H
#define GPIO_H

#define TRUE 1
#define FALSE 0
//==========================
//DATA DIRECTIONS/pin modes
//==========================
#define IN			0
#define OUT			1

#define OFF 0
#define ON 1

typedef unsigned char  BYTE;
typedef unsigned short WORD;
typedef unsigned long  DWORD;
typedef unsigned char  BOOL;
typedef unsigned int   UINT;

#define PACKED __packed

#ifdef MAIN
#define THEBYTE BYTE
#else
#define THEBYTE extern BYTE
#endif
#define PIN_MAPPING(THE_PIN) THEBYTE GPIO_PIN_##THE_PIN; THEBYTE GPIO_PORT_##THE_PIN;

//=====================
// PORT DEFINATIONS
//=====================
PIN_MAPPING( uP_AUX_LED)
PIN_MAPPING( REV2)
PIN_MAPPING( REV1)
PIN_MAPPING( REV0)

//=============================================
//periph clocking
//=============================================
#define CCLK_OVER_4 0			// datasheet p. 63
#define CCLK_OVER_1 1 // not for RTC
#define CCLK_OVER_2 2
#define CCLK_OVER_8 3
#define BASE_CLK_96MHZ   (95232000)
#define BASE_CLK_72MHZ   (72000000)
#define BASE_CLK_36MHZ   (36000000)
#define BASE_CLK_7680000HZ   (7680000)

//===============================
// PERIPHERAL POWER CONTROL BITS
//===============================
// 0 UNUSED
#define PCTIM0   1
#define PCTIM1   2
#define PCUART0  3
#define PCUART1  4
#define PCUART2  24
#define PCUART3  25
// 5 UNUSED
#define PCPWM1   6
#define PCI2C0   7
#define PCSPI    8
#define PCRTC    9
#define PCSSP1  10
#define PCEMC   11
#define PCAD    12
#define PCAN1   13
#define PCAN2   14
// 15 UNUSED
// 16 UNUSED
// 17 UNUSED
// 18 UNUSED
#define PCI2C1  19
// 20 UNUSED
#define PCSSP0  21
#define PCTIM2  22
#define PCTIM3  23
#define PCUART2 24
#define PCUART3 25
#define PCI2C2  26
#define PCI2S   27
#define PCSDC   28
#define PCGPDMA 28
#define PCENET  30
#define PCUSB   31

//=====================
// PERIPH CLOCK BITS
//=====================
#define PCLK_UART0  6
#define PCLK_UART1  8
#define PCLK_UART2 16
#define PCLK_UART3 18
#define PCLK_RTC   18

//==================
// Interrupts
//==================
#define EXT_INT 01

//========================
// Pin Mode Bit Locations
//========================
#define EXTINT0  20
#define EXTINT1  22
#define EXTINT2  24
#define EXTINT3  26

#define EXTINT0_CLEAR 0x00000001
#define EXTINT1_CLEAR 0x00000002
#define EXTINT2_CLEAR 0x00000004
#define EXTINT3_CLEAR 0x0000000

//========================
// SERIAL PORTS
//========================

#define FIFO_ENABLE 0x01

#define THRE_INTERRUPT_ENABLE 0x02
#define RX_DATA_AVAILABLE_INTERRUPT_ENABLE 0x01
#define RDR  0x01
#define THRE 0x20
#define TEMT 0x40

//RS485 MODES
#define XMIT 			0
#define RECV			1

#define SUCCESS 	 1
#define ERROR 		-1

#define EIGHTBITS  	0x03	// LCR select 8 bits
#define DATA_WIDTH  0			// LCR bits 1:0 = data witdth
#define DLAB	   		7    	// LCR bit 7
/////
// BAUD = PCLK/(16*((256*DLM)+DLL)*(1+(divAddValue/MultValue))
// ignoring divAddValue and multValue
// BAUD = PCLK/(16*((256*DLM)+DLL))
// PCLK/BAUD = (16*((256*DLM)+DLL))
// PCLK/(BAUD*16) = (256*DLM)+DLL;
// DLM = (PCLK/(BAUD*16))/256)
// DLL = PCLK/(BAUD*16)-(256*DLM)
/////
#define DLL_9600_96MHZ		 			0x71	//MAGIC #: 9600bps@96000000 CCLK = PCLK	  
#define DLM_9600_96MHZ					2

#define DLL_115200_96MHZ	 			0x34	//MAGIC #: 115200bps@96000000 CCLK = PCLK	  
#define DLM_115200_96MHZ				0

#define DLL_300_72MHZ		 			0x98	//MAGIC #: 300bps@72000000 CCLK = PCLK	  
#define DLM_300_72MHZ					58	

#define DLL_600_72MHZ		 			0x4C	//MAGIC #: 600bps@72000000 CCLK = PCLK	  
#define DLM_600_72MHZ					29	

#define DLL_1200_72MHZ		 			0xA6	//MAGIC #: 1200bps@72000000 CCLK = PCLK	  
#define DLM_1200_72MHZ					14	

#define DLL_2400_72MHZ		 			0x53	//MAGIC #: 2400bps@72000000 CCLK = PCLK	  
#define DLM_2400_72MHZ					7	

#define DLL_4800_72MHZ		 			0xA9	//MAGIC #: 4800bps@72000000 CCLK = PCLK	  
#define DLM_4800_72MHZ					3	

#define DLL_9600_72MHZ		 			0xD4	//MAGIC #: 9600bps@72000000 CCLK = PCLK	  
#define DLM_9600_72MHZ					1	

#define DLL_14400_72MHZ		 			0x38	//MAGIC #: 14400bps@72000000 CCLK = PCLK	  
#define DLM_14400_72MHZ					1	

#define DLL_19200_72MHZ		 			0xEA	//MAGIC #: 19200bps@72000000 CCLK = PCLK	  
#define DLM_19200_72MHZ					0	

#define DLL_19200_7680000HZ		 		25	//MAGIC #: 19200bps@7680000 CCLK = PCLK	  
#define DLM_19200_7680000HZ				0
#define DIVADDVAL_72MHZ					0
#define MULVAL_72MHZ					0
#define DIVADDVAL_7680000HZ				0
#define MULVAL_7680000HZ				0

#define DLL_28800_72MHZ		 			0x9C	//MAGIC #: 28800bps@72000000 CCLK = PCLK	  
#define DLM_28800_72MHZ					0	

#define DLL_38400_72MHZ		 			0x75	//MAGIC #: 38400bps@72000000 CCLK = PCLK	  
#define DLM_38400_72MHZ					0	

#define DLL_57600_72MHZ		 			0x4E	//MAGIC #: 57600bps@72000000 CCLK = PCLK	  
#define DLM_57600_72MHZ					0	

#define DLL_115200_72MHZ	 			0x27	//MAGIC #: 115200bps@72000000 CCLK = PCLK	  
#define DLM_115200_72MHZ				0

#define DLL_125000_72MHZ	 			0x24	//MAGIC #: 125000bps@72000000 CCLK = PCLK	  
#define DLM_125000_72MHZ				0

#define DLL_9600_36MHZ		 			0xE6	//MAGIC #: 9600bps@36000000 CCLK = PCLK	  
#define DLM_9600_36MHZ					0	

#define DLL_19200_36MHZ					0x75	//MAGIC #: 19200bps@36000000 CCLK = PCLK
#define DLM_19200_36MHZ					0

#define DLL_57600_36MHZ					0x27	//MAGIC #: 57600bps@36000000 CCLK = PCLK
#define DLM_57600_36MHZ					0

#define DLL_115200_36MHZ				0x13	//MAGIC #: 115200bps@36000000 CCLK = PCLK
#define DLM_115200_36MHZ				0

#define DLL_125000_36MHZ				0x12	//MAGIC #: 115200bps@36000000 CCLK = PCLK
#define DLM_125000_36MHZ				0

// periph clock
#define PCLK_UART2 	16

// periph power
#define PCUART2 		24	
	
#define GPIO_PIN_MPU9250_INT 17	
#define GPIO_PIN_RX0 3
#define GPIO_PIN_RX3 1

// FUNCTIONS:
void hwGPIOConfig(void);
void hwPetWatchDog(void);
BYTE hwGetRevNumber(void);

#endif		// GPIO_H
