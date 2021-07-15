#ifndef SERIALDEBUG_H
#define SERIALDEBUG_H

// global function prototypes
void serialDebugInit(void);
void serialDebugDoWork(void);
BOOL serialDebugGetMessage(char **bMessage, BYTE *bMessageLength);
void serialDebugSendData(BYTE nData);
void serialDebugInitSetBaud(BOOL speed);
BOOL serialDebugFifoEmtpy(void);
void serialDebugSetWake(BOOL setWake);
BOOL serialDebugGetQuitFlag(void);

#endif		// SERIALDEBUG_H

