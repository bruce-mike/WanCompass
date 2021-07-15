#ifndef SERIAL_H
#define SERIAL_H

void serialInit(void);
void serialDoWork(void);
BOOL serialGetMessage(char **bMessage, BYTE *bMessageLength);
void serialSendData(BYTE nData);
void serialInitSetBaud(BOOL speed);
BOOL serialFifoEmtpy(void);
void serialSetWake(BOOL setWake);
BOOL serialGetQuitFlag(void);

#endif		// SERIAL_H

