#ifndef COMPASS_H
#define COMPASS_H

typedef enum
{
	COMPASS_BOARD_STATE_INIT,
	COMPASS_BOARD_STATE_BOARD_INIT,
	COMPASS_BOARD_STATE_IDLE,
	COMPASS_BOARD_STATE_IDLE_TIMEOUT,
	COMPASS_BOARD_STATE_COMMAND_MCAL_INIT,
	COMPASS_BOARD_STATE_COMMAND_MCAL_RUN,
	COMPASS_BOARD_STATE_COMMAND_MCAL_FINISH,
	COMPASS_BOARD_STATE_COMMAND_ACAL,
	COMPASS_BOARD_STATE_COMMAND_AUTO,
	COMPASS_BOARD_STATE_ERROR,
	COMPASS_BOARD_STATE_COUNT
} COMPASS_BOARD_STATE;

BOOL compassRun(BOOL init);
COMPASS_BOARD_STATE getCompassState(void);
BOOL getReadContinuous(void);
void setSpeed(BOOL speed);

#endif		// COMPASS_H

