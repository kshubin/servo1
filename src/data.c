
#include "servo1.h"


//	USART buffer
uint8_t	rxBuff[BUFSZ];
uint16_t	rxPos;
uint16_t	rxPosPrev;

int		parseStage;
uint16_t	chksum;
uint16_t	chVals[20];

uint64_t    mSecCounter;
uint64_t	timeout;

uint32_t	len;
uint32_t	msgLen;

Chann		channelState[20];

