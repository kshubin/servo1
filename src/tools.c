

#include	<stm32f10x_usart.h>
#include "servo1.h"
#include "data.h"


void
cleanUSARTBuffer(void)
{
	while (rxPos > 0)	{
	  	rxPos--;
		rxBuff[rxPos]	=	(uint8_t)0;
	}
	rxPosPrev	=	rxPos;
}

int
parseUsartMsg(void)
{
	int	ret	=	0;
	int	curr;
	int	j, i, k;
	uint16_t	len;
	uint16_t	chksum;
	uint16_t	val;
	
	if (rxPos > 256)	{
		curr	=	rxPos - 30;
		
		while (curr > 0)	{
			for (j = curr; j >= 0; j--)	{
				if (rxBuff[j] == 0x40)
					break;
			}
			if (j > 32)
				curr	=	j - 30;
			else
				return	0;
			if (j > 0)	{
				len	=	rxBuff[j - 1];
				if (len >= 0x10 && len < 0x40 && (j + len) <= rxPos)	{
					chksum	=	0xFFFF;
					chksum	-=	rxBuff[j - 1];
					chksum	-=	rxBuff[j];
					for (i = 1; i < (len - 3); i++)	{
						chksum	-=	rxBuff[j + i];
					}
					val	=	rxBuff[j + i + 1];
					val	<<=	8;
					val	+=	rxBuff[j + i];
					if (chksum == val)	{
						k	=	0;
						for (i = 1; i < (len - 3); i++)	{
							val	=	rxBuff[j + i + 1];
							val	<<=	8;
							val	+=	rxBuff[j + i];
							i++;
							channelState[k].chVal	=	val;
							k++;
							if (k > 20)
								break;
						}
						ret	=	ACK;
						break;
					}
				}
			}
		}
		
		cleanUSARTBuffer();
	}
	return	ret;
}

void
doCntrl(void)
{
	int	i;
  
	for (i = 0; i < 20; i++)	{
		if (channelState[i].chVal <= 1500)	{
			channelState[i].chState	=	0;
		}
		else	{
	  		channelState[i].chState	=	1;
		}
  	}
}


