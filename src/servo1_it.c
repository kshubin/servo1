

#include	<misc.h>
#include	<stm32f10x_tim.h>
#include	<stm32f10x_usart.h>
#include	<stm32f10x_gpio.h>

#include "servo1.h"
#include "data.h"


const	PwmChannel	pwmCh[32]	=	{
		{ GPIOA, GPIO_Pin_1 },
		{ GPIOA, GPIO_Pin_2 },
		{ GPIOA, GPIO_Pin_3 },
		{ GPIOA, GPIO_Pin_4 },
		{ GPIOA, GPIO_Pin_5 },
		{ GPIOA, GPIO_Pin_6 },
		{ GPIOA, GPIO_Pin_7 },
		{ GPIOA, GPIO_Pin_8 },
		{ GPIOA, GPIO_Pin_9 },
		{ GPIOA, GPIO_Pin_10 },
		{ GPIOA, GPIO_Pin_11 },
		{ GPIOA, GPIO_Pin_12 },
		{ GPIOA, GPIO_Pin_15 },
		{ GPIOB, GPIO_Pin_3 },
		{ GPIOB, GPIO_Pin_4 },
		{ GPIOB, GPIO_Pin_5 },
		{ GPIOB, GPIO_Pin_6 },
		{ GPIOB, GPIO_Pin_7 },
		{ GPIOB, GPIO_Pin_8 },
		{ GPIOB, GPIO_Pin_9 },
		{ (GPIO_TypeDef*) 0, 0 },
		{ (GPIO_TypeDef*) 0, 0 },
		{ (GPIO_TypeDef*) 0, 0 },
		{ (GPIO_TypeDef*) 0, 0 },
		{ (GPIO_TypeDef*) 0, 0 },
		{ (GPIO_TypeDef*) 0, 0 },
		{ (GPIO_TypeDef*) 0, 0 },
		{ (GPIO_TypeDef*) 0, 0 },
		{ (GPIO_TypeDef*) 0, 0 },
		{ (GPIO_TypeDef*) 0, 0 },
		{ (GPIO_TypeDef*) 0, 0 },
		{ (GPIO_TypeDef*) 0, 0 }
};




void
USART3_IRQHandler(void)
{
	uint16_t	ch;

	if(USART3->SR & USART_IT_RXNE)	{
		/* Read one byte from the receive data register */
		ch	=	(uint16_t)(USART3->DR & (uint16_t)0x01FF);

		if (rxPos < (BUFSZ - 1))	{
			rxBuff[rxPos] = ((uint8_t)(ch & 0xFF));
			rxPos++;
		}
	}
}


void
TIM3_IRQHandler(void)
{
	int	i;
	PwmChannel	ch;

	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)	{
		/* Clear the IT pending Bit */
		TIM3->SR = (uint16_t)~TIM_IT_Update;
		for (i = 0; i < 20; i++)	{
			ch	=	pwmCh[i];
			if (ch.gpio == (GPIO_TypeDef*) 0)
				break;
			//GPIO_SetBits(ch.gpio, ch.pin);
			ch.gpio->BSRR = ch.pin;
		}
	}
	else	{
		if (TIM_GetITStatus(TIM3, TIM_IT_CC1) != RESET)	{
			/* Clear the IT pending Bit */
			TIM3->SR = (uint16_t)~TIM_IT_CC1;
			for (i = 0; i < 20; i++)	{
				ch	=	pwmCh[i];
				if (ch.gpio == (GPIO_TypeDef*) 0)
					break;
				if (channelState[i].chState == 0)	{
				  	//GPIO_ResetBits(ch.gpio, ch.pin);
					ch.gpio->BRR = ch.pin;
				}
			}
		}
		else	{
			if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET)	{
				/* Clear the IT pending Bit */
  				TIM3->SR = (uint16_t)~TIM_IT_CC2;
				for (i = 0; i < 20; i++)	{
					ch	=	pwmCh[i];
					if (ch.gpio == (GPIO_TypeDef*) 0)
						break;
					//GPIO_ResetBits(ch.gpio, ch.pin);
					ch.gpio->BRR = ch.pin;
				}
			}
		}
	}
}




void
TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)    {
	/* Clear the IT pending Bit */
	TIM2->SR = (uint16_t)~TIM_IT_Update;

    mSecCounter++;
 }
}




